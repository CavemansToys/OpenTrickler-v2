#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include <u8g2.h>
#include <math.h>

#include "app.h"
#include "app_state.h"
#include "FloatRingBuffer.h"
#include "mini_12864_module.h"
#include "display.h"
#include "scale.h"
#include "motors.h"
#include "charge_mode.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "profile.h"
#include "common.h"
#include "servo_gate.h"
#include "ai_tuning.h"
#include "error.h"
#include "hardware/watchdog.h"


uint8_t charge_weight_digits[] = {0, 0, 0, 0, 0};

charge_mode_config_t charge_mode_config;

// Scale related
extern scale_config_t scale_config;
extern servo_gate_t servo_gate;


const eeprom_charge_mode_data_t default_charge_mode_data = {
    .charge_mode_data_rev = EEPROM_CHARGE_MODE_DATA_REV,

    .coarse_stop_threshold = 5,
    .fine_stop_threshold = 0.03,

    .set_point_sd_margin = 0.02,
    .set_point_mean_margin = 0.02,

    .decimal_places = DP_2,

    // Precharges
    .precharge_enable = false,
    .precharge_time_ms = 1000,
    .precharge_speed_rps = 2,

    // LED related - use explicit union initialization for C++ compatibility
    .neopixel_normal_charge_colour = {._raw_colour = RGB_COLOUR_GREEN},        // green
    .neopixel_under_charge_colour = {._raw_colour = RGB_COLOUR_YELLOW},        // yellow
    .neopixel_over_charge_colour = {._raw_colour = RGB_COLOUR_RED},            // red
    .neopixel_not_ready_colour = {._raw_colour = RGB_COLOUR_BLUE},             // blue

    // AI Tuning time targets
    .coarse_time_target_ms = 10000,
    .total_time_target_ms = 15000,

    // ML data collection enabled by default
    .ml_data_collection_enabled = false,

    // Auto zero on cup return
    .auto_zero_on_cup_return = false,

    // Pulse mode defaults
    .pulse_mode_enabled = false,
    .pulse_threshold = 0.5f,        // Start pulsing when within 0.5 grains (range: 0.3-1.0)
    .pulse_duration_ms = 30,        // 30ms motor burst
    .pulse_wait_ms = 150,           // 150ms wait for scale
};

// Configures
TaskHandle_t scale_measurement_render_task_handler = NULL;
static char title_string[30];

static TickType_t charge_start_tick = 0;
static float last_charge_elapsed_seconds = 0.0f;

// Deferred ML recording data (to avoid flash writes during charge)
static bool ml_record_pending = false;
static float ml_coarse_time_ms = 0.0f;
static float ml_fine_time_ms = 0.0f;

// Deferred AI tuning recording data
static bool ai_tuning_record_pending = false;
static float ai_coarse_time_ms = 0.0f;
static float ai_fine_time_ms = 0.0f;
static float ai_total_time_ms = 0.0f;
static ai_motor_mode_t ai_final_motor_mode = AI_MOTOR_MODE_NORMAL;
static float ai_coarse_kp_used = 0.0f;
static float ai_coarse_kd_used = 0.0f;
static float ai_fine_kp_used = 0.0f;
static float ai_fine_kd_used = 0.0f;

// Menu system
extern AppState_t exit_state;
extern QueueHandle_t encoder_event_queue;
extern neopixel_led_config_t neopixel_led_config;


// Definitions
typedef enum {
    CHARGE_MODE_EVENT_NO_EVENT = (1 << 0),
    CHARGE_MODE_EVENT_UNDER_CHARGE = (1 << 1),
    CHARGE_MODE_EVENT_OVER_CHARGE = (1 << 2),
} ChargeModeEventBit_t;


static void format_elapsed_time(char *buffer, size_t len, TickType_t start_tick) {
    TickType_t now = xTaskGetTickCount();
    uint32_t elapsed_ticks = now - start_tick;

    // Tick to milliseconds
    float elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    snprintf(buffer, len, "%.2f s", elapsed_seconds);
}


void scale_measurement_render_task(void *p) {
    char current_weight_string[WEIGHT_STRING_LEN];
    char time_buffer[16];

    u8g2_t *display_handler = get_display_handler();

    while (true) {
        TickType_t last_render_tick = xTaskGetTickCount();

        acquire_display_buffer_access();

        u8g2_ClearBuffer(display_handler);

        // Set font for title and timer
        u8g2_SetFont(display_handler, u8g2_font_helvB08_tr);

        // Format the timer string based on current state
        if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
            format_elapsed_time(time_buffer, sizeof(time_buffer), charge_start_tick);
        } else if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_REMOVAL ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_CUP_RETURN ||
                   charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_ZERO) {
            snprintf(time_buffer, sizeof(time_buffer), "%.2f s", last_charge_elapsed_seconds);
        } else {
            snprintf(time_buffer, sizeof(time_buffer), "--.- s");
        }

        // Calculate x positions
        uint8_t screen_width = u8g2_GetDisplayWidth(display_handler);
        uint8_t time_width = u8g2_GetStrWidth(display_handler, time_buffer);

        // Draw title on left
        u8g2_DrawStr(display_handler, 5, 10, title_string);

        // Draw timer on right edge
        u8g2_DrawStr(display_handler, screen_width - time_width - 5, 10, time_buffer);  // 5 px padding from edge

        // Draw line under title
        u8g2_DrawHLine(display_handler, 0, 13, screen_width);

        // Current weight (only show values > -1.0)
        memset(current_weight_string, 0x0, sizeof(current_weight_string));
        float scale_measurement = scale_get_current_measurement();
        if (scale_measurement > -1.0) {
            float_to_string(current_weight_string, sizeof(current_weight_string), scale_measurement, charge_mode_config.eeprom_charge_mode_data.decimal_places);
        } else {
            strcpy(current_weight_string, "---");
        }

        // Draw current weight value
        u8g2_SetFont(display_handler, u8g2_font_profont22_tf);
        u8g2_DrawStr(display_handler, 26, 35, current_weight_string);

        // Draw profile name
        profile_t *current_profile = profile_get_selected();
        u8g2_SetFont(display_handler, u8g2_font_helvR08_tr);
        u8g2_DrawStr(display_handler, 5, 61, current_profile->name);

        u8g2_SendBuffer(display_handler);

        release_display_buffer_access();

        vTaskDelayUntil(&last_render_tick, pdMS_TO_TICKS(20));
    }
}


void charge_mode_wait_for_zero() {
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );
    
    // Wait for 5 measurements and wait for stable
    FloatRingBuffer data_buffer(10);

    // Update current status
    snprintf(title_string, sizeof(title_string), "Waiting for Zero");

    // Stop condition: 10 stable measurements in 200ms apart (2 seconds minimum)
    while (true) {
        TickType_t last_measurement_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement (max delay 300 seconds   )
        float current_measurement;
        if (scale_block_wait_for_next_measurement(300, &current_measurement)){
            data_buffer.enqueue(current_measurement);
        }

        // Generate stop condition
        if (data_buffer.getCounter() >= 10){
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin && 
                fabsf(data_buffer.getMean()) < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin) {
                break;
            }
        }

        // Wait for minimum 300 ms (but can skip if previously wait already)
        vTaskDelayUntil(&last_measurement_tick, pdMS_TO_TICKS(300));
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_COMPLETE;
}

void charge_mode_wait_for_complete() {

    charge_start_tick = xTaskGetTickCount();
    TickType_t coarse_end_tick = 0;  // Track when coarse phase ends
    TickType_t fine_start_tick = 0;  // Track when fine phase starts

    // Track PID params used for AI tuning telemetry
    float coarse_kp_used = 0, coarse_kd_used = 0;
    float fine_kp_used = 0, fine_kd_used = 0;

    // Set colour to under charge
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
        true
    );

    // If the servo gate is used then it has to be opened
    if (servo_gate.gate_state != GATE_DISABLED) {
        servo_gate_set_state(GATE_OPEN, false);
    }

    // Update current status
    char target_weight_string[WEIGHT_STRING_LEN];
    float_to_string(target_weight_string, sizeof(target_weight_string), charge_mode_config.target_charge_weight, charge_mode_config.eeprom_charge_mode_data.decimal_places);

    snprintf(title_string, sizeof(title_string), 
             "Target: %s", 
             target_weight_string);

    // Read trickling parameter from the current profile
    profile_t * current_profile = profile_get_selected();

    // Find the minimum of max speed from the motor and the profile
    float coarse_trickler_max_speed = fmin(get_motor_max_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_max_flow_speed_rps);
    float coarse_trickler_min_speed = fmax(get_motor_min_speed(SELECT_COARSE_TRICKLER_MOTOR),
                                           current_profile->coarse_min_flow_speed_rps);
    float fine_trickler_max_speed = fmin(get_motor_max_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_max_flow_speed_rps);
    float fine_trickler_min_speed = fmax(get_motor_min_speed(SELECT_FINE_TRICKLER_MOTOR),
                                         current_profile->fine_min_flow_speed_rps);

    float integral = 0.0f;
    float last_error = 0.0f;
    // Anti-windup: clamp integral so ki*integral stays within motor speed bounds
    const float integral_max = coarse_trickler_max_speed * 10.0f;

    TickType_t last_sample_tick = xTaskGetTickCount();
    TickType_t current_sample_tick = last_sample_tick;
    bool should_coarse_trickler_move = true;

    // Phase 2 precharge: Fill pan to coarse threshold using tuned coarse PID from Phase 1
    ai_motor_mode_t initial_motor_mode = ai_tuning_get_motor_mode();
    if (initial_motor_mode == AI_MOTOR_MODE_FINE_ONLY) {
        printf("AI Tuning Phase 2: Precharging with tuned coarse PID...\n");

        // Get the tuned coarse params from Phase 1
        ai_tuning_session_t* session = ai_tuning_get_session();
        float tuned_coarse_kp = session->recommended_coarse_kp;
        float tuned_coarse_kd = session->recommended_coarse_kd;

        float precharge_target = charge_mode_config.target_charge_weight -
                                 charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold;

        float precharge_integral = 0.0f;
        float precharge_last_error = 0.0f;
        TickType_t precharge_last_tick = xTaskGetTickCount();

        int precharge_scale_fail_count = 0;
        while (true) {
            float current_weight;
            if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
                precharge_scale_fail_count++;
                if (precharge_scale_fail_count >= 10) {
                    // Scale disconnected for ~2 seconds - emergency stop
                    motor_set_speed(SELECT_BOTH_MOTOR, 0);
                    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
                    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);
                    if (servo_gate.gate_state != GATE_DISABLED) {
                        servo_gate_set_state(GATE_CLOSE, false);
                    }
                    report_error(ERR_SCALE_DRIVER_SELECT);
                    charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                    return;
                }
                continue;
            }
            precharge_scale_fail_count = 0;

            float precharge_error = precharge_target - current_weight;

            if (precharge_error < charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold) {
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
                // Pause between coarse and fine to let scale settle
                vTaskDelay(pdMS_TO_TICKS(1000));
                fine_start_tick = xTaskGetTickCount();  // Track when fine phase starts
                printf("AI Tuning Phase 2: Precharge complete at %.3f\n", current_weight);
                break;
            }

            // Use tuned coarse PID from Phase 1
            TickType_t now = xTaskGetTickCount();
            float dt_ms = (now - precharge_last_tick) * portTICK_PERIOD_MS;
            precharge_integral += precharge_error;
            // Anti-windup: clamp integral to prevent runaway accumulation
            const float PRECHARGE_INTEGRAL_MAX = 50.0f;
            precharge_integral = fmaxf(-PRECHARGE_INTEGRAL_MAX, fminf(precharge_integral, PRECHARGE_INTEGRAL_MAX));
            float derivative = (dt_ms > 0.0f) ? (precharge_error - precharge_last_error) / dt_ms : 0.0f;

            float new_p = tuned_coarse_kp * precharge_error;
            float new_i = current_profile->coarse_ki * precharge_integral;
            float new_d = tuned_coarse_kd * derivative;
            float pid_output = new_p + new_i + new_d;
            if (pid_output <= 0.0f) {
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            } else {
                float new_speed = fmax(coarse_trickler_min_speed, fmin(pid_output, coarse_trickler_max_speed));
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, new_speed);
            }

            precharge_last_tick = now;
            precharge_last_error = precharge_error;

            // Check for user abort
            ButtonEncoderEvent_t button_event = button_wait_for_input(false);
            if (button_event == BUTTON_RST_PRESSED) {
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
                charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                return;
            }
        }

        // fine_start_tick already set when precharge completed
    }

    int scale_fail_count = 0;
    while (true) {
        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Run the PID controlled loop to start charging
        // Perform the measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            scale_fail_count++;
            if (scale_fail_count >= 10) {
                // Scale disconnected for ~2 seconds - emergency stop
                motor_set_speed(SELECT_BOTH_MOTOR, 0);
                motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
                motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);
                if (servo_gate.gate_state != GATE_DISABLED) {
                    servo_gate_set_state(GATE_CLOSE, false);
                }
                report_error(ERR_SCALE_DRIVER_SELECT);
                charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
                return;
            }
            continue;
        }
        scale_fail_count = 0;
        current_sample_tick = xTaskGetTickCount();

        float error = charge_mode_config.target_charge_weight - current_weight;

        // Check if AI tuning is active and get motor mode
        bool ai_tuning_active = ai_tuning_is_active();
        ai_motor_mode_t motor_mode = ai_tuning_get_motor_mode();

        float coarse_kp, coarse_kd, fine_kp, fine_kd;
        if (ai_tuning_active) {
            // Try to get AI tuning params - if mutex is busy, use profile values
            if (!ai_tuning_get_next_params(&coarse_kp, &coarse_kd, &fine_kp, &fine_kd)) {
                // Use thread-safe read to avoid race with REST API
                profile_get_pid_params(&coarse_kp, &coarse_kd, &fine_kp, &fine_kd);
            }
        } else {
            // Use thread-safe read to avoid race with REST API
            profile_get_pid_params(&coarse_kp, &coarse_kd, &fine_kp, &fine_kd);
        }

        // Validate PID parameters - reject NaN/Inf to prevent motor lockup
        if (isnanf(coarse_kp) || isinff(coarse_kp)) coarse_kp = 0.0f;
        if (isnanf(coarse_kd) || isinff(coarse_kd)) coarse_kd = 0.0f;
        if (isnanf(fine_kp) || isinff(fine_kp)) fine_kp = 0.0f;
        if (isnanf(fine_kd) || isinff(fine_kd)) fine_kd = 0.0f;

        // Track params used for telemetry
        coarse_kp_used = coarse_kp;
        coarse_kd_used = coarse_kd;
        fine_kp_used = fine_kp;
        fine_kd_used = fine_kd;

        // Stop conditions based on motor mode
        if (motor_mode == AI_MOTOR_MODE_COARSE_ONLY) {
            // Phase 1: Stop when coarse threshold reached (don't run fine)
            if (error <= charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold) {
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
                motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
                break;
            }
        } else {
            // Normal or Phase 2: Stop when fine threshold reached
            if (error <= charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold) {
                motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
                break;
            }
        }

        // Update PID variables
        float elapse_time_ms = (current_sample_tick - last_sample_tick) * portTICK_PERIOD_MS;
        integral += error;
        // Anti-windup: clamp integral to prevent excessive accumulation
        if (integral > integral_max) integral = integral_max;
        if (integral < -integral_max) integral = -integral_max;
        // Protect against division by zero if ticks haven't advanced
        float derivative = (elapse_time_ms > 0.0f) ? (error - last_error) / elapse_time_ms : 0.0f;

        // Motor control based on mode
        if (motor_mode == AI_MOTOR_MODE_COARSE_ONLY) {
            // Phase 1: Only coarse runs, fine is OFF
            motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);

            float new_p = coarse_kp * error;
            float new_i = current_profile->coarse_ki * integral;
            float new_d = coarse_kd * derivative;
            float pid_output = new_p + new_i + new_d;
            if (pid_output <= 0.0f) {
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
            } else {
                float new_speed = fmax(coarse_trickler_min_speed, fmin(pid_output, coarse_trickler_max_speed));
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, new_speed);
            }
        }
        else if (motor_mode == AI_MOTOR_MODE_FINE_ONLY) {
            // Phase 2: Only fine runs, coarse is OFF
            motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);

            // Check if pulse mode should be used
            bool use_pulse = charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled &&
                             error < charge_mode_config.eeprom_charge_mode_data.pulse_threshold &&
                             error > charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold;

            if (use_pulse) {
                // Pulse mode: short burst, then wait for scale
                float pulse_speed = fmax(fine_trickler_min_speed, fine_trickler_max_speed * 0.3f);
                motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, pulse_speed);
                vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms));
                motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
                vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms));
            } else {
                // Normal PID control
                float new_p = fine_kp * error;
                float new_i = current_profile->fine_ki * integral;
                float new_d = fine_kd * derivative;
                float pid_output = new_p + new_i + new_d;
                if (pid_output <= 0.0f) {
                    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
                } else {
                    float new_speed = fmax(fine_trickler_min_speed, fmin(pid_output, fine_trickler_max_speed));
                    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, new_speed);
                }
            }
        }
        else {
            // Normal mode: coarse then fine
            if (should_coarse_trickler_move) {
                // Coarse phase - fine motor OFF
                motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);

                if (error < charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold) {
                    // Coarse done, switch to fine
                    should_coarse_trickler_move = false;
                    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
                    // Pause between coarse and fine to let scale settle
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    // Reset PID state for fine phase
                    integral = 0.0f;
                    last_error = error;
                    coarse_end_tick = xTaskGetTickCount();
                    fine_start_tick = coarse_end_tick;
                } else {
                    // Run coarse motor
                    float new_p = coarse_kp * error;
                    float new_i = current_profile->coarse_ki * integral;
                    float new_d = coarse_kd * derivative;
                    float pid_output = new_p + new_i + new_d;
                    if (pid_output <= 0.0f) {
                        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
                    } else {
                        float new_speed = fmax(coarse_trickler_min_speed, fmin(pid_output, coarse_trickler_max_speed));
                        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, new_speed);
                    }
                }
            } else {
                // Fine phase - coarse motor OFF
                motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);

                // Check if pulse mode should be used (close to target, helps slow scales)
                // Note: Pulse mode now works during AI tuning as well
                bool use_pulse = charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled &&
                                 error < charge_mode_config.eeprom_charge_mode_data.pulse_threshold &&
                                 error > charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold;

                if (use_pulse) {
                    // Pulse mode: short burst, then wait for scale
                    float pulse_speed = fmax(fine_trickler_min_speed, fine_trickler_max_speed * 0.3f);
                    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, pulse_speed);
                    vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms));
                    motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
                    vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms));
                } else {
                    // Normal PID control
                    float new_p = fine_kp * error;
                    float new_i = current_profile->fine_ki * integral;
                    float new_d = fine_kd * derivative;
                    float pid_output = new_p + new_i + new_d;
                    if (pid_output <= 0.0f) {
                        motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, 0);
                    } else {
                        float new_speed = fmax(fine_trickler_min_speed, fmin(pid_output, fine_trickler_max_speed));
                        motor_set_speed(SELECT_FINE_TRICKLER_MOTOR, new_speed);
                    }
                }
            }
        }

        // Record state
        last_sample_tick = current_sample_tick;
        last_error = error;
    }

    // Stop the timer
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed_ticks = now - charge_start_tick;
    last_charge_elapsed_seconds = (float)(elapsed_ticks * portTICK_PERIOD_MS) / 1000.0f;

    // Close the gate immediately to prevent gravity-fed powder flow
    if (servo_gate.gate_state != GATE_DISABLED) {
        servo_gate_set_state(GATE_CLOSE, true);
    }

    // Calculate timing for AI tuning telemetry
    float coarse_time_ms = 0.0f;
    float fine_time_ms = 0.0f;
    float total_time_ms = last_charge_elapsed_seconds * 1000.0f;

    ai_motor_mode_t final_motor_mode = ai_tuning_get_motor_mode();
    if (final_motor_mode == AI_MOTOR_MODE_COARSE_ONLY) {
        // Phase 1: All time was coarse
        coarse_time_ms = total_time_ms;
        fine_time_ms = 0.0f;
    } else if (final_motor_mode == AI_MOTOR_MODE_FINE_ONLY) {
        // Phase 2: Precharge time + fine time
        if (fine_start_tick > 0) {
            coarse_time_ms = (float)((fine_start_tick - charge_start_tick) * portTICK_PERIOD_MS);
            fine_time_ms = (float)((now - fine_start_tick) * portTICK_PERIOD_MS);
        } else {
            coarse_time_ms = 0.0f;
            fine_time_ms = total_time_ms;
        }
    } else if (coarse_end_tick > 0) {
        // Normal mode: split timing
        coarse_time_ms = (float)((coarse_end_tick - charge_start_tick) * portTICK_PERIOD_MS);
        fine_time_ms = (float)((now - fine_start_tick) * portTICK_PERIOD_MS);
    }

    // Defer AI tuning recording to cup removal phase (need settled scale reading)
    if (ai_tuning_is_active()) {
        ai_tuning_record_pending = true;
        ai_coarse_time_ms = coarse_time_ms;
        ai_fine_time_ms = fine_time_ms;
        ai_total_time_ms = total_time_ms;
        ai_final_motor_mode = final_motor_mode;
        ai_coarse_kp_used = coarse_kp_used;
        ai_coarse_kd_used = coarse_kd_used;
        ai_fine_kp_used = fine_kp_used;
        ai_fine_kd_used = fine_kd_used;
    }
    else {
        // Not in tuning mode - defer ML recording to cup removal phase
        // (to avoid slow flash writes blocking the charge loop)
        if (charge_mode_config.eeprom_charge_mode_data.ml_data_collection_enabled) {
            ml_record_pending = true;
            ml_coarse_time_ms = coarse_time_ms;
            ml_fine_time_ms = fine_time_ms;
        }
    }

    // Precharge
    if (charge_mode_config.eeprom_charge_mode_data.precharge_enable && servo_gate.gate_state != GATE_DISABLED) {
        // Set a fixed delay between closing the gate and precharge to allow the gate to fully close
        vTaskDelay(pdMS_TO_TICKS(500));

        // Start the pre-charge
        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps);
        vTaskDelay(pdMS_TO_TICKS(charge_mode_config.eeprom_charge_mode_data.precharge_time_ms));

        motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR, 0);
    }
    else {
        vTaskDelay(pdMS_TO_TICKS(20));  // Wait for other tasks to complete  
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_REMOVAL;
}

void charge_mode_wait_for_cup_removal() {
    // Update current status
    snprintf(title_string, sizeof(title_string), "Remove Cup");

    FloatRingBuffer data_buffer(5);

    // Post charge analysis (while waiting for removal of the cup)
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for other tasks to complete

    // Take current measurement
    float current_measurement = scale_get_current_measurement();
    float error = charge_mode_config.target_charge_weight - current_measurement;

    // Deferred ML recording - do it now while user is removing cup
    // This avoids flash writes during the active charge loop
    if (ml_record_pending) {
        ml_record_pending = false;
        profile_t* current_profile = profile_get_selected();
        uint8_t profile_idx = profile_get_selected_idx();
        float overthrow = current_measurement - charge_mode_config.target_charge_weight;

        ai_tuning_record_charge(profile_idx,
                                 current_profile->coarse_kp, current_profile->coarse_kd,
                                 current_profile->fine_kp, current_profile->fine_kd,
                                 overthrow, ml_coarse_time_ms, ml_fine_time_ms);
    }

    // Deferred AI tuning recording - scale has now settled after 1000ms wait
    if (ai_tuning_record_pending) {
        ai_tuning_record_pending = false;

        // Calculate overthrow relative to the correct target for current phase
        float effective_target;
        if (ai_final_motor_mode == AI_MOTOR_MODE_COARSE_ONLY) {
            // Phase 1: Target is coarse stop point (target - coarse_threshold)
            effective_target = charge_mode_config.target_charge_weight -
                              charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold;
        } else {
            // Phase 2 or normal: Target is final weight
            effective_target = charge_mode_config.target_charge_weight;
        }

        float overthrow = current_measurement - effective_target;
        float overthrow_percent = (effective_target > 0.0f) ?
                                  (100.0f * overthrow / effective_target) : 0.0f;

        ai_drop_telemetry_t telemetry = {0};
        telemetry.drop_number = ai_tuning_get_session()->drops_completed + 1;
        telemetry.coarse_time_ms = ai_coarse_time_ms;
        telemetry.fine_time_ms = ai_fine_time_ms;
        telemetry.total_time_ms = ai_total_time_ms;
        telemetry.final_weight = current_measurement;
        telemetry.target_weight = charge_mode_config.target_charge_weight;
        telemetry.overthrow = overthrow;
        telemetry.overthrow_percent = overthrow_percent;
        telemetry.coarse_kp_used = ai_coarse_kp_used;
        telemetry.coarse_kd_used = ai_coarse_kd_used;
        telemetry.fine_kp_used = ai_fine_kp_used;
        telemetry.fine_kd_used = ai_fine_kd_used;

        ai_tuning_record_drop(&telemetry);

        printf("AI Tuning: Drop %d - weight=%.3f target=%.3f error=%.3fg time=%.1fms\n",
               telemetry.drop_number, current_measurement, effective_target, overthrow, ai_total_time_ms);

        // Check if AI tuning is now complete or errored
        if (ai_tuning_is_complete()) {
            printf("AI Tuning: COMPLETE! Exiting charge mode for user to review results.\n");
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        if (ai_tuning_get_session()->state == AI_TUNING_ERROR) {
            printf("AI Tuning: ERROR - %s\n", ai_tuning_get_session()->error_message);
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
    }

    // Update LED colour before moving to the next stage
    // Over charged
    if (error <= -charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold) {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour, 
            true
        );

        // Set over charge
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_OVER_CHARGE;
    }
    // Under charged
    else if (error >= charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold) {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour, 
            true
        );

        // Set under charge flag
        charge_mode_config.charge_mode_event |= CHARGE_MODE_EVENT_UNDER_CHARGE;

    }
    // Normal
    else {
        neopixel_led_set_colour(
            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour, 
            true
        );

        // Clear over and under charge bit
        charge_mode_config.charge_mode_event &= ~(CHARGE_MODE_EVENT_UNDER_CHARGE | CHARGE_MODE_EVENT_OVER_CHARGE);
    }

    // Stop condition: 5 stable measurements in 300ms apart (1.5 seconds minimum)
    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }
        data_buffer.enqueue(current_weight);

        // Generate stop condition
        if (data_buffer.getCounter() >= 5) {
            if (data_buffer.getSd() < charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin &&
                data_buffer.getMean() + 10 < charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin){
                break;
            }
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(300));
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_CUP_RETURN;
}

void charge_mode_wait_for_cup_return() { 
    // Set colour to not ready
    neopixel_led_set_colour(
        neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour, 
        true
    );

    snprintf(title_string, sizeof(title_string), "Return Cup");


    FloatRingBuffer data_buffer(5);

    while (true) {
        TickType_t last_sample_tick = xTaskGetTickCount();

        // Non block waiting for the input
        ButtonEncoderEvent_t button_encoder_event = button_wait_for_input(false);
        if (button_encoder_event == BUTTON_RST_PRESSED) {
            charge_mode_config.charge_mode_state = CHARGE_MODE_EXIT;
            return;
        }
        else if (button_encoder_event == BUTTON_ENCODER_PRESSED) {
            scale_config.scale_handle->force_zero();
        }

        // Perform measurement
        float current_weight;
        if (!scale_block_wait_for_next_measurement(200, &current_weight)) {
            // If no measurement within 200ms then poll the button and retry
            continue;
        }

        if (current_weight >= 0) {
            break;
        }

        // Wait for next measurement
        vTaskDelayUntil(&last_sample_tick, pdMS_TO_TICKS(20));
    }

    // Auto zero scale if enabled
    if (charge_mode_config.eeprom_charge_mode_data.auto_zero_on_cup_return) {
        scale_config.scale_handle->force_zero();
    }

    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;
}


uint8_t charge_mode_menu(bool charge_mode_skip_user_input) {
    // Create target weight, if the charge mode weight is built by charge_weight_digits
    if (!charge_mode_skip_user_input) {
        switch (charge_mode_config.eeprom_charge_mode_data.decimal_places) {
            case DP_2:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 100 + \
                                                charge_weight_digits[3] * 10 + \
                                                charge_weight_digits[2] * 1 + \
                                                charge_weight_digits[1] * 0.1 + \
                                                charge_weight_digits[0] * 0.01;
                break;
            case DP_3:
                charge_mode_config.target_charge_weight = charge_weight_digits[4] * 10 + \
                                                charge_weight_digits[3] * 1 + \
                                                charge_weight_digits[2] * 0.1 + \
                                                charge_weight_digits[1] * 0.01 + \
                                                charge_weight_digits[0] * 0.001;
                break;
            default:
                charge_mode_config.target_charge_weight = 0;
                break;
        }
    }

    // If the display task is never created then we shall create one, otherwise we shall resume the task
    if (scale_measurement_render_task_handler == NULL) {
        // The render task shall have lower priority than the current one
        UBaseType_t current_task_priority = uxTaskPriorityGet(xTaskGetCurrentTaskHandle());
        xTaskCreate(scale_measurement_render_task, "Scale Measurement Render Task", configMINIMAL_STACK_SIZE, NULL, current_task_priority - 1, &scale_measurement_render_task_handler);
    }
    else {
        vTaskResume(scale_measurement_render_task_handler);
    }

    // Enable motor on entering the charge mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, true);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, true);
    
    charge_mode_config.charge_mode_state = CHARGE_MODE_WAIT_FOR_ZERO;

    bool quit = false;
    while (quit == false) {
        switch (charge_mode_config.charge_mode_state) {
            case CHARGE_MODE_WAIT_FOR_ZERO:
                charge_mode_wait_for_zero();
                break;
            case CHARGE_MODE_WAIT_FOR_COMPLETE:
                charge_mode_wait_for_complete();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_REMOVAL:
                charge_mode_wait_for_cup_removal();
                break;
            case CHARGE_MODE_WAIT_FOR_CUP_RETURN:
                charge_mode_wait_for_cup_return();
                break;
            case CHARGE_MODE_EXIT:
            default:
                quit = true;
                break;
        }
    }

    // Reset LED to default colour
    neopixel_led_set_colour(neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.mini12864_backlight_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led1_colour,
                            neopixel_led_config.eeprom_neopixel_led_metadata.default_led_colours.led2_colour,
                            true);

    // vTaskDelete(scale_measurement_render_handler);
    vTaskSuspend(scale_measurement_render_task_handler);

    // Diable motors on exiting the mode
    motor_enable(SELECT_COARSE_TRICKLER_MOTOR, false);
    motor_enable(SELECT_FINE_TRICKLER_MOTOR, false);

    return 1;  // return back to main menu
}


bool charge_mode_config_init(void) {
    bool is_ok = true;

    // Read charge mode config from EEPROM
    memset(&charge_mode_config, 0x0, sizeof(charge_mode_config));
    is_ok = eeprom_read(EEPROM_CHARGE_MODE_BASE_ADDR, (uint8_t *)&charge_mode_config.eeprom_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    if (!is_ok) {
        printf("Unable to read from EEPROM at address %x, using defaults\n", EEPROM_CHARGE_MODE_BASE_ADDR);
        report_error(ERR_CHARGE_EEPROM_READ);
        memcpy(&charge_mode_config.eeprom_charge_mode_data, &default_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    }
    else if (charge_mode_config.eeprom_charge_mode_data.charge_mode_data_rev != EEPROM_CHARGE_MODE_DATA_REV) {
        memcpy(&charge_mode_config.eeprom_charge_mode_data, &default_charge_mode_data, sizeof(eeprom_charge_mode_data_t));

        // Write back
        if (!charge_mode_config_save()) {
            printf("Unable to write to %x\n", EEPROM_CHARGE_MODE_BASE_ADDR);
            report_error(ERR_CHARGE_EEPROM_WRITE);
        }
    }

    // Register to eeprom save all
    eeprom_register_handler(charge_mode_config_save);

    return true;
}


bool charge_mode_config_save(void) {
    bool is_ok = eeprom_write(EEPROM_CHARGE_MODE_BASE_ADDR, (uint8_t *) &charge_mode_config.eeprom_charge_mode_data, sizeof(eeprom_charge_mode_data_t));
    return is_ok;
}



bool http_rest_charge_mode_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // c1 (str): neopixel_normal_charge_colour
    // c2 (str): neopixel_under_charge_colour
    // c3 (str): neopixel_over_charge_colour
    // c4 (str): neopixel_not_ready_colour

    // c5 (float): coarse_stop_threshold
    // c6 (float): fine_stop_threshold
    // c7 (float): set_point_sd_margin
    // c8 (float): set_point_mean_margin
    // c9 (int): decimal point enum
    // c10 (bool): precharge_enable
    // c11 (int): precharge_time_ms
    // c12 (float): precharge_speed_rps
    // c13 (int): coarse_time_target_ms
    // c14 (int): total_time_target_ms
    // c15 (bool): ml_data_collection_enabled

    // ee (bool): save to eeprom

    static char charge_mode_json_buffer[400];  // Increased for pulse mode (c17-c20)
    bool save_to_eeprom = false;

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "c5") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c6") == 0) {
            charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c7") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c8") == 0) {
            charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "c9") == 0) {
            charge_mode_config.eeprom_charge_mode_data.decimal_places = (decimal_places_t) atoi(values[idx]);
        }
        
        // Pre charge related settings
        else if (strcmp(params[idx], "c10") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_enable = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c11") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_time_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c12") == 0) {
            charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps = strtof(values[idx], NULL);
        }

        // AI Tuning time targets
        else if (strcmp(params[idx], "c13") == 0) {
            charge_mode_config.eeprom_charge_mode_data.coarse_time_target_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c14") == 0) {
            charge_mode_config.eeprom_charge_mode_data.total_time_target_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c15") == 0) {
            charge_mode_config.eeprom_charge_mode_data.ml_data_collection_enabled = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c16") == 0) {
            charge_mode_config.eeprom_charge_mode_data.auto_zero_on_cup_return = string_to_boolean(values[idx]);
        }

        // Pulse mode settings
        else if (strcmp(params[idx], "c17") == 0) {
            charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "c18") == 0) {
            float val = strtof(values[idx], NULL);
            charge_mode_config.eeprom_charge_mode_data.pulse_threshold = fmaxf(0.3f, fminf(1.0f, val));
        }
        else if (strcmp(params[idx], "c19") == 0) {
            charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms = strtol(values[idx], NULL, 10);
        }
        else if (strcmp(params[idx], "c20") == 0) {
            charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms = strtol(values[idx], NULL, 10);
        }

        // LED related settings
        else if (strcmp(params[idx], "c1") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c2") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c3") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "c4") == 0) {
            charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour = hex_string_to_decimal(values[idx]);
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }
    
    // Perform action
    if (save_to_eeprom) {
        charge_mode_config_save();
    }

    // Response
    snprintf(charge_mode_json_buffer,
             sizeof(charge_mode_json_buffer),
             "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n"
             "{\"c1\":\"#%06lx\",\"c2\":\"#%06lx\",\"c3\":\"#%06lx\",\"c4\":\"#%06lx\","
             "\"c5\":%.3f,\"c6\":%.3f,\"c7\":%.3f,\"c8\":%.3f,\"c9\":%d,\"c10\":%s,\"c11\":%ld,\"c12\":%0.3f,"
             "\"c13\":%ld,\"c14\":%ld,\"c15\":%s,\"c16\":%s,"
             "\"c17\":%s,\"c18\":%.3f,\"c19\":%ld,\"c20\":%ld}",
             charge_mode_config.eeprom_charge_mode_data.neopixel_normal_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_under_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_over_charge_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.neopixel_not_ready_colour._raw_colour,
             charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.fine_stop_threshold,
             charge_mode_config.eeprom_charge_mode_data.set_point_sd_margin,
             charge_mode_config.eeprom_charge_mode_data.set_point_mean_margin,
             charge_mode_config.eeprom_charge_mode_data.decimal_places,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.precharge_enable),
             charge_mode_config.eeprom_charge_mode_data.precharge_time_ms,
             charge_mode_config.eeprom_charge_mode_data.precharge_speed_rps,
             charge_mode_config.eeprom_charge_mode_data.coarse_time_target_ms,
             charge_mode_config.eeprom_charge_mode_data.total_time_target_ms,
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.ml_data_collection_enabled),
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.auto_zero_on_cup_return),
             boolean_to_string(charge_mode_config.eeprom_charge_mode_data.pulse_mode_enabled),
             charge_mode_config.eeprom_charge_mode_data.pulse_threshold,
             charge_mode_config.eeprom_charge_mode_data.pulse_duration_ms,
             charge_mode_config.eeprom_charge_mode_data.pulse_wait_ms);

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_rest_charge_mode_state(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings
    // s0 (float): Charge weight set point (unitless)
    // s1 (float): Current weight (unitless)
    // s2 (charge_mode_state_t | int): Charge mode state
    // s3 (uint32_t): Charge mode event
    // s4 (string): Profile Name
    // s5 (string): Elapsed time in seconds, live during charging

    static char charge_mode_json_buffer[160];  // Increased to fit s5
    char elapsed_time_buffer[16] = {0};

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "s0") == 0) {
            charge_mode_config.target_charge_weight = strtof(values[idx], NULL);
        }
        else if (strcmp(params[idx], "s2") == 0) {
            charge_mode_state_t new_state = (charge_mode_state_t) atoi(values[idx]);

            // Exit
            if (new_state == CHARGE_MODE_EXIT && charge_mode_config.charge_mode_state != CHARGE_MODE_EXIT) {
                ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }
            // Enter
            else if (new_state == CHARGE_MODE_WAIT_FOR_ZERO && charge_mode_config.charge_mode_state == CHARGE_MODE_EXIT) {
                // Set exit_status for the menu
                exit_state = APP_STATE_ENTER_CHARGE_MODE_FROM_REST;

                // Then signal the menu to stop
                ButtonEncoderEvent_t button_event = OVERRIDE_FROM_REST;
                xQueueSend(encoder_event_queue, &button_event, portMAX_DELAY);
            }

            charge_mode_config.charge_mode_state = new_state;
        }
    }

    // Handle the special case
    float current_measurement = scale_get_current_measurement();
    char weight_string[16];
    if (isnanf(current_measurement)) {
        snprintf(weight_string, sizeof(weight_string), "\"nan\"");
    }
    else if (isinff(current_measurement)) {
        snprintf(weight_string, sizeof(weight_string), "\"inf\"");
    }
    else {
        snprintf(weight_string, sizeof(weight_string), "%0.3f", current_measurement);
    }

    // Format elapsed time
    if (charge_mode_config.charge_mode_state == CHARGE_MODE_WAIT_FOR_COMPLETE) {
        TickType_t now = xTaskGetTickCount();
        float elapsed_seconds = (float)((now - charge_start_tick) * portTICK_PERIOD_MS) / 1000.0f;
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", elapsed_seconds);
    } else {
        snprintf(elapsed_time_buffer, sizeof(elapsed_time_buffer), "%.2f", last_charge_elapsed_seconds);
    }

    // Capture event flags before clearing to avoid race with charge mode task
    uint32_t captured_event = charge_mode_config.charge_mode_event;
    charge_mode_config.charge_mode_event = 0;

    // Response
    snprintf(charge_mode_json_buffer,
             sizeof(charge_mode_json_buffer),
             "%s"
             "{\"s0\":%0.3f,\"s1\":%s,\"s2\":%d,\"s3\":%lu,\"s4\":\"%s\",\"s5\":\"%s\"}",
             http_json_header,
             charge_mode_config.target_charge_weight,
             weight_string,
             (int) charge_mode_config.charge_mode_state,
             captured_event,
             profile_get_selected()->name,
             elapsed_time_buffer);

    size_t data_length = strlen(charge_mode_json_buffer);
    file->data = charge_mode_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}