/**
 * @file encoder.cpp
 * @brief Rotary encoder and button input handling
 *
 * Shared encoder/button code used by both Mini 12864 and TFT35/TFT43 displays.
 */

#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <queue.h>

#include "pico/stdlib.h"

#include "configuration.h"
#include "gpio_irq_handler.h"
#include "encoder.h"
#include "display_config.h"
#include "error.h"
#include "common.h"

// Encoder event queue
QueueHandle_t encoder_event_queue = NULL;

// Cached encoder inversion for ISR-safe access
static volatile bool cached_inverted_encoder = false;

// ISR: Encoder rotation
static void _isr_on_encoder_update(uint gpio, uint32_t event) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t state = 2;
    static int8_t count = 0;

    bool en1 = gpio_get(BUTTON0_ENCODER_PIN1);
    bool en2 = gpio_get(BUTTON0_ENCODER_PIN2);

    switch (state) {
        case 0: {
            if (en1) {
                count += 1;
                state = 1;
            }
            else if (en2) {
                count -= 1;
                state = 3;
            }
            break;
        }
        case 1: {
            if (!en1) {
                count -= 1;
                state = 0;
            }
            else if (en2) {
                count += 1;
                state = 2;
            }
            break;
        }
        case 2: {
            if (!en1) {
                count += 1;
                state = 3;
            }
            else if (!en2) {
                count -= 1;
                state = 1;
            }
            break;
        }
        case 3: {
            if (!en1) {
                count += 1;
                state = 0;
            }
            else if (en2) {
                count -= 1;
                state = 2;
            }
            break;
        }
        default:
            break;
    }

    ButtonEncoderEvent_t button_encoder_event;
    if (count >= 4) {
        count = 0;

        if (cached_inverted_encoder) {
            button_encoder_event = BUTTON_ENCODER_ROTATE_CCW;
        }
        else {
            button_encoder_event = BUTTON_ENCODER_ROTATE_CW;
        }

        if (encoder_event_queue) {
            xQueueSendFromISR(encoder_event_queue, &button_encoder_event, &xHigherPriorityTaskWoken);
        }
    }
    else if (count <= -4) {
        count = 0;

        if (cached_inverted_encoder) {
            button_encoder_event = BUTTON_ENCODER_ROTATE_CW;
        }
        else {
            button_encoder_event = BUTTON_ENCODER_ROTATE_CCW;
        }

        if (encoder_event_queue) {
            xQueueSendFromISR(encoder_event_queue, &button_encoder_event, &xHigherPriorityTaskWoken);
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ISR: Encoder button press
static void _isr_on_button_enc_update(uint gpio, uint32_t event) {
    if (encoder_event_queue == NULL) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static TickType_t last_call_time = 0;
    const TickType_t debounce_timeout_ticks = pdMS_TO_TICKS(250);

    // Button debounce
    TickType_t current_call_time = xTaskGetTickCountFromISR();
     if ((current_call_time - last_call_time) > debounce_timeout_ticks) {
        ButtonEncoderEvent_t button_encoder_event = BUTTON_ENCODER_PRESSED;
        xQueueSendFromISR(encoder_event_queue, &button_encoder_event, &xHigherPriorityTaskWoken);
    }
    last_call_time = current_call_time;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ISR: Reset button press
static void _isr_on_button_rst_update(uint gpio, uint32_t event) {
    if (encoder_event_queue == NULL) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static TickType_t last_call_time = 0;
    const TickType_t debounce_timeout_ticks = pdMS_TO_TICKS(250);

    // Button debounce
    TickType_t current_call_time = xTaskGetTickCountFromISR();
    if ((current_call_time - last_call_time) > debounce_timeout_ticks) {
        ButtonEncoderEvent_t button_encoder_event = BUTTON_RST_PRESSED;
        xQueueSendFromISR(encoder_event_queue, &button_encoder_event, &xHigherPriorityTaskWoken);
    }
    last_call_time = current_call_time;

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void button_init(void) {
    printf("Initializing Encoder/Button -- ");

    // Configure encoder pins
    gpio_init(BUTTON0_ENCODER_PIN1);
    gpio_set_dir(BUTTON0_ENCODER_PIN1, GPIO_IN);
    gpio_pull_up(BUTTON0_ENCODER_PIN1);

    gpio_init(BUTTON0_ENCODER_PIN2);
    gpio_set_dir(BUTTON0_ENCODER_PIN2, GPIO_IN);
    gpio_pull_up(BUTTON0_ENCODER_PIN2);

    // Configure encoder button
    gpio_init(BUTTON0_ENC_PIN);
    gpio_set_dir(BUTTON0_ENC_PIN, GPIO_IN);
    gpio_pull_up(BUTTON0_ENC_PIN);

    // Configure reset button
    gpio_init(BUTTON0_RST_PIN);
    gpio_set_dir(BUTTON0_RST_PIN, GPIO_IN);
    gpio_pull_up(BUTTON0_RST_PIN);

    // Create event queue
    encoder_event_queue = xQueueCreate(5, sizeof(ButtonEncoderEvent_t));
    if (encoder_event_queue == 0) {
        printf("ERROR: Failed to create encoder event queue\n");
        report_error(ERR_DISPLAY_ENCODER_QUEUE_CREATE);
        // Continue anyway - encoder may not work but device won't crash
    }

    // Cache encoder inversion setting for ISR-safe access
    cached_inverted_encoder = display_config_get()->inverted_encoder;

    // Register interrupt handlers
    irq_handler.register_interrupt(BUTTON0_ENCODER_PIN1, gpio_irq_handler::irq_event::change, _isr_on_encoder_update);
    irq_handler.register_interrupt(BUTTON0_ENCODER_PIN2, gpio_irq_handler::irq_event::change, _isr_on_encoder_update);
    irq_handler.register_interrupt(BUTTON0_ENC_PIN, gpio_irq_handler::irq_event::fall, _isr_on_button_enc_update);
    irq_handler.register_interrupt(BUTTON0_RST_PIN, gpio_irq_handler::irq_event::fall, _isr_on_button_rst_update);

    printf("done\n");
}

ButtonEncoderEvent_t button_wait_for_input(bool block) {
    TickType_t delay_ticks;

    if (block) {
        // Use bounded timeout (4s) instead of portMAX_DELAY so callers
        // like menu_task can feed the hardware watchdog (8.3s timeout)
        delay_ticks = pdMS_TO_TICKS(4000);
    }
    else {
        delay_ticks = 0;
    }

    // Need to check if the queue has been initialized
    if (encoder_event_queue == NULL) {
        return BUTTON_NO_EVENT;
    }

    ButtonEncoderEvent_t button_encoder_event;
    if (!xQueueReceive(encoder_event_queue, &button_encoder_event, delay_ticks)) {
        button_encoder_event = BUTTON_NO_EVENT;
    }

    return button_encoder_event;
}

bool http_rest_button_control(struct fs_file *file, int num_params, char *params[], char *values[]) {
    static char button_control_json_buffer[256];

    if (encoder_event_queue == NULL) {
        snprintf(button_control_json_buffer, sizeof(button_control_json_buffer),
                 "%s{\"error\":\"queue not initialized\"}", http_json_header);
        size_t data_length = strlen(button_control_json_buffer);
        file->data = button_control_json_buffer;
        file->len = data_length;
        file->index = data_length;
        file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;
        return true;
    }

    int len = snprintf(button_control_json_buffer, sizeof(button_control_json_buffer),
                       "%s{\"button_pressed\":[", http_json_header);

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "CW") == 0 && strcmp(values[idx], "true") == 0) {
            ButtonEncoderEvent_t button_event = BUTTON_ENCODER_ROTATE_CW;
            xQueueSend(encoder_event_queue, &button_event, 0);
            len += snprintf(button_control_json_buffer + len, sizeof(button_control_json_buffer) - len, "\"CW\",");
        }

        if (strcmp(params[idx], "CCW") == 0 && strcmp(values[idx], "true") == 0) {
            ButtonEncoderEvent_t button_event = BUTTON_ENCODER_ROTATE_CCW;
            xQueueSend(encoder_event_queue, &button_event, 0);
            len += snprintf(button_control_json_buffer + len, sizeof(button_control_json_buffer) - len, "\"CCW\",");
        }

        if (strcmp(params[idx], "PRESS") == 0 && strcmp(values[idx], "true") == 0) {
            ButtonEncoderEvent_t button_event = BUTTON_ENCODER_PRESSED;
            xQueueSend(encoder_event_queue, &button_event, 0);
            len += snprintf(button_control_json_buffer + len, sizeof(button_control_json_buffer) - len, "\"PRESS\",");
        }

        if (strcmp(params[idx], "RST") == 0 && strcmp(values[idx], "true") == 0) {
            ButtonEncoderEvent_t button_event = BUTTON_RST_PRESSED;
            xQueueSend(encoder_event_queue, &button_event, 0);
            len += snprintf(button_control_json_buffer + len, sizeof(button_control_json_buffer) - len, "\"RST\",");
        }

        if (len >= (int)sizeof(button_control_json_buffer)) {
            len = (int)sizeof(button_control_json_buffer) - 1;
            break;
        }
    }

    // Remove trailing comma
    if (len > 0 && button_control_json_buffer[len-1] == ',') {
        len--;
    }

    len += snprintf(button_control_json_buffer + len, sizeof(button_control_json_buffer) - len, "]}");

    file->data = button_control_json_buffer;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
