#include "generic_scale.h"
#include "scale.h"

#include "FreeRTOS.h"
#include "task.h"
#include "configuration.h"



void _generic_scale_listener_task(void *p);
void _generic_scale_init(void *self);


scale_handle_t generic_scale_drv_handle = {
    .read_loop_task = _generic_scale_listener_task,
    .force_zero = NULL,
};
extern scale_config_t scale_config;


/**
 * @brief Generic scale listener task
 */
void _generic_scale_listener_task(void *p) {
    char rx_buffer[32];
    uint8_t rx_buffer_idx = 0;

    while (true) {
        // Read all available data
        while (uart_is_readable(SCALE_UART)) {
            char ch = uart_getc(SCALE_UART);
            rx_buffer[rx_buffer_idx++] = ch;

            // Stop condition 1: When \n is received
            if (ch == '\n') {
                // Null terminate the string
                rx_buffer[rx_buffer_idx] = '\0';

                // Reset buffer index
                rx_buffer_idx = 0;
            }
            // Stop condition 2: Buffer full
            else if (rx_buffer_idx >= sizeof(rx_buffer) - 1) {
                // Null terminate the string
                rx_buffer[rx_buffer_idx] = '\0';

                // Reset buffer index
                rx_buffer_idx = 0;
            }
            else {
                // Continue receiving
                continue;
            }

            // Parse the received string to float
            char *endptr;
            float weight = strtof(rx_buffer, &endptr);

            // If the conversion is successful then post the measurement.
            if (endptr != rx_buffer) {
                scale_config.current_scale_measurement = weight;
                if (scale_config.scale_measurement_ready) {
                    xSemaphoreGive(scale_config.scale_measurement_ready);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}