/**
 * @file encoder.h
 * @brief Rotary encoder and button input handling
 *
 * This module handles the rotary encoder and buttons shared between
 * Mini 12864 and TFT35/TFT43 displays.
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <FreeRTOS.h>
#include <queue.h>
#include "http_rest.h"

typedef enum {
    BUTTON_NO_EVENT = 0,
    BUTTON_ENCODER_ROTATE_CW,
    BUTTON_ENCODER_ROTATE_CCW,
    BUTTON_ENCODER_PRESSED,
    BUTTON_RST_PRESSED,

    // Overrides from other sources, used to signal other thread to proceed
    OVERRIDE_FROM_REST,
} ButtonEncoderEvent_t;

#ifdef __cplusplus
extern "C" {
#endif

// Encoder event queue (shared between display types)
extern QueueHandle_t encoder_event_queue;

/**
 * Initialize button and encoder hardware
 */
void button_init(void);

/**
 * Wait for button encoder input
 */
ButtonEncoderEvent_t button_wait_for_input(bool block);

/**
 * REST endpoint for button control (inject button events via web)
 */
bool http_rest_button_control(struct fs_file *file, int num_params, char *params[], char *values[]);

#ifdef __cplusplus
}
#endif

#endif  // ENCODER_H_
