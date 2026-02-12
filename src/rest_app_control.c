#include <FreeRTOS.h>
#include <queue.h>
#include <stdio.h>

#include "rest_app_control.h"
#include "error.h"


QueueHandle_t rest_event_queue = NULL;


bool rest_app_control_init() {
    rest_event_queue = xQueueCreate(1, sizeof(rest_control_event_t));

    if (rest_event_queue == NULL) {
        report_error(ERR_REST_QUEUE_CREATE);
        return false;
    }
    return true;
}

