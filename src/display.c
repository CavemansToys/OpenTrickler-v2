#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <u8g2.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "display.h"
#include "http_rest.h"
#include "error.h"


// Local variables
u8g2_t display_handler;
SemaphoreHandle_t display_buffer_access_mutex = NULL;

u8g2_t * get_display_handler(void) {
    return &display_handler;
}

bool display_mutex_init(void) {
    if (display_buffer_access_mutex == NULL) {
        display_buffer_access_mutex = xSemaphoreCreateMutex();
        if (display_buffer_access_mutex == NULL) {
            return false;
        }
    }
    return true;
}

void acquire_display_buffer_access() {
    if (display_buffer_access_mutex == NULL) {
        // Fallback lazy init with critical section for safety
        taskENTER_CRITICAL();
        if (display_buffer_access_mutex == NULL) {
            display_buffer_access_mutex = xSemaphoreCreateMutex();
        }
        taskEXIT_CRITICAL();
        if (display_buffer_access_mutex == NULL) {
            report_error(ERR_DISPLAY_MUTEX_CREATE);
            return;
        }
    }
    // Use timeout to prevent freeze
    if (xSemaphoreTake(display_buffer_access_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return;
    }
}

void release_display_buffer_access() {
    if (display_buffer_access_mutex != NULL) {
        xSemaphoreGive(display_buffer_access_mutex);
    }
}


/* u8g2 buffer structure can be decoded according to the description here: 
    https://github.com/olikraus/u8g2/wiki/u8g2reference#memory-structure-for-controller-with-u8x8-support

    Here is the Python script helping to explain how u8g2 buffer are arranged.

        with open(f, 'rb') as fp:
            display_buffer = fp.read()

        tile_width = 0x10  # 16 tiles per tile row

        for tile_row_idx in range(8):
            for bit in range(8):
                # Each tile row includes 16 * 8 bytes
                for byte_idx in range(tile_width * 8):
                    data_offset = byte_idx + tile_row_idx * tile_width * 8
                    data = display_buffer[data_offset]
                    if (1 << bit) & data:
                        print(' * ', end='')
                    else:
                        print('   ', end='')

                print()

*/
// WARNING: This function returns a raw pointer to the live display buffer without
// holding the display mutex. The HTTP server may read the buffer while the UI task
// is actively updating it, resulting in a torn read (partial old + partial new frame).
// A full snapshot under mutex is impractical due to buffer size and HTTP callback
// constraints. Consumers should treat the data as best-effort / advisory only.
bool http_get_display_buffer(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t buffer_size = 8 * u8g2_GetBufferTileHeight(&display_handler) * u8g2_GetBufferTileWidth(&display_handler);
    file->data = (const char *) u8g2_GetBufferPtr(&display_handler);
    file->len = buffer_size;
    file->index = buffer_size;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}