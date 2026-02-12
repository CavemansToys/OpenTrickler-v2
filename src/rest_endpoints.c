#include <string.h>
#include <stdlib.h>
#include "rest_endpoints.h"
#include "http_rest.h"
#include "charge_mode.h"
#include "motors.h"
#include "scale.h"
#include "wireless.h"
#include "eeprom.h"
#include "encoder.h"  // For http_rest_button_control (shared between display types)
#include "neopixel_led.h"
#include "profile.h"
#include "cleanup_mode.h"
#include "servo_gate.h"
#include "system_control.h"
#include "rest_errors.h"
#include "rest_ai_tuning.h"
#include "ai_tuning.h"
#include "display_config.h"
#include "flash_storage.h"

// Mini 12864 specific includes
#ifndef USE_COLOR_TFT
#include "mini_12864_module.h"
#include "display.h"
#endif

// Generated headers by html2header.py, css2header.py, and bin2header.py under scripts
#include "display_mirror.html.h"
#include "web_portal.html.h"
#include "web_portal_basic.html.h"
#include "styles.css.h"
#include "favicon.ico.h"


bool http_404_error(struct fs_file *file, int num_params, char *params[], char *values[]) {

    static const char not_found_response[] = "HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\n\r\n"
                 "{\"error\":404}";
    file->data = not_found_response;
    file->len = sizeof(not_found_response) - 1;
    file->index = sizeof(not_found_response) - 1;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}


bool http_display_mirror(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_display_mirror_html);

    file->data = html_display_mirror_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_web_portal(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_web_portal_html);

    file->data = html_web_portal_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_web_portal_basic(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(html_web_portal_basic_html);

    file->data = html_web_portal_basic_html;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_styles_css(struct fs_file *file, int num_params, char *params[], char *values[]) {
    size_t len = strlen(css_styles_css);

    file->data = css_styles_css;
    file->len = len;
    file->index = len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool http_favicon(struct fs_file *file, int num_params, char *params[], char *values[]) {
    file->data = (const char *)favicon_ico;
    file->len = favicon_ico_len;
    file->index = favicon_ico_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT;

    return true;
}


bool rest_endpoints_init(bool connected_to_home) {
    // Both AP mode and Home WiFi serve the full portal at root
    rest_register_handler("/", http_web_portal);

    rest_register_handler("/portal", http_web_portal);
    rest_register_handler("/basic", http_web_portal_basic);
    rest_register_handler("/styles.css", http_styles_css);
    rest_register_handler("/favicon.ico", http_favicon);
    rest_register_handler("/404", http_404_error);
    rest_register_handler("/rest/scale_action", http_rest_scale_action);
    rest_register_handler("/rest/scale_config", http_rest_scale_config);
    rest_register_handler("/rest/charge_mode_config", http_rest_charge_mode_config);
    rest_register_handler("/rest/charge_mode_state", http_rest_charge_mode_state);
    rest_register_handler("/rest/cleanup_mode_state", http_rest_cleanup_mode_state);
    rest_register_handler("/rest/system_control", http_rest_system_control);
    rest_register_handler("/rest/coarse_motor_config", http_rest_coarse_motor_config);
    rest_register_handler("/rest/fine_motor_config", http_rest_fine_motor_config);

    // Encoder/button control - works for all display types
    rest_register_handler("/rest/button_control", http_rest_button_control);

    // Mini 12864 specific endpoints
#ifndef USE_COLOR_TFT
    rest_register_handler("/rest/mini_12864_config", http_rest_mini_12864_module_config);
    rest_register_handler("/display_buffer", http_get_display_buffer);
    rest_register_handler("/display_mirror", http_display_mirror);
#endif

    rest_register_handler("/rest/wireless_config", http_rest_wireless_config);
    rest_register_handler("/rest/pico_led", http_rest_pico_led);
    rest_register_handler("/rest/neopixel_led_config", http_rest_neopixel_led_config);
    rest_register_handler("/rest/profile_config", http_rest_profile_config);
    rest_register_handler("/rest/profile_summary", http_rest_profile_summary);
    rest_register_handler("/rest/servo_gate_state", http_rest_servo_gate_state);
    rest_register_handler("/rest/servo_gate_config", http_rest_servo_gate_config);
    rest_register_handler("/rest/errors", http_rest_errors);
    rest_register_handler("/rest/clear_errors", http_rest_clear_errors);
    rest_register_handler("/rest/display_config", http_rest_display_config);

    // Initialize flash storage for ML history (faster than I2C EEPROM)
    flash_storage_init();

    // Initialize AI tuning system and REST endpoints
    ai_tuning_init();
    rest_ai_tuning_init();

    return true;
}
