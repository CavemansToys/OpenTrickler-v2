#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico/flash.h"

#include "wireless.h"
#include "http_rest.h"
#include "common.h"

// Fixed flash offset for WiFi config - works for both pico_w (2MB) and pico2_w (4MB)
// Located at 960KB, after firmware (~750KB) but before OTA staging (1MB+)
#define WIFI_FLASH_OFFSET  0x0F0000  // 960KB - fixed location regardless of flash size
#define CONFIG_MAGIC 0x57494649  // "WIFI" in hex

// WiFi credentials structure for flash storage
typedef struct {
    uint32_t magic;
    char home_ssid[33];
    char home_password[64];
    uint32_t auth_method;
    uint32_t timeout_ms;
    bool wifi_enabled;
    uint32_t checksum;
} wifi_credentials_t;

// Global WiFi config - exposed for REST API
char home_ssid[33];
char home_password[64];
uint32_t home_auth_method = 2;
uint32_t home_timeout_ms = 30000;
bool home_wifi_enabled = true;

// Current WiFi status - for UI display
char wifi_ssid[33] = "Not Connected";
char wifi_ip_address[16] = "0.0.0.0";

// Calculate simple checksum
static uint32_t calculate_checksum(const wifi_credentials_t *creds) {
    uint32_t sum = creds->magic;
    for (size_t i = 0; i < sizeof(creds->home_ssid); i++) sum += creds->home_ssid[i];
    for (size_t i = 0; i < sizeof(creds->home_password); i++) sum += creds->home_password[i];
    sum += creds->auth_method + creds->timeout_ms + (creds->wifi_enabled ? 1 : 0);
    return sum;
}

// Read WiFi credentials from flash
static bool read_wifi_credentials(wifi_credentials_t *creds) {
    const uint8_t *flash_ptr = (const uint8_t *)(XIP_BASE + WIFI_FLASH_OFFSET);
    memcpy(creds, flash_ptr, sizeof(wifi_credentials_t));
    if (creds->magic != CONFIG_MAGIC) {
        printf("No valid WiFi config at 0x%X\n", WIFI_FLASH_OFFSET);
        return false;
    }
    if (creds->checksum != calculate_checksum(creds)) {
        printf("WiFi config checksum mismatch\n");
        return false;
    }
    printf("Valid WiFi config found at 0x%X\n", WIFI_FLASH_OFFSET);
    return true;
}

// Static buffer for flash write (must persist during flash_safe_execute callback)
static uint8_t g_wifi_write_buffer[FLASH_PAGE_SIZE] __attribute__((aligned(4)));

// Callback for flash_safe_execute - performs the actual erase + program
static void wifi_flash_write_callback(void *param) {
    (void)param;
    flash_range_erase(WIFI_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(WIFI_FLASH_OFFSET, g_wifi_write_buffer, FLASH_PAGE_SIZE);
}

// Write WiFi credentials to flash
static bool write_wifi_credentials(void) {
    wifi_credentials_t creds = {0};
    snprintf(creds.home_ssid, sizeof(creds.home_ssid), "%s", home_ssid);
    snprintf(creds.home_password, sizeof(creds.home_password), "%s", home_password);
    creds.auth_method = home_auth_method;
    creds.timeout_ms = home_timeout_ms;
    creds.wifi_enabled = home_wifi_enabled;
    creds.magic = CONFIG_MAGIC;
    creds.checksum = calculate_checksum(&creds);

    printf("Writing WiFi credentials to flash at 0x%X...\n", WIFI_FLASH_OFFSET);

    // Prepare write buffer
    memset(g_wifi_write_buffer, 0, FLASH_PAGE_SIZE);
    memcpy(g_wifi_write_buffer, &creds, sizeof(wifi_credentials_t));

    // Use flash_safe_execute for FreeRTOS-safe flash programming
    int rc = flash_safe_execute(wifi_flash_write_callback, NULL, 1000);
    if (rc != PICO_OK) {
        printf("ERROR: flash_safe_execute failed with %d\n", rc);
        return false;
    }

    // Verify write
    wifi_credentials_t verify = {0};
    if (!read_wifi_credentials(&verify)) {
        printf("ERROR: Flash write verification failed\n");
        return false;
    }
    printf("WiFi credentials written and verified\n");
    return true;
}

// Load WiFi config from flash on startup
bool wireless_config_init(void) {
    wifi_credentials_t saved_creds = {0};
    bool has_saved = read_wifi_credentials(&saved_creds);

    if (has_saved) {
        snprintf(home_ssid, sizeof(home_ssid), "%s", saved_creds.home_ssid);
        snprintf(home_password, sizeof(home_password), "%s", saved_creds.home_password);
        home_auth_method = saved_creds.auth_method;
        home_timeout_ms = saved_creds.timeout_ms;
        home_wifi_enabled = saved_creds.wifi_enabled;
        printf("Loaded WiFi config: SSID=%s\n", home_ssid);
    }
    return has_saved;
}

// Save WiFi config to flash
bool wireless_config_save(void) {
    return write_wifi_credentials();
}

// REST API handler for wireless configuration
bool http_rest_wireless_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mapping
    // w0 (str): ssid
    // w1 (str): pw
    // w2 (int): auth
    // w3 (int): timeout_ms
    // w4 (bool): enable
    // save (bool): save to flash

    static char wireless_config_json_buffer[256];
    bool save_config = false;

    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "w0") == 0) {
            strncpy(home_ssid, values[idx], sizeof(home_ssid) - 1);
            home_ssid[sizeof(home_ssid) - 1] = '\0';
        }
        else if (strcmp(params[idx], "w1") == 0) {
            strncpy(home_password, values[idx], sizeof(home_password) - 1);
            home_password[sizeof(home_password) - 1] = '\0';
        }
        else if (strcmp(params[idx], "w2") == 0) {
            int val = atoi(values[idx]);
            if (val < 0) val = 0;
            if (val > 3) val = 3;
            home_auth_method = (uint32_t) val;
        }
        else if (strcmp(params[idx], "w3") == 0) {
            int val = atoi(values[idx]);
            if (val < 1000) val = 1000;
            if (val > 120000) val = 120000;
            home_timeout_ms = (uint32_t) val;
        }
        else if (strcmp(params[idx], "w4") == 0) {
            home_wifi_enabled = string_to_boolean(values[idx]);
        }
        else if (strcmp(params[idx], "save") == 0) {
            save_config = string_to_boolean(values[idx]);
        }
    }

    if (save_config) {
        wireless_config_save();
    }

    // Sanitize SSID for JSON: replace any quotes/backslashes with underscores
    char safe_ssid[33];
    strncpy(safe_ssid, home_ssid, sizeof(safe_ssid) - 1);
    safe_ssid[sizeof(safe_ssid) - 1] = '\0';
    for (char *p = safe_ssid; *p; p++) {
        if (*p == '"' || *p == '\\' || *p < 0x20) {
            *p = '_';
        }
    }

    snprintf(wireless_config_json_buffer,
             sizeof(wireless_config_json_buffer),
             "%s"
             "{\"w0\":\"%.32s\",\"w2\":%"PRIu32",\"w3\":%"PRIu32",\"w4\":%s}",
             http_json_header,
             safe_ssid,
             home_auth_method,
             home_timeout_ms,
             boolean_to_string(home_wifi_enabled));

    size_t data_length = strlen(wireless_config_json_buffer);
    file->data = wireless_config_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}

// Simple LED control - use cyw43_gpio_set() directly (not cyw43_arch_gpio_put which can hang)
#define LED_GPIO 0

void pico_led_set(bool on) {
    cyw43_gpio_set(&cyw43_state, LED_GPIO, on);
}

// Stub for menu compatibility - WiFi info now shown via web portal
uint8_t wireless_view_wifi_info(void) {
    return 40;  // Returns to the Wireless menu
}

bool http_rest_pico_led(struct fs_file *file, int num_params, char *params[], char *values[]) {
    static char pico_led_json_buffer[128];

    for (int idx = 0; idx < num_params; idx++) {
        if (strcmp(params[idx], "state") == 0) {
            bool on = (strcmp(values[idx], "1") == 0 || strcmp(values[idx], "on") == 0 || strcmp(values[idx], "true") == 0);
            cyw43_gpio_set(&cyw43_state, LED_GPIO, on);
        }
    }

    bool led_state = false;
    cyw43_gpio_get(&cyw43_state, LED_GPIO, &led_state);

    snprintf(pico_led_json_buffer, sizeof(pico_led_json_buffer),
             "%s{\"state\":%s}",
             http_json_header,
             boolean_to_string(led_state));

    size_t data_length = strlen(pico_led_json_buffer);
    file->data = pico_led_json_buffer;
    file->len = data_length;
    file->index = data_length;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
