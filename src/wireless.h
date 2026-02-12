#ifndef WIRELESS_H_
#define WIRELESS_H_

#include <stdint.h>
#include <stdbool.h>
#include "http_rest.h"

// Auth types (same as CYW43)
typedef enum {
    AUTH_OPEN = 0,
    AUTH_WPA_TKIP_PSK = 1,
    AUTH_WPA2_AES_PSK = 2,
    AUTH_WPA2_MIXED_PSK = 3,
} cyw43_auth_t;

// Global WiFi config - exposed for app.cpp
extern char home_ssid[33];
extern char home_password[64];
extern uint32_t home_auth_method;
extern uint32_t home_timeout_ms;
extern bool home_wifi_enabled;

// Current WiFi status - for UI display
extern char wifi_ssid[33];        // Currently connected SSID
extern char wifi_ip_address[16];  // Current IP address

#ifdef __cplusplus
extern "C" {
#endif

// Config management (uses flash, not EEPROM)
bool wireless_config_init(void);
bool wireless_config_save(void);

// Menu stub (WiFi info now shown via web portal at 192.168.4.1)
uint8_t wireless_view_wifi_info(void);

// REST API handlers
bool http_rest_wireless_config(struct fs_file *file, int num_params, char *params[], char *values[]);
bool http_rest_pico_led(struct fs_file *file, int num_params, char *params[], char *values[]);
void pico_led_set(bool on);

#ifdef __cplusplus
}
#endif

#endif  // WIRELESS_H_
