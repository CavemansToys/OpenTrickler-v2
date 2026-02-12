/*
 * OpenTrickler v2.9.0 RP2040 - Full firmware with WiFi
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/watchdog.h"
#include "lwip/apps/httpd.h"

#include "FreeRTOSConfig.h"
#include "configuration.h"

#include "rest_endpoints.h"
#include "rest_app_control.h"
#include "wireless.h"
#include "error.h"
#include "eeprom.h"
#include "neopixel_led.h"
#include "scale.h"
#include "charge_mode.h"
#include "profile.h"
#include "servo_gate.h"
#include "encoder.h"  // Shared encoder/button for all display types
#include "display_config.h"
#include "motors.h"

// Display-specific includes
#ifdef USE_COLOR_TFT
#include "tft35/tft35_module.h"
#else
#include "mini_12864_module.h"
#include "menu.h"
#endif

extern "C" {
#include "dhcpserver.h"
#include "dnsserver.h"
}

// WiFi AP settings (fallback)
#define WIFI_AP_SSID "OpenTrickler"
#define WIFI_AP_PASSWORD "opentrickler"

// Global for DHCP/DNS servers
static dhcp_server_t dhcp_server;
static dns_server_t dns_server;
static bool connected_to_home = false;
static bool need_reboot = false;

static uint32_t get_cyw43_auth(uint32_t auth_method) {
    switch (auth_method) {
        case 0: return CYW43_AUTH_OPEN;
        case 1: return CYW43_AUTH_WPA_TKIP_PSK;
        case 2: return CYW43_AUTH_WPA2_AES_PSK;
        case 3: return CYW43_AUTH_WPA2_MIXED_PSK;
        default: return CYW43_AUTH_WPA2_AES_PSK;
    }
}

// WiFi task — handles cyw43 init, WiFi connect, and HTTP server
// Must run after the FreeRTOS scheduler starts (required by pico_cyw43_arch_lwip_sys_freertos)
void wifi_init_task(void *params) {
    (void)params;

    if (cyw43_arch_init()) {
        printf("WiFi init FAILED!\n");
        report_error(ERR_WIFI_INIT_FAIL);
        // Stay alive for reboot handling even if WiFi failed
        goto maintenance_loop;
    }
    printf("WiFi init OK\n");

    {
        bool connected = false;
        if (home_wifi_enabled && strlen(home_ssid) > 0) {
            printf("Connecting to home WiFi: %s\n", home_ssid);
            cyw43_arch_enable_sta_mode();

            int err = cyw43_arch_wifi_connect_async(
                home_ssid, home_password,
                get_cyw43_auth(home_auth_method));

            if (err == 0) {
                uint32_t start_ms = to_ms_since_boot(get_absolute_time());
                int status = CYW43_LINK_DOWN;

                while ((to_ms_since_boot(get_absolute_time()) - start_ms) < home_timeout_ms) {
                    status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);

                    if (status == CYW43_LINK_UP) {
                        const char *ip_str = ip4addr_ntoa(netif_ip4_addr(netif_list));
                        printf("Connected! IP: %s\n", ip_str);
                        // Update WiFi status for UI display
                        strncpy(wifi_ssid, home_ssid, sizeof(wifi_ssid) - 1);
                        wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
                        strncpy(wifi_ip_address, ip_str, sizeof(wifi_ip_address) - 1);
                        wifi_ip_address[sizeof(wifi_ip_address) - 1] = '\0';
                        connected = true;
                        break;
                    } else if (status < 0) {
                        printf("Connection failed (status %d)\n", status);
                        break;
                    }

                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                if (!connected) {
                    printf("WiFi timeout/failed (status %d), starting AP\n", status);
                    cyw43_arch_disable_sta_mode();
                }
            } else {
                printf("WiFi async connect failed (err %d), starting AP\n", err);
                cyw43_arch_disable_sta_mode();
            }
        }

        if (!connected) {
            printf("Starting AP: %s\n", WIFI_AP_SSID);
            cyw43_arch_enable_ap_mode(WIFI_AP_SSID, WIFI_AP_PASSWORD, CYW43_AUTH_WPA2_AES_PSK);

            ip4_addr_t gw, mask;
            IP4_ADDR(&gw, 192, 168, 4, 1);
            IP4_ADDR(&mask, 255, 255, 255, 0);
            dhcp_server_init(&dhcp_server, &gw, &mask);
            dns_server_init(&dns_server, &gw);

            // Update WiFi status for UI display (AP mode)
            strncpy(wifi_ssid, WIFI_AP_SSID, sizeof(wifi_ssid) - 1);
            wifi_ssid[sizeof(wifi_ssid) - 1] = '\0';
            strncpy(wifi_ip_address, "192.168.4.1", sizeof(wifi_ip_address) - 1);
            wifi_ip_address[sizeof(wifi_ip_address) - 1] = '\0';
        }

        connected_to_home = connected;
        rest_endpoints_init(connected_to_home);
        httpd_init();
        printf("HTTP server started\n");
        error_set_wifi_ready(true);
    }

maintenance_loop:
    while (1) {
        if (need_reboot) {
            printf("Rebooting...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            watchdog_reboot(0, 0, 0);
            while(1);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main()
{
    error_system_init();

    printf("\n=== OpenTrickler v2.9.0 RP2040 ===\n");

    // Load WiFi credentials from flash (just reads flash memory, no hardware init)
    wireless_config_init();

    // REST app control init (creates the REST event queue)
    if (!rest_app_control_init()) {
        printf("REST app control init FAILED!\n");
    } else {
        printf("REST app control init OK\n");
    }

    // EEPROM init
    if (!eeprom_init()) {
        printf("EEPROM init FAILED!\n");
        report_error(ERR_EEPROM_I2C_INIT);
    } else {
        printf("EEPROM init OK\n");
        error_set_eeprom_ready(true);
    }

    // Neopixel init
    if (!neopixel_led_init()) {
        printf("Neopixel init FAILED!\n");
        report_error(ERR_NEOPIXEL_PIO_INIT);
    } else {
        printf("Neopixel init OK\n");
        error_set_neopixel_ready(true);
    }

    // Display config init (load display type, rotation, brightness from EEPROM)
    if (!display_config_init()) {
        printf("Display config init FAILED!\n");
    } else {
        printf("Display config init OK\n");
    }

    // Display init based on compiled display type
#ifdef USE_COLOR_TFT
    // TFT35/TFT43 display init
    tft35_module_init();
    // Also init rotary encoder (shared encoder module)
    button_init();
    printf("TFT35/TFT43 init OK\n");
    error_set_display_ready(true);
#else
    // Mini 12864 display init (includes button_init and display_init)
    if (!mini_12864_module_init()) {
        printf("Mini 12864 init FAILED!\n");
        report_error(ERR_DISPLAY_MUTEX_CREATE);
    } else {
        printf("Mini 12864 init OK\n");
        error_set_display_ready(true);
    }
#endif

    // Scale init
    if (!scale_init()) {
        printf("Scale init FAILED!\n");
        report_error(ERR_SCALE_UART_INIT);
    } else {
        printf("Scale init OK\n");
    }

    // Profile init
    if (!profile_data_init()) {
        printf("Profile init FAILED!\n");
        report_error(ERR_PROFILE_EEPROM_READ);
    } else {
        printf("Profile init OK\n");
    }

    // Charge mode init
    if (!charge_mode_config_init()) {
        printf("Charge mode init FAILED!\n");
        report_error(ERR_CHARGE_EEPROM_READ);
    } else {
        printf("Charge mode init OK\n");
    }

    // Servo init
    if (!servo_gate_init()) {
        printf("Servo init FAILED!\n");
        report_error(ERR_SERVO_TASK_CREATE);
    } else {
        printf("Servo init OK\n");
    }

    // Motor init (must be before scheduler starts)
    motor_init_err_t motor_init_err = motors_init();
    if (motor_init_err != MOTOR_INIT_OK) {
        handle_motor_init_error(motor_init_err);
    } else {
        printf("Motors init OK\n");
    }

    // Create FreeRTOS tasks
    // WiFi task handles cyw43 init, connect, and HTTPD — pinned to core 0
    TaskHandle_t wifi_task_handle;
    xTaskCreate(wifi_init_task, "WiFi Task", 4096, NULL, 7, &wifi_task_handle);
    vTaskCoreAffinitySet(wifi_task_handle, (1 << 0));  // Pin to core 0

    // Create UI task based on display type
#ifdef USE_COLOR_TFT
    xTaskCreate(tft35_lvgl_task, "LVGL Task", 4096, NULL, 2, NULL);
#else
    xTaskCreate(menu_task, "Menu Task", 4096, NULL, 2, NULL);
#endif

    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();

    while (true);
}
