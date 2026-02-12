#ifndef CAT24C256_EEPROM_H_
#define CAT24C256_EEPROM_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// EEPROM Address Map (CAT24C256 = 32KB total)
// Each section gets 1KB for future expansion
#define EEPROM_METADATA_BASE_ADDR               (0 * 1024)     // 0K  - Board ID, revision
#define EEPROM_SCALE_CONFIG_BASE_ADDR           (1 * 1024)     // 1K  - Scale settings
// 2K-3K: Reserved for future use
#define EEPROM_MOTOR_CONFIG_BASE_ADDR           (4 * 1024)     // 4K  - Motor settings
#define EEPROM_CHARGE_MODE_BASE_ADDR            (5 * 1024)     // 5K  - Charge mode settings
#define EEPROM_APP_CONFIG_BASE_ADDR             (6 * 1024)     // 6K  - App settings
#define EEPROM_NEOPIXEL_LED_CONFIG_BASE_ADDR    (7 * 1024)     // 7K  - Neopixel LED settings
#define EEPROM_MINI_12864_CONFIG_BASE_ADDR      (8 * 1024)     // 8K  - Mini 12864 display
#define EEPROM_PROFILE_DATA_BASE_ADDR           (9 * 1024)     // 9K  - Powder profiles
#define EEPROM_SERVO_GATE_CONFIG_BASE_ADDR      (10 * 1024)    // 10K - Servo gate settings
#define EEPROM_TFT35_CONFIG_BASE_ADDR           (11 * 1024)    // 11K - TFT35 display settings
#define EEPROM_AI_TUNING_HISTORY_BASE_ADDR      (12 * 1024)    // 12K - AI tuning ML history
#define EEPROM_DISPLAY_CONFIG_BASE_ADDR         (13 * 1024)    // 13K - Display type selection
#define EEPROM_AI_TUNING_CONFIG_BASE_ADDR       (14 * 1024)    // 14K - AI tuning config (ranges, thresholds)
// 15K-31K: Reserved for future use

#define EEPROM_METADATA_REV                     2              // 16 byte 


typedef struct {
    uint16_t eeprom_metadata_rev;
    char unique_id[8];
} __attribute__((packed)) eeprom_metadata_t;

// EEPROM save handler function
typedef bool (*eeprom_save_handler_t)(void);


#ifdef __cplusplus
extern "C" {
#endif

bool eeprom_init(void);
bool eeprom_config_save();
bool eeprom_read(uint16_t data_addr, uint8_t * data, size_t len);
bool eeprom_write(uint16_t data_addr, uint8_t * data, size_t len);

bool eeprom_get_board_id(char ** board_id_buffer, size_t bytes_to_copy);

/*
 * Fill all EEPROM with 0xFF
 */
uint8_t eeprom_erase(bool);
uint8_t eeprom_save_all(void);
void eeprom_register_handler(eeprom_save_handler_t handler);


#ifdef __cplusplus
}
#endif

#endif  // CAT24C256_EEPROM_H_