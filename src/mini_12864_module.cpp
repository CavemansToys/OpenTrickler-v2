//
//            -----
//        5V |10  9 | GND
//        -- | 8  7 | --
//    (DIN)  | 6  5#| (RESET)
//  (LCD_A0) | 4  3 | (LCD_CS)
// (BTN_ENC) | 2  1 | --
//            ------
//             EXP1
//            -----
//        5V |10  9 | GND
//   (RESET) | 8  7 | --
//   (MOSI)  | 6  5#| (EN2)
//        -- | 4  3 | (EN1)
//  (LCD_SCK)| 2  1 | --
//            ------
//             EXP2
//
// For Pico W
// EXP1_2 (BTN_ENC) <-> PIN29 (GP22)
// EXP2_5 (ENCODER2) <-> PIN19 (GP14)
// EXP2_3 (ENCODER1) <-> PIN20 (GP15)
// EXP2_8 (BTN_RST) <-> PIN16 (GP12)

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>

#include "pico/stdlib.h"
#include "u8g2.h"
#include "mui.h"
#include "mui_u8g2.h"

#include "configuration.h"
#include "mini_12864_module.h"
#include "encoder.h"
#include "http_rest.h"
#include "eeprom.h"
#include "common.h"
#include "display.h"
#include "display_config.h"


// Configs
mini_12864_module_config_t mini_12864_module_config;

// Local variables
extern u8g2_t display_handler;

// External variables
extern muif_t muif_list[];
extern fds_t fds_data[];
extern const size_t muif_cnt;


bool mini_12864_module_config_save() {
    bool is_ok = eeprom_write(EEPROM_MINI_12864_CONFIG_BASE_ADDR, (uint8_t *) &mini_12864_module_config, sizeof(mini_12864_module_config));
    return is_ok;
}


bool mini_12864_module_init() {
    // Display settings are now managed by display_config (rotation, encoder inversion, etc.)
    // display_config_init() should be called before this function

    // Run subsequent function inits
    button_init();  // From encoder.cpp
    display_init();

    return true;
}


uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg)
    {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:	// called once during init phase of u8g2/u8x8
            // We don't initialize here
            break;							// can be used to setup pins
        case U8X8_MSG_DELAY_NANO:			// delay arg_int * 1 nano second
            break;
        case U8X8_MSG_DELAY_100NANO:		// delay arg_int * 100 nano seconds
            __asm volatile ("NOP\n");
            break;
        case U8X8_MSG_DELAY_10MICRO:		// delay arg_int * 10 micro seconds
            busy_wait_us(arg_int * 10ULL);
            break;
        case U8X8_MSG_DELAY_MILLI:			// delay arg_int * 1 milli second
        {
            BaseType_t scheduler_state = xTaskGetSchedulerState();
            delay_ms(arg_int, scheduler_state);
            break;
        }
        case U8X8_MSG_DELAY_I2C:				// arg_int is the I2C speed in 100KHz, e.g. 4 = 400 KHz
            break;							// arg_int=1: delay by 5us, arg_int = 4: delay by 1.25us
        case U8X8_MSG_GPIO_D0:				// D0 or SPI clock pin: Output level in arg_int
            //case U8X8_MSG_GPIO_SPI_CLOCK:
            break;
        case U8X8_MSG_GPIO_D1:				// D1 or SPI data pin: Output level in arg_int
            //case U8X8_MSG_GPIO_SPI_DATA:
            break;
        case U8X8_MSG_GPIO_D2:				// D2 pin: Output level in arg_int
        break;
        case U8X8_MSG_GPIO_D3:				// D3 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D4:				// D4 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D5:				// D5 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D6:				// D6 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_D7:				// D7 pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_E:				// E/WR pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_CS:				// CS (chip select) pin: Output level in arg_int
            gpio_put(DISPLAY0_CS_PIN, arg_int);
            break;
        case U8X8_MSG_GPIO_DC:				// DC (data/cmd, A0, register select) pin: Output level in arg_int
            gpio_put(DISPLAY0_A0_PIN, arg_int);
            break;
        case U8X8_MSG_GPIO_RESET:			// Reset pin: Output level in arg_int
            gpio_put(DISPLAY0_RESET_PIN, arg_int);
            break;
        case U8X8_MSG_GPIO_CS1:				// CS1 (chip select) pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_CS2:				// CS2 (chip select) pin: Output level in arg_int
            break;
        case U8X8_MSG_GPIO_I2C_CLOCK:		// arg_int=0: Output low at I2C clock pin
            break;							// arg_int=1: Input dir with pullup high for I2C clock pin
        case U8X8_MSG_GPIO_I2C_DATA:			// arg_int=0: Output low at I2C data pin
            break;							// arg_int=1: Input dir with pullup high for I2C data pin
        case U8X8_MSG_GPIO_MENU_SELECT:
            u8x8_SetGPIOResult(u8x8, /* get menu select pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_NEXT:
            u8x8_SetGPIOResult(u8x8, /* get menu next pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_PREV:
            u8x8_SetGPIOResult(u8x8, /* get menu prev pin state */ 0);
            break;
        case U8X8_MSG_GPIO_MENU_HOME:
            u8x8_SetGPIOResult(u8x8, /* get menu home pin state */ 0);
            break;
        default:
            u8x8_SetGPIOResult(u8x8, 1);			// default return value
        break;
    }
    return 1;
}

uint8_t u8x8_byte_pico_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
        case U8X8_MSG_BYTE_SEND:
            // taskENTER_CRITICAL();
            spi_write_blocking(DISPLAY0_SPI, (uint8_t *) arg_ptr, arg_int);
            // taskEXIT_CRITICAL();
            break;
        case U8X8_MSG_BYTE_INIT:
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
            break;
        case U8X8_MSG_BYTE_SET_DC:
            u8x8_gpio_SetDC(u8x8, arg_int);
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
            u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->post_chip_enable_wait_ns, NULL);
            break;
        case U8X8_MSG_BYTE_END_TRANSFER:
            u8x8->gpio_and_delay_cb(u8x8, U8X8_MSG_DELAY_NANO, u8x8->display_info->pre_chip_disable_wait_ns, NULL);
            u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
            break;
        default:
            return 0;
    }
    return 1;
}



void display_init() {
    // Configure 12864
    printf("Initializing Display Task -- ");
    // Initialize SPI engine
    spi_init(DISPLAY0_SPI, 4000 * 1000);

    // Configure port for SPI
    gpio_set_function(DISPLAY0_SCK_PIN, GPIO_FUNC_SPI);  // SCK
    gpio_set_function(DISPLAY0_TX_PIN, GPIO_FUNC_SPI);  // Tx (MOSI)

    // Configure property for CS
    gpio_init(DISPLAY0_CS_PIN);
    gpio_set_dir(DISPLAY0_CS_PIN, GPIO_OUT);
    gpio_put(DISPLAY0_CS_PIN, 1);

    // Configure property for A0 (D/C)
    gpio_init(DISPLAY0_A0_PIN);
    gpio_set_dir(DISPLAY0_A0_PIN, GPIO_OUT);
    gpio_put(DISPLAY0_A0_PIN, 0);

    // Configure property for RESET
    gpio_init(DISPLAY0_RESET_PIN);
    gpio_set_dir(DISPLAY0_RESET_PIN, GPIO_OUT);
    gpio_put(DISPLAY0_RESET_PIN, 0);

    // Initialize driver
    u8g2_Setup_uc1701_mini12864_f(
        &display_handler,
        U8G2_R0,
        u8x8_byte_pico_hw_spi,
        u8x8_gpio_and_delay
    );

    // Initialize Screen
    u8g2_InitDisplay(&display_handler);
    u8g2_SetPowerSave(&display_handler, 0);
    u8g2_SetContrast(&display_handler, 255);

    // Initialize screen rotation from display_config
    const u8g2_cb_t *u8g2_cb;
    switch (display_config_get()->rotation)
    {
        case DISPLAY_ROTATION_90:
            u8g2_cb = U8G2_R1;
            break;
        case DISPLAY_ROTATION_180:
            u8g2_cb = U8G2_R2;
            break;
        case DISPLAY_ROTATION_270:
            u8g2_cb = U8G2_R3;
            break;
        case DISPLAY_ROTATION_0:
        default:  // default is no rotation
            u8g2_cb = U8G2_R0;
            break;
    }
    u8g2_SetDisplayRotation(&display_handler, u8g2_cb);

    // Clear
    u8g2_ClearBuffer(&display_handler);
    u8g2_ClearDisplay(&display_handler);

    printf("done\n");
}



bool http_rest_mini_12864_module_config(struct fs_file *file, int num_params, char *params[], char *values[]) {
    // Mappings (legacy - now delegated to display_config):
    // b0 (bool): inverted_encoder_direction -> display_config.inverted_encoder
    // b1 (int): display_rotation -> display_config.rotation
    // ee (bool): save to eeprom
    static char buf[128];
    bool save_to_eeprom = false;
    display_config_t *config = display_config_get();

    // Control
    for (int idx = 0; idx < num_params; idx += 1) {
        if (strcmp(params[idx], "b0") == 0) {
            config->inverted_encoder = string_to_boolean(values[idx]);
        }
        if (strcmp(params[idx], "b1") == 0) {
            int rot = atoi(values[idx]);
            if (rot >= DISPLAY_ROTATION_0 && rot <= DISPLAY_ROTATION_270) {
                config->rotation = (display_rotation_t) rot;
            }
        }
        else if (strcmp(params[idx], "ee") == 0) {
            save_to_eeprom = string_to_boolean(values[idx]);
        }
    }

    // Perform action
    if (save_to_eeprom) {
        display_config_save();
    }

    // Response
    snprintf(buf, sizeof(buf),
             "%s"
             "{\"b0\":%s, \"b1\":%d}",
             http_json_header,
             boolean_to_string(config->inverted_encoder),
             config->rotation);

    size_t response_len = strlen(buf);
    file->data = buf;
    file->len = response_len;
    file->index = response_len;
    file->flags = FS_FILE_FLAGS_HEADER_INCLUDED;

    return true;
}
