#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "configuration.h"
#include "eeprom.h"
#include "error.h"

// Include only for PICO board with specific flash chip
#include "pico/unique_id.h"


#define PAGE_SIZE   64  // 64 byte page size


bool cat24c256_eeprom_init() {
    // Initialize I2C bus with 400k baud rate
    i2c_init(EEPROM_I2C, 400 * 1000);

    // Initialize PINs as I2C function
    gpio_set_function(EEPROM_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(EEPROM_SCL_PIN, GPIO_FUNC_I2C);

    gpio_pull_up(EEPROM_SDA_PIN);
    gpio_pull_up(EEPROM_SCL_PIN);

    // Probe for EEPROM device on I2C bus - send 0 bytes and check for ACK
    uint8_t dummy;
    absolute_time_t timeout = make_timeout_time_ms(100);
    int ret = i2c_read_blocking_until(EEPROM_I2C, EEPROM_ADDR, &dummy, 1, false, timeout);
    if (ret < 0) {
        printf("EEPROM not found at I2C address 0x%02X\n", EEPROM_ADDR);
        return false;
    }

    return true;
}


bool _cat24c256_write_page(uint16_t data_addr, uint8_t * data, size_t len){
    // Fixed buffer to avoid VLA stack overflow - max is PAGE_SIZE + 2 bytes for address
    if (len > PAGE_SIZE) {
        report_error(ERR_EEPROM_INVALID_SIZE);
        return false;
    }

    uint8_t buf[PAGE_SIZE + 2];  // Fixed size buffer
    buf[0] = (data_addr >> 8) & 0xFF; // High byte of address
    buf[1] = data_addr & 0xFF; // Low byte of address

    // Copy data to buffer
    memcpy(&buf[2], data, len);

    // Send to the EEPROM with 100ms timeout to prevent freeze on I2C bus stuck
    int ret;
    absolute_time_t timeout = make_timeout_time_ms(100);
    ret = i2c_write_blocking_until(EEPROM_I2C, EEPROM_ADDR, buf, len + 2, false, timeout);
    return ret != PICO_ERROR_GENERIC && ret != PICO_ERROR_TIMEOUT;
}


bool cat24c256_write(uint16_t base_addr, uint8_t * data, size_t len) {
    uint16_t num_pages = len / PAGE_SIZE;
    bool is_ok;

    for (uint16_t page = 0; page <= num_pages; page += 1) {
        uint16_t offset = page * PAGE_SIZE;
        size_t write_size = PAGE_SIZE;
        if (page == num_pages) {
            write_size = len % PAGE_SIZE;
        }

        is_ok = _cat24c256_write_page(base_addr + offset, data + offset, write_size);
        // Use FreeRTOS delay to yield CPU instead of busy-wait
        // EEPROM needs 5ms write cycle time
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            busy_wait_us(5 * 1000ULL);
        }
        if (!is_ok) {
            return false;
        }
    }

    return true;
}


bool cat24c256_read(uint16_t data_addr, uint8_t * data, size_t len) {
    uint8_t buf[2];  // Include first two bytes for address
    buf[0] = (data_addr >> 8) & 0xFF; // High byte of address
    buf[1] = data_addr & 0xFF; // Low byte of address

    // Use timeout to prevent freeze on I2C bus stuck
    absolute_time_t timeout = make_timeout_time_ms(100);
    int write_ret = i2c_write_blocking_until(EEPROM_I2C, EEPROM_ADDR, buf, 2, true, timeout);
    if (write_ret == PICO_ERROR_GENERIC || write_ret == PICO_ERROR_TIMEOUT) {
        report_error(ERR_EEPROM_WRITE_FAIL);
        return false;
    }

    int bytes_read;
    timeout = make_timeout_time_ms(100);
    bytes_read = i2c_read_blocking_until(EEPROM_I2C, EEPROM_ADDR, data, len, false, timeout);

    return bytes_read == (int)len;
}


bool cat24c256_eeprom_erase() {
    uint8_t dummy_buffer[PAGE_SIZE];
    memset(dummy_buffer, 0xff, PAGE_SIZE);

    bool all_ok = true;
    for (size_t page=0; page < 512; page++) {
        size_t page_offset = page * PAGE_SIZE;
        if (!cat24c256_write(page_offset, dummy_buffer, PAGE_SIZE)) {
            report_error(ERR_EEPROM_WRITE_FAIL);
            all_ok = false;
        }
    }
    return all_ok;
}
