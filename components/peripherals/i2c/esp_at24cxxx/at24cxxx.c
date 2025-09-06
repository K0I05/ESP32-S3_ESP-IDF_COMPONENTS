/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file at24cxxx.c
 *
 * ESP-IDF driver for AT24CXXX EEPROM types
 * 
 * https://github.com/nopnop2002/esp-idf-24c/tree/master
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

/**
 * dependency includes
 */

#include "include/at24cxxx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
 
/**
 * constant definitions
 */

#define AT24CXXX_POWERUP_DELAY_MS      UINT16_C(120)
#define AT24CXXX_APPSTART_DELAY_MS     UINT16_C(10)    /*!< at24cxxx delay after initialization before application start-up */
#define AT24CXXX_READ_DELAY_MS         UINT16_C(5) 
#define AT24CXXX_WRITE_DELAY_MS        UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/**
 * macro definitions
 */

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief AT24CXXX device descriptor structure definition.
 */
typedef struct at24cxxx_device_s {
    at24cxxx_config_t           config;        /*!< at24cxxx device configuration */
    i2c_master_dev_handle_t     i2c_handle;    /*!< at24cxxx i2c device handle */
    at24cxxx_memory_mapping_t   memory_map;    /*!< at24cxxx memory map structure */
    uint8_t                    *buffer;        /*!< at24cxxx data buffer */
} at24cxxx_device_t;

 
/**
 * static constant declarations
 */

static const char* TAG = "at24cxxx";

/**
 * @brief AT24CXXX memory layout by EEPROM type.  See datasheets by EEPROM family type for details.
 */
static const at24cxxx_memory_mapping_t at24cxxx_memory_maps[5] = {
    { .eeprom_type=AT24CXXX_AT24C32,  .memory_size_kbits=32,  .memory_size_bits=32768,  .memory_size_bytes=4096,  
        .page_size_bytes=32,  .number_of_pages=128, .max_data_address=(128 * 32) - 1,  .write_time_ms=10+5 },  // AT24C32
    { .eeprom_type=AT24CXXX_AT24C64,  .memory_size_kbits=64,  .memory_size_bits=64536,  .memory_size_bytes=8192,  
        .page_size_bytes=32,  .number_of_pages=256, .max_data_address=(128 * 64) - 1,  .write_time_ms=10+5 },  // AT24C64
    { .eeprom_type=AT24CXXX_AT24C128, .memory_size_kbits=128, .memory_size_bits=131072, .memory_size_bytes=16384, 
        .page_size_bytes=64,  .number_of_pages=256, .max_data_address=(128 * 128) - 1, .write_time_ms=5+5 },   // AT24C128
    { .eeprom_type=AT24CXXX_AT24C256, .memory_size_kbits=256, .memory_size_bits=262144, .memory_size_bytes=32768, 
        .page_size_bytes=64,  .number_of_pages=512, .max_data_address=(128 * 256) - 1, .write_time_ms=5+5 },   // AT24C256
    { .eeprom_type=AT24CXXX_AT24C512, .memory_size_kbits=512, .memory_size_bits=524288, .memory_size_bytes=65536, 
        .page_size_bytes=128, .number_of_pages=512, .max_data_address=(128 * 512) - 1, .write_time_ms=5+5 },   // AT24C512
};

/**
 * static function and subroutine declarations
 */

/**
 * function and subroutine declarations
 */

 static inline esp_err_t at24cxxx_i2c_read(at24cxxx_device_t *const device, uint8_t *data, const uint16_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, data, size, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_read failed" );

    return ESP_OK;
}

static inline esp_err_t at24cxxx_i2c_read_from(at24cxxx_device_t *const device, const uint16_t reg_addr, uint8_t *data, uint16_t *const size) {
    const bit16_uint8_buffer_t tx = { (uint8_t)((reg_addr >> 8) & 0xff), (uint8_t)(reg_addr & 0xff) }; // msb, lsb (register)

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, data, device->memory_map.page_size_bytes, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_read_from failed" );

    /* set output parameters */
    *size = device->memory_map.page_size_bytes;

    return ESP_OK;
}

/**
 * @brief AT24CXXX I2C HAL read byte from word register address transaction.
 * 
 * @param device AT24CXXX device descriptor.
 * @param reg_addr AT24CXXX word register address to read from.
 * @param byte AT24CXXX read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t at24cxxx_i2c_read_byte_from(at24cxxx_device_t *const device, const uint16_t reg_addr, uint8_t *const byte) {
    const bit16_uint8_buffer_t tx = { (uint8_t)((reg_addr >> 8) & 0xff), (uint8_t)(reg_addr & 0xff) }; // msb, lsb (register)
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

static inline esp_err_t at24cxxx_i2c_write_to(at24cxxx_device_t *const device, const uint16_t reg_addr, const uint8_t *data, const uint16_t size) {
    uint16_t idx = 0;

    /* validate arguments */
    ESP_ARG_CHECK( device && data );

    /* reset data buffer */
    memset(device->buffer, 0, device->memory_map.page_size_bytes + 2);

    /* append register address to data buffer */
    device->buffer[idx++] = (uint8_t)((reg_addr >> 8) & 0xff);  // msb
    device->buffer[idx++] = (uint8_t)(reg_addr & 0xff);         // lsb

    /* append data to data buffer */
    for(uint16_t i = 0; i < size; i++) {
        device->buffer[idx++] = data[i];
    }

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, device->buffer, size + 2, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_write_to, i2c write failed" );
                 
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(device->memory_map.write_time_ms));

    return ESP_OK;
}

/**
 * @brief AT24CXXX I2C HAL write byte to word register address transaction.
 * 
 * @param device AT24CXXX device descriptor.
 * @param reg_addr AT24CXXX register address to write to as a word.
 * @param byte AT24CXXX write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t at24cxxx_i2c_write_byte_to(at24cxxx_device_t *const device, const uint16_t reg_addr, const uint8_t byte) {
    const bit24_uint8_buffer_t tx = { (uint8_t)((reg_addr >> 8) & 0xff), (uint8_t)(reg_addr & 0xff), byte }; // msb, lsb (register), data

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "at24cxxx_i2c_write_word_to, i2c write failed" );
                        
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(device->memory_map.write_time_ms));

    return ESP_OK;
}

esp_err_t at24cxxx_get_memory_map(at24cxxx_handle_t handle, at24cxxx_memory_mapping_t *const memory_map) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && memory_map );

    /* copy memory map */
    *memory_map = dev->memory_map;

    return ESP_OK;
}

esp_err_t at24cxxx_init(const i2c_master_bus_handle_t master_handle, const at24cxxx_config_t *at24cxxx_config, at24cxxx_handle_t *const at24cxxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && (at24cxxx_config || at24cxxx_handle) );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AT24CXXX_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, at24cxxx_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, at24cxxx device handle initialization failed", at24cxxx_config->i2c_address);

    /* validate memory availability for handle */
    at24cxxx_device_t* dev = (at24cxxx_device_t*)calloc(1, sizeof(at24cxxx_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c at24cxxx device, init failed");

    /* copy configuration */
    dev->config     = *at24cxxx_config;
    dev->memory_map = at24cxxx_memory_maps[dev->config.eeprom_type];

    /* validate memory availability for device buffer */
    dev->buffer = (uint8_t*)calloc(1, dev->memory_map.page_size_bytes + 2);
    ESP_GOTO_ON_FALSE(dev->buffer, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c at24cxxx device buffer, init failed");

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* set device handle */
    *at24cxxx_handle = (at24cxxx_handle_t)dev;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AT24CXXX_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t at24cxxx_read_current_address(at24cxxx_handle_t handle, uint8_t *const data) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt read i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_read(dev, data, 1), TAG, "i2c read current address failed" );

    return ESP_OK;
}

esp_err_t at24cxxx_read_random_byte(at24cxxx_handle_t handle, const uint16_t data_addr, uint8_t *const data) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    ESP_RETURN_ON_FALSE((data_addr <= dev->memory_map.max_data_address), ESP_ERR_INVALID_ARG, TAG, "data address 0x%04x is out of range", data_addr);

    /* attempt read i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_read_byte_from(dev, data_addr, data), TAG, "i2c read from word address 0x%04x failed", data_addr );

    return ESP_OK;
}

esp_err_t at24cxxx_read_sequential_bytes(at24cxxx_handle_t handle, uint8_t *data, const uint16_t size) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt read i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_read(dev, data, size), TAG, "i2c sequential read failed" );

    return ESP_OK;
}

esp_err_t at24cxxx_read_page(at24cxxx_handle_t handle, const uint16_t data_addr, uint8_t *data, uint16_t *const size) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt read i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_read_from(dev, data_addr, data, size), TAG, "i2c page read failed" );

    return ESP_OK;
}

esp_err_t at24cxxx_write_byte(at24cxxx_handle_t handle, const uint16_t data_addr, const uint8_t data) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    ESP_RETURN_ON_FALSE((data_addr <= dev->memory_map.max_data_address), ESP_ERR_INVALID_ARG, TAG, "data address 0x%04x is out of range", data_addr);

    /* attempt write i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_write_byte_to(dev, data_addr, data), TAG, "i2c write to word address 0x%04x failed", data_addr );

    return ESP_OK;
}

esp_err_t at24cxxx_write_page(at24cxxx_handle_t handle, const uint16_t data_addr, const uint8_t *data, const uint16_t size) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    ESP_RETURN_ON_FALSE((data_addr <= dev->memory_map.max_data_address), ESP_ERR_INVALID_ARG, TAG, "data address 0x%04x is out of range", data_addr);
    ESP_RETURN_ON_FALSE((data_addr+size <= dev->memory_map.max_data_address), ESP_ERR_INVALID_ARG, TAG, "data address 0x%04x is out of range for page size", data_addr);
    ESP_RETURN_ON_FALSE((size <= dev->memory_map.page_size_bytes), ESP_ERR_INVALID_ARG, TAG, "page size is out of range");

    /* attempt write i2c transaction */
    ESP_RETURN_ON_ERROR( at24cxxx_i2c_write_to(dev, data_addr, data, size), TAG, "i2c write to page address 0x%04x failed", data_addr );

    return ESP_OK;
}

esp_err_t at24cxxx_erase(at24cxxx_handle_t handle) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* iterate through each page */
    for(uint16_t i = 0; i < dev->memory_map.number_of_pages; i++) {
        uint8_t buffer[dev->memory_map.page_size_bytes];

        /* set data address */
        uint16_t data_addr = (i * dev->memory_map.page_size_bytes);

        /* copy 0xff to page bytes */
        memset(buffer, 0xff, dev->memory_map.page_size_bytes);

        /* attempt to write page */
        ESP_RETURN_ON_ERROR(at24cxxx_i2c_write_to(dev, data_addr, buffer, dev->memory_map.page_size_bytes), TAG, "unable to write page, erase failed");
    }

    return ESP_OK;
}

esp_err_t at24cxxx_remove(at24cxxx_handle_t handle) {
    at24cxxx_device_t* dev = (at24cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t at24cxxx_delete(at24cxxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( at24cxxx_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* at24cxxx_get_fw_version(void) {
    return (const char*)AT24CXXX_FW_VERSION_STR;
}

int32_t at24cxxx_get_fw_version_number(void) {
    return (int32_t)AT24CXXX_FW_VERSION_INT32;
}