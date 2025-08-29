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
 * @file at24cxxx.h
 * @defgroup drivers at24cxxx
 * @{
 *
 * ESP-IDF driver for at24cxxx EEPROM types
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AT24CXXX_H__
#define __AT24CXXX_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "at24cxxx_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_AT24CXXX_DEV_CLK_SPD   UINT32_C(100000) /*!< at24cxxx i2c device scl clock frequency (100KHz) */
#define I2C_AT24CXXX_DEV_ADDR      UINT8_C(0x50)    /*!< at24cxxx i2c device address */

/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `at24cxxx_config_t` to default configuration settings for the 32-Kbits EEPROM family type.
 */
#define I2C_AT24C32_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AT24CXXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AT24CXXX_DEV_CLK_SPD,       \
    .eeprom_type     = AT24CXXX_AT24C32 }

/**
 * @brief Macro that initializes `at24cxxx_config_t` to default configuration settings for the 64-Kbits EEPROM family type.
 */
#define I2C_AT24C64_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AT24CXXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AT24CXXX_DEV_CLK_SPD,       \
    .eeprom_type     = AT24CXXX_AT24C64 }

/**
 * @brief Macro that initializes `at24cxxx_config_t` to default configuration settings for the 128-Kbits EEPROM family type.
 */
#define I2C_AT24C128_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AT24CXXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AT24CXXX_DEV_CLK_SPD,       \
    .eeprom_type     = AT24CXXX_AT24C128 }

/**
 * @brief Macro that initializes `at24cxxx_config_t` to default configuration settings for the 256-Kbits EEPROM family type.
 */
#define I2C_AT24C256_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AT24CXXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AT24CXXX_DEV_CLK_SPD,       \
    .eeprom_type     = AT24CXXX_AT24C256 }

/**
 * @brief Macro that initializes `at24cxxx_config_t` to default configuration settings for the 512-Kbits EEPROM family type.
 */
#define I2C_AT24C512_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_AT24CXXX_DEV_ADDR,          \
    .i2c_clock_speed = I2C_AT24CXXX_DEV_CLK_SPD,       \
    .eeprom_type     = AT24CXXX_AT24C512 }


/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief AT24CXXX EEPROM types enumerator definition.
 */
typedef enum at24cxxx_types_e {
    AT24CXXX_AT24C32 = 0,   /*!< at24cxxx 32-Kbits EEPROM */
    AT24CXXX_AT24C64,       /*!< at24cxxx 64-Kbits EEPROM */
    AT24CXXX_AT24C128,      /*!< at24cxxx 128-Kbits EEPROM */
    AT24CXXX_AT24C256,      /*!< at24cxxx 256-Kbits EEPROM */
    AT24CXXX_AT24C512       /*!< at24cxxx 512-Kbits EEPROM */
} at24cxxx_types_t;

/**
 * @brief AT24CXXX memory mapping structure definition. 
 */
typedef struct at24cxxx_memory_mapping_s {
    at24cxxx_types_t eeprom_type;   /*!< at24cxxx EEPROM type, see `at24cxxx_eeprom_types_t` enumerator for supported EEPROM types */
    uint32_t memory_size_kbits;     /*!< at24cxxx memory size in Kbits */
    uint32_t memory_size_bits;      /*!< at24cxxx memory size in bits */
    uint32_t memory_size_bytes;     /*!< at24cxxx memory size in bytes */
    uint16_t page_size_bytes;       /*!< at24cxxx page size in bytes */
    uint16_t number_of_pages;       /*!< at24cxxx number of pages */
    uint16_t max_data_address;      /*!< at24cxxx maximum data word address */
    uint8_t write_time_ms;          /*!< at24cxxx write time in ms */
} at24cxxx_memory_mapping_t;


/**
 * @brief AT24CXXX configuration structure definition.
 */
typedef struct at24cxxx_config_s {
    uint16_t            i2c_address;       /*!< at24cxxx i2c device address */
    uint32_t            i2c_clock_speed;   /*!< at24cxxx i2c device scl clock speed in hz */
    at24cxxx_types_t    eeprom_type;       /*!< at24cxxx EEPROM type, see `at24cxxx_eeprom_types_t` enumerator for supported EEPROM types */
} at24cxxx_config_t;

/**
 * @brief AT24CXXX context structure.
 */
struct at24cxxx_context_t {
    at24cxxx_config_t           dev_config;    /*!< at24cxxx device configuration */
    i2c_master_dev_handle_t     i2c_handle;    /*!< at24cxxx i2c device handle */
    at24cxxx_memory_mapping_t   memory_map;    /*!< at24cxxx memory map structure */
    uint8_t                    *buffer;        /*!< at24cxxx data buffer */
};

/**
 * @brief AT24CXXX context structure definition.
 */
typedef struct at24cxxx_context_t at24cxxx_context_t;


/**
 * @brief AT24CXXX handle structure definition.
 */
typedef struct at24cxxx_context_t* at24cxxx_handle_t;

/**
 * public function and subroutine declarations
 */


/**
 * @brief Initializes an AT24CXXX device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] at24cxxx_config AT24CXXX device configuration.
 * @param[out] at24cxxx_handle AT24CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_init(const i2c_master_bus_handle_t master_handle, const at24cxxx_config_t *at24cxxx_config, at24cxxx_handle_t *const at24cxxx_handle);

/**
 * @brief Reads data from AT24CXXX EEPROM at current address.  See datasheet for details.
 * 
 * @param[in] handle AT24CXXX device handle.
 * @param[out] data AT24CXXX data read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_read_current_address(at24cxxx_handle_t handle, uint8_t *const data);

/**
 * @brief Reads random data from AT24CXXX EEPROM.  See datasheet for details.
 * 
 * @param[in] handle AT24CXXX device handle.
 * @param[in] data_addr AT24CXXX data address to read from.
 * @param[out] data AT24CXXX data read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_read_random_byte(at24cxxx_handle_t handle, const uint16_t data_addr, uint8_t *const data);

/**
 * @brief Reads data sequentially from AT24CXXX EEPROM at current address.  See datasheet for details. 
 * 
 * @param handle AT24CXXX device handle.
 * @param data AT24CXXX data to read.
 * @param size AT24CXXX size of data to read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_read_sequential_bytes(at24cxxx_handle_t handle, uint8_t *data, const uint16_t size);

/**
 * @brief Reads a page of data from AT24CXXX EEPROM.
 * 
 * @param handle AT24CXXX device handle.
 * @param data_addr AT24CXXX data address to read from.
 * @param data AT24CXXX data to read.
 * @param size AT24CXXX size of data read.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_read_page(at24cxxx_handle_t handle, const uint16_t data_addr, uint8_t *data, uint16_t *const size);

/**
 * @brief Writes data to AT24CXXX EEPROM.  See datasheet for details.
 * 
 * @param[in] handle AT24CXXX device handle.
 * @param[in] data_addr AT24CXXX data address to write to.
 * @param[in] data AT24CXXX data to write.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_write_byte(at24cxxx_handle_t handle, const uint16_t data_addr, const uint8_t data);

/**
 * @brief Writes a page of data to AT24CXXX EEPROM.
 * 
 * @param handle AT24CXXX device handle.
 * @param data_addr AT24CXXX data address to write to.
 * @param data AT24CXXX data to write.
 * @param size AT24CXXX size of data to write.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_write_page(at24cxxx_handle_t handle, const uint16_t data_addr, const uint8_t *data, const uint16_t size);


/**
 * @brief Erases data onboard the AT24CXXX EEPROM.  See datasheet for details.
 *
 * @param handle AT24CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_erase(at24cxxx_handle_t handle);

/**
 * @brief Removes an AT24CXXX device from master bus.
 *
 * @param[in] handle AT24CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_remove(at24cxxx_handle_t handle);

/**
 * @brief Removes an AT24CXXX device from master bus and frees handle.
 * 
 * @param handle AT24CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t at24cxxx_delete(at24cxxx_handle_t handle);

/**
 * @brief Converts AT24CXXX firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* AT24CXXX firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* at24cxxx_get_fw_version(void);

/**
 * @brief Converts AT24CXXX firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t AT24CXXX firmware version number.
 */
int32_t at24cxxx_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __AT24CXXX_H__
