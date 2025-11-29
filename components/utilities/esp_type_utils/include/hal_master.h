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
 * @file hal_master.h
 * @defgroup Hardware Abstraction Layer Master (hal_master)
 * @{
 *
 * 
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __HAL_MASTER_H__
#define __HAL_MASTER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_XFR_TIMEOUT_MS           (1000)

/**
 * @brief HAL Master Bus Interface Types
 */
typedef enum hal_master_interfaces_e {
    HAL_MASTER_BIF_I2C = 0,   /*!< I2C master bus interface */
    HAL_MASTER_BIF_SPI = 1    /*!< SPI master bus interface */
} hal_master_interfaces_t;

/**
 * @brief HAL Master Device Structure
 */
typedef struct hal_master_dev_s hal_master_dev_t;

/**
 * @brief HAL Master Device Handle
 */
typedef hal_master_dev_t* hal_master_dev_handle_t;

/**
 * @brief HAL Master Interface V-Table
 */
typedef struct {
    /**
     * @brief HAL master interface to probe the device to check if it is present.
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to probe devices on HAL master bus.
     * @return esp_err_t ESP_OK if device is present.
     */
    esp_err_t (*probe)(hal_master_dev_handle_t device, const uint8_t reg_addr);
    /**
     * @brief HAL master interface to read data from the device.
     * 
     * @param device HAL master device handle.
     * @param buffer Buffer to store read data.
     * @param size Number of bytes to read.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*read)(hal_master_dev_handle_t device, uint8_t *buffer, const uint8_t size);
    /**
     * @brief HAL master interface to read data from a specific register address (byte).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to read from.
     * @param buffer Buffer to store read data.
     * @param size Number of bytes to read.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*read_from)(hal_master_dev_handle_t device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size);
    /**
     * @brief HAL master interface to read data from a specific register address (word).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address (word) to read from.
     * @param buffer Buffer to store read data.
     * @param size Number of bytes to read.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*read_from_word)(hal_master_dev_handle_t device, const uint16_t reg_addr, uint8_t *buffer, const uint8_t size);
    /**
     * @brief HAL master interface to read a byte from a specific register address (byte).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to read from.
     * @param byte Pointer to store the read byte.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*read_byte_from)(hal_master_dev_handle_t device, const uint8_t reg_addr, uint8_t *const byte);
    /**
     * @brief HAL master interface to read a word from a specific register address (byte).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to read from.
     * @param word Pointer to store the read word.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*read_word_from)(hal_master_dev_handle_t device, const uint8_t reg_addr, uint16_t *const word);
    /**
     * @brief HAL master interface to write data to the device.
     * 
     * @param dev HAL master device handle.
     * @param buffer Buffer containing data to write.
     * @param size Number of bytes to write.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*write)(hal_master_dev_handle_t device, const uint8_t *buffer, const uint8_t size);
    /**
     * @brief HAL master interface to write data to a specific register address (byte).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to write to.
     * @param buffer Buffer containing data to write.
     * @param size Number of bytes to write.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*write_to)(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t size);
    /**
     * @brief HAL master interface to write data to a specific register address (word).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address (word) to write to.
     * @param buffer Buffer containing data to write.
     * @param size Number of bytes to write.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*write_to_word)(hal_master_dev_handle_t device, const uint16_t reg_addr, const uint8_t *buffer, const uint8_t size);
    /**
     * @brief HAL master interface to write a byte to a specific register address (byte).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to write to.
     * @param byte Byte to write.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*write_byte_to)(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint8_t byte);
    /**
     * @brief HAL master interface to write a word to a specific register address (byte).
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address to write to.
     * @param word Word to write.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*write_word_to)(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint16_t word);
    /**
     * @brief HAL master interface to write a command (byte) to the device.
     * 
     * @param device HAL master device handle.
     * @param reg_addr Register address or command to write.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*write_command)(hal_master_dev_handle_t device, const uint8_t reg_addr);
    /**
     * @brief HAL master interface to remove the device from the bus.
     * 
     * @param device HAL master device handle.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*remove)(hal_master_dev_handle_t device);
    /**
     * @brief HAL master interface to delete the HAL master device handle.
     * 
     * @param device HAL master device handle.
     * @return esp_err_t ESP_OK on success.
     */
    esp_err_t (*delete)(hal_master_dev_handle_t device);
} hal_master_interface_t;

/**
 * @brief HAL Master Device Structure
 */
struct hal_master_dev_s {
    hal_master_interfaces_t       mb_bif;  /*!< HAL master bus interface type */
    const hal_master_interface_t *mb_api;  /*!< Pointer to the HAL master bus interface V-Table */
    void                        *dev_ctx;  /*!< Opaque context (device driver handle) */
    void                         *mb_ctx;  /*!< Opaque context (master bus driver handle) */
};

/**
 * @brief Factory function to create a new I2C HAL device.
 * 
 * @param bus_handle I2C master bus handle.
 * @param dev_config I2C device configuration.
 * @param ret_handle Pointer to store the created HAL device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hal_master_new_i2c_device(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, hal_master_dev_handle_t *ret_handle);

/**
 * @brief Factory function to create a new SPI HAL device.
 * 
 * @param host_device SPI host device.
 * @param dev_config SPI device configuration.
 * @param ret_handle Pointer to store the created HAL device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hal_master_new_spi_device(spi_host_device_t host_device, const spi_device_interface_config_t *dev_config, hal_master_dev_handle_t *ret_handle);


/* Static inline helpers for cleaner syntax */

/**
 * @brief HAL master inline implementation to probe the device to check if it is present.
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to probe devices on HAL master bus.
 * @return esp_err_t ESP_OK if device is present.
 */
static inline esp_err_t hal_master_probe(hal_master_dev_handle_t device, const uint8_t reg_addr) {
    if (!device || !device->mb_api || !device->mb_api->probe) return ESP_ERR_INVALID_ARG;
    return device->mb_api->probe(device, reg_addr);
}

/**
 * @brief HAL master inline implementation to read data from the device.
 * 
 * @param device HAL master device handle.
 * @param buffer Buffer to store read data.
 * @param size Number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_read(hal_master_dev_handle_t device, uint8_t *buffer, const uint8_t size) {
    if (!device || !device->mb_api || !device->mb_api->read) return ESP_ERR_INVALID_ARG;
    return device->mb_api->read(device, buffer, size);
}

/**
 * @brief HAL master inline implementation to read data from a specific register address (byte).
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to read from.
 * @param buffer Buffer to store read data.
 * @param size Number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_read_from(hal_master_dev_handle_t device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    if (!device || !device->mb_api || !device->mb_api->read_from) return ESP_ERR_INVALID_ARG;
    return device->mb_api->read_from(device, reg_addr, buffer, size);
}

/**
 * @brief HAL master inline implementation to read a byte from a specific register address (byte).
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to read from.
 * @param byte Pointer to store the read byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_read_byte_from(hal_master_dev_handle_t device, const uint8_t reg_addr, uint8_t *const byte) {
    if (!device || !device->mb_api || !device->mb_api->read_byte_from) return ESP_ERR_INVALID_ARG;
    return device->mb_api->read_byte_from(device, reg_addr, byte);
}

/**
 * @brief HAL master inline implementation to read a word from a specific register address (byte).
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to read from.
 * @param word Pointer to store the read word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_read_word_from(hal_master_dev_handle_t device, const uint8_t reg_addr, uint16_t *const word) {
    if (!device || !device->mb_api || !device->mb_api->read_word_from) return ESP_ERR_INVALID_ARG;
    return device->mb_api->read_word_from(device, reg_addr, word);
}

/**
 * @brief HAL master inline implementation to write data to the device.
 * 
 * @param device HAL master device handle.
 * @param buffer Buffer containing data to write.
 * @param size Number of bytes to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write(hal_master_dev_handle_t device, const uint8_t *buffer, const uint8_t size) {
    if (!device || !device->mb_api || !device->mb_api->write) return ESP_ERR_INVALID_ARG;
    return device->mb_api->write(device, buffer, size);
}

/**
 * @brief HAL master inline implementation to write data to a specific register address (byte).
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to write to.
 * @param buffer Buffer containing data to write.
 * @param size Number of bytes to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_to(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t size) {
    if (!device || !device->mb_api || !device->mb_api->write_to) return ESP_ERR_INVALID_ARG;
    return device->mb_api->write_to(device, reg_addr, buffer, size);
}

/**
 * @brief HAL master inline implementation to write a byte to a specific register address (byte).
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to write to.
 * @param byte Byte to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_byte_to(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint8_t byte) {
    if (!device || !device->mb_api || !device->mb_api->write_byte_to) return ESP_ERR_INVALID_ARG;
    return device->mb_api->write_byte_to(device, reg_addr, byte);
}

/**
 * @brief HAL master inline implementation to write a word to a specific register address (byte).
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address to write to.
 * @param word Word to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_word_to(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint16_t word) {
    if (!device || !device->mb_api || !device->mb_api->write_word_to) return ESP_ERR_INVALID_ARG;
    return device->mb_api->write_word_to(device, reg_addr, word);
}

/**
 * @brief HAL master inline implementation to write a command (byte) to the device.
 * 
 * @param device HAL master device handle.
 * @param reg_addr Register address or command to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_command(hal_master_dev_handle_t device, const uint8_t reg_addr) {
    if (!device || !device->mb_api || !device->mb_api->write_command) return ESP_ERR_INVALID_ARG;
    return device->mb_api->write_command(device, reg_addr);
}

/**
 * @brief HAL master inline implementation to remove the device from the bus.
 * 
 * @param device HAL master device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_remove(hal_master_dev_handle_t device) {
    if (!device || !device->mb_api || !device->mb_api->remove) return ESP_ERR_INVALID_ARG;
    return device->mb_api->remove(device);
}

/**
 * @brief HAL master inline implementation to delete the HAL master device handle.
 * 
 * @param device HAL master device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_delete(hal_master_dev_handle_t device) {
    if (!device || !device->mb_api || !device->mb_api->delete) return ESP_ERR_INVALID_ARG;
    return device->mb_api->delete(device);
}

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __HAL_MASTER_H__