/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
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
 * @file hal_master.c
 *
 * ESP-IDF Hardware Abstraction Layer (HAL) master communication bus subroutines for I2C and SPI devices
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "hal_master.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_check.h>

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
 * I2C Implementation
 */

static esp_err_t i2c_probe(hal_master_dev_handle_t device, const uint8_t reg_addr) {
    ESP_ARG_CHECK(device);
    i2c_master_bus_handle_t i2c_bus = (i2c_master_bus_handle_t)device->mb_ctx;
    return i2c_master_probe(i2c_bus, reg_addr, I2C_MASTER_XFR_TIMEOUT_MS);
}

static esp_err_t i2c_read(hal_master_dev_handle_t device, uint8_t *buffer, const uint8_t size) {
    ESP_ARG_CHECK(device);
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)device->dev_ctx;
    return i2c_master_receive(i2c_dev, buffer, size, I2C_MASTER_XFR_TIMEOUT_MS);
}

static esp_err_t i2c_read_from(hal_master_dev_handle_t device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    ESP_ARG_CHECK(device);
    const uint8_t tx_buffer[1] = { reg_addr };
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)device->dev_ctx;
    return i2c_master_transmit_receive(i2c_dev, tx_buffer, 1, buffer, size, I2C_MASTER_XFR_TIMEOUT_MS);
}

static esp_err_t i2c_read_byte_from(hal_master_dev_handle_t device, const uint8_t reg_addr, uint8_t *const byte) {
    ESP_ARG_CHECK(device);
    uint8_t rx_buffer[1] = {0};
    esp_err_t ret = i2c_read_from(device, reg_addr, rx_buffer, 1);
    if (ret == ESP_OK && byte) {
        *byte = rx_buffer[0];
    }
    return ret;
}

static esp_err_t i2c_read_word_from(hal_master_dev_handle_t device, const uint8_t reg_addr, uint16_t *const word) {
    ESP_ARG_CHECK(device);
    uint8_t rx_buffer[2] = {0};
    esp_err_t ret = i2c_read_from(device, reg_addr, rx_buffer, 2);
    if (ret == ESP_OK && word) {
        // combine the two bytes into a word (little-endian)
        *word = (uint16_t)rx_buffer[0] | ((uint16_t)rx_buffer[1] << 8);
    }
    return ret;
}

static esp_err_t i2c_write(hal_master_dev_handle_t device, const uint8_t *buffer, const uint8_t size) {
    ESP_ARG_CHECK(device);
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)device->dev_ctx;
    return i2c_master_transmit(i2c_dev, buffer, size, I2C_MASTER_XFR_TIMEOUT_MS);
}

static esp_err_t i2c_write_to(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint8_t *buffer, const uint8_t size) {
    ESP_ARG_CHECK(device);
    uint8_t *tx_buffer = (uint8_t *)malloc(size + 1);
    if (!tx_buffer) {
        return ESP_ERR_NO_MEM;
    }
    tx_buffer[0] = reg_addr;
    memcpy(&tx_buffer[1], buffer, size);
    esp_err_t ret = i2c_write(device, tx_buffer, size + 1);
    free(tx_buffer);
    return ret;
}

static esp_err_t i2c_write_byte_to(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint8_t byte) {
    ESP_ARG_CHECK(device);
    const uint8_t buffer[1] = { byte };
    return i2c_write_to(device, reg_addr, buffer, 1);
}

static esp_err_t i2c_write_word_to(hal_master_dev_handle_t device, const uint8_t reg_addr, const uint16_t word) {
    ESP_ARG_CHECK(device);
    // extract a word into two bytes (little-endian)
    const uint8_t buffer[2] = { (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) };
    return i2c_write_to(device, reg_addr, buffer, 2);
}

static esp_err_t i2c_write_command(hal_master_dev_handle_t device, const uint8_t reg_addr) {
    ESP_ARG_CHECK(device);
    const uint8_t buffer[1] = { reg_addr };
    return i2c_write(device, buffer, 1);
}

static esp_err_t i2c_remove(hal_master_dev_handle_t device) {
    ESP_ARG_CHECK(device);
    i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)device->dev_ctx;
    return i2c_master_bus_rm_device(i2c_dev);
}

static esp_err_t i2c_delete(hal_master_dev_handle_t device) {
    ESP_ARG_CHECK(device);
    esp_err_t ret = i2c_remove(device);
    if (ret == ESP_OK) {
        free(device);
    }
    return ret;
}

/*
 * HAL I2C API interface structure
 */
static const hal_master_interface_t i2c_interface = {
    .probe          = i2c_probe,
    .read           = i2c_read,
    .read_from      = i2c_read_from,
    .read_byte_from = i2c_read_byte_from,
    .read_word_from = i2c_read_word_from,
    .write          = i2c_write,
    .write_to       = i2c_write_to,
    .write_byte_to  = i2c_write_byte_to,
    .write_word_to  = i2c_write_word_to,
    .write_command  = i2c_write_command,
    .remove         = i2c_remove,
    .delete         = i2c_delete,
};

esp_err_t hal_master_new_i2c_device(i2c_master_bus_handle_t bus_handle, const i2c_device_config_t *dev_config, hal_master_dev_handle_t *ret_handle) {
    ESP_ARG_CHECK(bus_handle && dev_config && ret_handle);

    hal_master_dev_handle_t dev = (hal_master_dev_handle_t)calloc(1, sizeof(hal_master_dev_t));
    if (!dev) return ESP_ERR_NO_MEM;

    i2c_master_dev_handle_t i2c_dev_handle;
    esp_err_t ret = i2c_master_bus_add_device(bus_handle, dev_config, &i2c_dev_handle);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    dev->mb_bif  = HAL_MASTER_BIF_I2C;
    dev->mb_api  = &i2c_interface;
    dev->dev_ctx = i2c_dev_handle;
    dev->mb_ctx  = bus_handle;
    *ret_handle  = dev;

    return ESP_OK;
}

/*
 * SPI Implementation
 */

static esp_err_t spi_probe(hal_master_dev_handle_t device, const uint8_t reg_addr) {
    return ESP_ERR_NOT_SUPPORTED; // SPI doesn't really have a standard "probe"
}

static esp_err_t spi_read(hal_master_dev_handle_t device, uint8_t *buffer, const uint8_t size) {
    ESP_ARG_CHECK(device);
    spi_device_handle_t spi_dev = (spi_device_handle_t)device->dev_ctx;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.rx_buffer = buffer;
    return spi_device_transmit(spi_dev, &t);
}

static esp_err_t spi_write(hal_master_dev_handle_t device, const uint8_t *buffer, const uint8_t size) {
    ESP_ARG_CHECK(device);
    spi_device_handle_t spi_dev = (spi_device_handle_t)device->dev_ctx;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.tx_buffer = buffer;
    return spi_device_transmit(spi_dev, &t);
}

static esp_err_t spi_write_command(hal_master_dev_handle_t device, const uint8_t reg_addr) {
    ESP_ARG_CHECK(device);
    spi_device_handle_t spi_dev = (spi_device_handle_t)device->dev_ctx;
    spi_transaction_t t;
    uint8_t buffer[1] = { reg_addr };
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = buffer;
    return spi_device_transmit(spi_dev, &t);
}

static esp_err_t spi_remove(hal_master_dev_handle_t device) {
    ESP_ARG_CHECK(device);
    spi_device_handle_t spi_dev = (spi_device_handle_t)device->dev_ctx;
    return spi_bus_remove_device(spi_dev);
}

static esp_err_t spi_delete(hal_master_dev_handle_t device) {
    ESP_ARG_CHECK(device);
    esp_err_t ret = spi_remove(device);
    if (ret == ESP_OK) {
        free(device);
    }
    return ret;
}

/*
 * HAL SPI API interface structure
 */
static const hal_master_interface_t spi_interface = {
    .probe         = spi_probe,
    .read          = spi_read,
    .write         = spi_write,
    .write_command = spi_write_command,
    .remove        = spi_remove,
    .delete        = spi_delete,
};

esp_err_t hal_master_new_spi_device(spi_host_device_t host_device, const spi_device_interface_config_t *dev_config, hal_master_dev_handle_t *ret_handle) {
    ESP_ARG_CHECK(dev_config && ret_handle);

    hal_master_dev_handle_t dev = (hal_master_dev_handle_t)calloc(1, sizeof(hal_master_dev_t));
    if (!dev) return ESP_ERR_NO_MEM;

    spi_device_handle_t spi_dev_handle;
    esp_err_t ret = spi_bus_add_device(host_device, dev_config, &spi_dev_handle);
    if (ret != ESP_OK) {
        free(dev);
        return ret;
    }

    dev->mb_bif  = HAL_MASTER_BIF_SPI;
    dev->mb_api  = &spi_interface;
    dev->dev_ctx = spi_dev_handle;
    dev->mb_ctx  = &host_device;
    *ret_handle  = dev;

    return ESP_OK;
}