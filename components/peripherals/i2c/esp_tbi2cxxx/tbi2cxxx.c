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
 * @file tbi2cxxx.c
 *
 * ESP-IDF driver for TBI2CXXX ir temperature sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/tbi2cxxx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * TBI2CXXX definitions
 */

#define TBI2CXXX_CMD_AMB_TEMP_R        UINT8_C(0x06)
#define TBI2CXXX_CMD_OBJ_TEMP_R        UINT8_C(0x07)
#define TBI2CXXX_CMD_EMIS_COEF_RW      UINT8_C(0x24)
#define TBI2CXXX_CMD_SLV_ADDR_RW       UINT8_C(0x2e)

#define TBI2CXXX_DATA_READY_DELAY_MS   UINT16_C(105)
#define TBI2CXXX_POWERUP_DELAY_MS      UINT16_C(205)
#define TBI2CXXX_APPSTART_DELAY_MS     UINT16_C(10)    /*!< tbi2cxxx delay after initialization before application start-up */
#define TBI2CXXX_CMD_DELAY_MS          UINT16_C(5)     /*!< tbi2cxxx delay before attempting I2C transactions after a command is issued */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief TBI2CXXX device descriptor structure definition.
 */
typedef struct tbi2cxxx_device_s {
    tbi2cxxx_config_t       config;
    i2c_master_dev_handle_t i2c_handle; /*!< I2C device handle */
} tbi2cxxx_device_t;

/*
* static constant declarations
*/
static const char *TAG = "tbi2cxxx";

static const uint8_t crc8_table[256]= { // CRC table
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3  
};



/**
 * @brief Calculates PEC.
 * 
 * @param crc Data buffer to calculate PEC against.
 * @param size Size of data buffer.
 * @return uint8_t Calculated PEC.
 */
static inline uint8_t tbi2cxxx_calculate_pec(uint8_t *crc, uint8_t size) {
    uint8_t data, count;
    uint16_t remainder = 0;
    for(count=0; count<size; ++count) {
        data = *(crc++) ^ remainder;
        remainder = crc8_table[data] ^ (remainder >> 8);
    }
    return remainder;
}

/**
 * @brief Decodes raw `uint16_t` temperature to floating point temperature in degrees celsius.
 * 
 * @param encoded_temperature Raw `uint16_t` temperature to decode.
 * @return float Decoded floating point temperature in degrees celsius.
 */
static inline float tbi2cxxx_decode_temperature(uint16_t encoded_temperature) {
    float decoded_temperature = (float)encoded_temperature * 0.02f;
    decoded_temperature -= 273.15f;
    return decoded_temperature;
}

/**
 * @brief HAL reads a word (2-bytes) from TBI2CXXX.
 * 
 * @param device TBI2CXXX device descriptor.
 * @param reg_addr TBI2CXXX device register address (1-byte).
 * @param data `uint16_t` (2-byte) word read from TBI2CXXX.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tbi2cxxx_i2c_read_word_from(tbi2cxxx_device_t *const device, const uint8_t reg_addr, uint16_t *const data) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit24_uint8_buffer_t      rx = { };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write and read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to transmit and receive, read word failed" );

    /* set buffer data for pec validation */
    const bit40_uint8_buffer_t pec_buf = { 
        device->config.i2c_address << 1,          /*<! i2c device write address */
        reg_addr,                                       /*!< i2c eeprom address */
        (device->config.i2c_address << 1) | 0x01, /*<! i2c device read address */
        rx[0],                                          /*!< low-byte */
        rx[1]                                           /*!< high-bye */
    };

    /* calculate and validate pec from rx data */
    ESP_RETURN_ON_FALSE((rx[2] == tbi2cxxx_calculate_pec(pec_buf, BIT40_UINT8_BUFFER_SIZE)), ESP_ERR_INVALID_CRC, TAG, "invalid pec received, read word failed" );

    /* set output parameter */
    *data = (rx[1] << 8) | rx[0]; // high-byte | low-byte

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TBI2CXXX_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief HAL writes a word (2-bytes) to TBI2CXXX.
 * 
 * @param device TBI2CXXX device descriptor.
 * @param reg_addr TBI2CXXX device register address (1-byte).
 * @param data `uint16_t` (2-byte) word to write to TBI2CXXX.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tbi2cxxx_i2c_write_word_to(tbi2cxxx_device_t *const device, const uint8_t reg_addr, const uint16_t data) {
    bit32_uint8_buffer_t tx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set buffer data and calculate pec */
    tx[0] = device->config.i2c_address << 1;  /*<! i2c device write address */
    tx[1] = reg_addr;                               /*!< i2c eeprom address */
    tx[2] = data & 0xff;                            /*!< low-byte */
    tx[3] = (data >> 8) & 0xff;                     /*!< high-bye */
    uint8_t pec = tbi2cxxx_calculate_pec(tx, 4);/*!< pec */

    /* shift buffer data index up by one and set tx data */
    tx[0] = tx[1]; /*!< i2c eeprom address */
    tx[1] = tx[2]; /*!< low-byte */
    tx[2] = tx[3]; /*!< high-bye */
    tx[3] = pec;   /*!< pec */

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT32_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "unable to transmit, write word failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TBI2CXXX_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Erases a register on TBI2CXXX.
 * 
 * @param handle TBI2CXXX device handle.
 * @param reg_addr TBI2CXXX device register address (1-byte).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tbi2cxxx_erase_register(tbi2cxxx_handle_t handle, const uint8_t reg_addr) {
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_i2c_write_word_to(dev, reg_addr, 0x0000), TAG, "unable to transmit, erase register failed" );

    return ESP_OK;
}

esp_err_t tbi2cxxx_init(i2c_master_bus_handle_t master_handle, const tbi2cxxx_config_t *tbi2cxxx_config, tbi2cxxx_handle_t *tbi2cxxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && tbi2cxxx_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TBI2CXXX_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, tbi2cxxx_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, tbi2cxxx device handle initialization failed", tbi2cxxx_config->i2c_address);

    /* validate memory availability for handle */
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)calloc(1, sizeof(tbi2cxxx_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c tbi2cxxx device, init failed");

    /* copy configuration */
    dev->config = *tbi2cxxx_config;

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
    *tbi2cxxx_handle = (tbi2cxxx_handle_t)dev;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TBI2CXXX_APPSTART_DELAY_MS));

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

esp_err_t tbi2cxxx_get_temperatures(tbi2cxxx_handle_t handle, float *const ambient_temperature, float *const object_temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_get_ambient_temperature(handle, ambient_temperature), TAG, "unable to read ambient temperature from device, get temperatures failed" );
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_get_object_temperature(handle, object_temperature), TAG, "unable to read object temperature from device, get temperatures failed" );

    return ESP_OK;
}

esp_err_t tbi2cxxx_get_ambient_temperature(tbi2cxxx_handle_t handle, float *const temperature) {
    uint16_t encoded_temperature;
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_i2c_read_word_from(dev, TBI2CXXX_CMD_AMB_TEMP_R, &encoded_temperature), TAG, "unable to read word from device, get ambient temperature failed" );

    /* validate maximum range */
    ESP_RETURN_ON_FALSE((encoded_temperature < 0x7fff), ESP_ERR_INVALID_SIZE, TAG, "received word from device is out of range, get ambient temperature failed");

    /* set output parameter */
    *temperature = tbi2cxxx_decode_temperature(encoded_temperature);

    return ESP_OK;
}

esp_err_t tbi2cxxx_get_object_temperature(tbi2cxxx_handle_t handle, float *const temperature) {
    uint16_t encoded_temperature;
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_i2c_read_word_from(dev, TBI2CXXX_CMD_OBJ_TEMP_R, &encoded_temperature), TAG, "unable to read word from device, get object temperature failed" );

    /* validate maximum range */
    ESP_RETURN_ON_FALSE((encoded_temperature < 0x7fff), ESP_ERR_INVALID_SIZE, TAG, "received word from device is out of range, get object temperature failed");

    /* validate msb bit for error flag */
    ESP_RETURN_ON_FALSE(!(encoded_temperature & 0x8000), ESP_ERR_INVALID_RESPONSE, TAG, "error detected with received word from device, get object temperature failed");

    /* set output parameter */
    *temperature = tbi2cxxx_decode_temperature(encoded_temperature);

    return ESP_OK;
}

esp_err_t tbi2cxxx_get_emissivity(tbi2cxxx_handle_t handle, float *const coefficient) {
    uint16_t coefficient_e;
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_i2c_read_word_from(dev, TBI2CXXX_CMD_EMIS_COEF_RW, &coefficient_e), TAG, "unable to read word from device, get emissivity failed" );

    /* set output parameter */
    *coefficient = ((float)coefficient_e + 1.0f) / 65536.0f;

    return ESP_OK;
}

esp_err_t tbi2cxxx_set_emissivity(tbi2cxxx_handle_t handle, const float coefficient) {
    uint16_t coefficient_e;
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* validate emissivity coefficient range is between 0.1 and 1.0 */
    ESP_RETURN_ON_FALSE((coefficient <= 1.0f), ESP_ERR_INVALID_ARG, TAG, "emissivity coefficient range must be between 0.1 and 1.0, set emissivity failed");
    ESP_RETURN_ON_FALSE((coefficient >= 0.1f), ESP_ERR_INVALID_ARG, TAG, "emissivity coefficient range must be between 0.1 and 1.0, set emissivity failed");

    /* set emissivity coefficient (round down) */
    coefficient_e = (uint16_t)floorf(((coefficient * 65536.0f) - 1.0f));

    /* attempt i2c write transaction to erase register */
    ESP_RETURN_ON_ERROR( tbi2cxxx_erase_register(handle, TBI2CXXX_CMD_EMIS_COEF_RW), TAG, "unable to erase register on device, set emissivity failed" );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( tbi2cxxx_i2c_write_word_to(dev, TBI2CXXX_CMD_EMIS_COEF_RW, coefficient_e), TAG, "unable to write word from device, set emissivity failed" );

    return ESP_OK;
}

esp_err_t tbi2cxxx_remove(tbi2cxxx_handle_t handle) {
    tbi2cxxx_device_t* dev = (tbi2cxxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t tbi2cxxx_delete(tbi2cxxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( tbi2cxxx_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* tbi2cxxx_get_fw_version(void) {
    return (const char*)TBI2CXXX_FW_VERSION_STR;
}

int32_t tbi2cxxx_get_fw_version_number(void) {
    return (int32_t)TBI2CXXX_FW_VERSION_INT32;
}