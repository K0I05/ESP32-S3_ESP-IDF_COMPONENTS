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
 * @file sgp4x.c
 *
 * ESP-IDF driver for SGP4X sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/sgp4x.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * SGP4X definitions
 */
#define SGP4X_CRC8_G_POLYNOM            UINT8_C(0x31)       //!< sgp4x I2C CRC8 polynomial

#define SGP4X_CMD_RESET                 UINT16_C(0x0006)
#define SGP4X_CMD_RESET_                UINT8_C(0x06)       //!< sgp4x I2C soft-reset command - for some reason this is an 1-byte command
#define SGP4X_CMD_SERIAL_NUMBER         UINT16_C(0x3682)    //!< sgp4x I2C serial number request command
#define SGP4X_CMD_EXEC_CONDITIONING     UINT16_C(0x2612)    //!< sgp4x I2C
#define SGP4X_CMD_MEAS_RAW_SIGNALS      UINT16_C(0x2619)    //!< sgp4x I2C
#define SGP4X_CMD_EXEC_SELF_TEST        UINT16_C(0x280e)    //!< sgp4x I2C
#define SGP4X_CMD_TURN_HEATER_OFF       UINT16_C(0x3615)    //!< sgp4x I2C

#define SGP4X_HUMIDITY_COMP_DEFAULT     (float)(50.0)
#define SGP4X_TEMPERATURE_COMP_DEFAULT  (float)(25.0)

#define SGP4X_TEMPERATURE_MAX           (float)(130.0)
#define SGP4X_TEMPERATURE_MIN           (float)(-45.0)
#define SGP4X_HUMIDITY_MAX              (float)(100.0)
#define SGP4X_HUMIDITY_MIN              (float)(0.0)

#define SGP4X_CONDITIONING_TIMEOUT_MS   UINT16_C(1000) /*!< conditioning timeout 1-sec, 10-sec in total and maximum */
#define SGP4X_POWERUP_DELAY_MS          UINT16_C(10)
#define SGP4X_APPSTART_DELAY_MS         UINT16_C(10)    /*!< delay after initialization before application start-up */ 
#define SGP4X_CMD_DELAY_MS              UINT16_C(5)     /*!< delay before attempting I2C transactions after a command is issued */
#define SGP4X_RETRY_DELAY_MS            UINT16_C(2)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds


/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief SGP4X device descriptor structure definition.
 */
typedef struct sgp4x_device_s {
    sgp4x_config_t          config;         /*!< sgp4x device configuration */
    i2c_master_dev_handle_t i2c_handle;     /*!< sgp4x i2c device handle */
    uint64_t                serial_number;  /*!< sgp4x serial number */
} sgp4x_device_t;

/*
* static constant declarations
*/
static const char *TAG = "sgp4x";

/*
* functions and subroutines
*/


/**
 * @brief SGP4X I2C HAL read transaction.
 * 
 * @param device SGP4X device descriptor.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sgp4x_i2c_read(sgp4x_device_t *const device, uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read failed" );

    return ESP_OK;
}

/**
 * @brief SGP4X I2C HAL write transaction.
 * 
 * @param device SGP4X device descriptor.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sgp4x_i2c_write(sgp4x_device_t *const device, const uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief SGP4X I2C HAL write command to register address transaction.
 * 
 * @param device SGP4X device descriptor.
 * @param reg_addr SGP4X command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sgp4x_i2c_write_command(sgp4x_device_t *const device, const uint16_t reg_addr) {
    const bytes_to_uint16_t   cmd = { .value = reg_addr };
    const bit16_uint8_buffer_t tx = { cmd.bytes[1], cmd.bytes[0] };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Calculates SGP4X crc8 value for 2-byte data packet.  See datasheet for details.
 *
 * @param[in] data[2] 2-byte data packet to perform crc8 check against.
 * @return uint8_t Calculated crc8 value.
 */
static inline uint8_t sgp4x_calculate_crc8(const uint8_t data[2]) {
    uint8_t crc = 0xff;
    for (size_t i = 0; i < 2; i++) {
        crc ^= data[i];
        for (size_t i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ SGP4X_CRC8_G_POLYNOM : crc << 1;
    }
    return crc;
}

/**
 * @brief Converts `uint16_t` variable from little endian order to
 * big endian order.
 * 
 * @param[in] leo Variable in little endian order.
 * @return uint16_t Converted variable in big endian order
 */
static inline uint16_t sgp4x_leo_to_beo(const uint16_t leo) {
    return (leo << 8) | (leo >> 8);
}

/**
 * @brief Gets the command execution duration in milli-seconds.  This delay is used after the I2C
 * write transaction to give the SGP4X time to process the command and return the results.
 * 
 * @param[in] command Command to determine duration against.
 * @return uint16_t Command execution duration in milli-seconds.
 */
static inline uint16_t sgp4x_get_command_duration(const uint16_t command) {
    switch(command) {
        case SGP4X_CMD_EXEC_CONDITIONING:
            return 50;
        case SGP4X_CMD_MEAS_RAW_SIGNALS:
            return 50;
        case SGP4X_CMD_EXEC_SELF_TEST:
            return 320;
        case SGP4X_CMD_TURN_HEATER_OFF:
            return 1;
        case SGP4X_CMD_SERIAL_NUMBER:
            return 1;
        case SGP4X_CMD_RESET:
            return 5;
        default:
            return 50;
    }
}

/**
 * @brief Converts temperature to compensation ticks.
 * 
 * @param[in] temperature Temperature in degrees Celsius.
 * @return bytes_to_uint16_t Temperature compensation ticks
 */
static inline bytes_to_uint16_t sgp4x_temperature_to_ticks(const float temperature) {
    const bytes_to_uint16_t ticks = {.value = (uint16_t)(temperature + 45) * 65535 / 175};
    return ticks;
}

/**
 * @brief Converts humidity to compensation ticks.
 * 
 * @param[in] humidity Humidity in precentage.
 * @return bytes_to_uint16_t Humidity compensation ticks.
 */
static inline bytes_to_uint16_t sgp4x_humidity_to_ticks(const float humidity) {
    const bytes_to_uint16_t ticks = {.value = (uint16_t)llround(humidity * 65535 / 100)};
    return ticks;
}

/**
 * @brief SGP4X I2C HAL read serial number register.
 * 
 * @param[in] device SGP4X device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sgp4x_i2c_get_serial_number_register(sgp4x_device_t *const device, uint64_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_write_command(device, SGP4X_CMD_SERIAL_NUMBER), TAG, "unable to write to i2c device handle, read serial number failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(sgp4x_get_command_duration(SGP4X_CMD_SERIAL_NUMBER)));

    /* attempt i2c read transaction */
    bit72_uint8_buffer_t rx_buffer = { 0 };
    ESP_RETURN_ON_ERROR( sgp4x_i2c_read(device, rx_buffer, BIT72_UINT8_BUFFER_SIZE), TAG, "unable to read to i2c device handle, read serial number failed");

    /* set 2-byte serial number parts */
    const bytes_to_uint16_t sn_1 = {
        .bytes[0] = rx_buffer[0],
        .bytes[1] = rx_buffer[1]
    };
    const bytes_to_uint16_t sn_2 = {
        .bytes[0] = rx_buffer[3],
        .bytes[1] = rx_buffer[4]
    };
    const bytes_to_uint16_t sn_3 = {
        .bytes[0] = rx_buffer[6],
        .bytes[1] = rx_buffer[7]
    };

    /* validate crc for each serial number part */
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(sn_1.bytes) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 with serial number part 1, read serial number failed" );
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(sn_2.bytes) == rx_buffer[5]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 with serial number part 2, read serial number failed" );
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(sn_3.bytes) == rx_buffer[8]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 with serial number part 3, read serial number failed" );

    /* set output parameter */
    *reg = (((uint64_t)sn_1.value) << 32) | (((uint64_t)sn_2.value) << 16) | ((uint64_t)sn_3.value);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(SGP4X_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief SGP4X I2C HAL write reset command.
 * 
 * @param device SGP4X device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sgp4x_i2c_set_reset(sgp4x_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_write_command(device, SGP4X_CMD_RESET), TAG, "unable to write to i2c device handle, soft-reset failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(sgp4x_get_command_duration(SGP4X_CMD_RESET)));

    return ESP_OK;
}

esp_err_t sgp4x_init(i2c_master_bus_handle_t master_handle, const sgp4x_config_t *sgp4x_config, sgp4x_handle_t *sgp4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && sgp4x_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(SGP4X_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, sgp4x_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, sgp4x device handle initialization failed", sgp4x_config->i2c_address);

    /* validate memory availability for handle */
    sgp4x_device_t* device = (sgp4x_device_t*)calloc(1, sizeof(sgp4x_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for device, sgp4x device handle initialization failed");

    /* copy configuration */
    device->config = *sgp4x_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "unable to add device to master bus, sgp4x device handle initialization failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* attempt to reset the device */
    //ESP_GOTO_ON_ERROR(sgp4x_i2c_set_reset(device), err_handle, TAG, "unable to soft-reset device, sgp4x device handle initialization failed");

    /* attempt to read device serial number */
    ESP_GOTO_ON_ERROR(sgp4x_i2c_get_serial_number_register(device, &device->serial_number), err_handle, TAG, "unable to read device serial number, sgp4x device handle initialization failed");

    /* set device handle */
    *sgp4x_handle = (sgp4x_handle_t)device;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(SGP4X_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t sgp4x_execute_compensated_conditioning(sgp4x_handle_t handle, const float temperature, const float humidity, uint16_t *sraw_voc) {
    const uint8_t               rx_retry_max   = 5;
    const bytes_to_uint16_t     command        = { .value = SGP4X_CMD_EXEC_CONDITIONING };
    esp_err_t                   ret            = ESP_OK;
    uint8_t                     rx_retry_count = 0;
    bytes_to_uint16_t           crc8_buffer    = { .value = 0 };
    bit64_uint8_buffer_t        tx_buffer      = { 0 };
    bit24_uint8_buffer_t        rx_buffer      = { 0 };
    sgp4x_device_t*             device         = (sgp4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    // validate range of temperature compensation parameter
    if(temperature > SGP4X_TEMPERATURE_MAX || temperature < SGP4X_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, execute compensated conditioning failed");
    }

    // validate range of humidity compensation parameter
    if(humidity > SGP4X_HUMIDITY_MAX || humidity < SGP4X_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, execute compensated conditioning failed");
    }

    /* convert compensation parameters to ticks */
    const bytes_to_uint16_t temperature_ticks = sgp4x_temperature_to_ticks(temperature);
    const bytes_to_uint16_t humidity_ticks    = sgp4x_humidity_to_ticks(humidity);

    /* calculate crc8 for compensation parameter ticks - big-endian order */
    crc8_buffer.bytes[0] = temperature_ticks.bytes[1];
    crc8_buffer.bytes[1] = temperature_ticks.bytes[0];
    const uint8_t temperature_ticks_crc8 = sgp4x_calculate_crc8(crc8_buffer.bytes);
    crc8_buffer.bytes[0] = humidity_ticks.bytes[1];
    crc8_buffer.bytes[1] = humidity_ticks.bytes[0];
    const uint8_t humidity_ticks_crc8 = sgp4x_calculate_crc8(crc8_buffer.bytes);

    /* construct tx packet - big-endian order */
    tx_buffer[0] = command.bytes[1];
    tx_buffer[1] = command.bytes[0];
    tx_buffer[2] = humidity_ticks.bytes[1];
    tx_buffer[3] = humidity_ticks.bytes[0];
    tx_buffer[4] = humidity_ticks_crc8;
    tx_buffer[5] = temperature_ticks.bytes[1];
    tx_buffer[6] = temperature_ticks.bytes[0];
    tx_buffer[7] = temperature_ticks_crc8;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_write(device, tx_buffer, BIT64_UINT8_BUFFER_SIZE), TAG, "unable to write to i2c device handle, execute compensated conditioning failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(sgp4x_get_command_duration(SGP4X_CMD_EXEC_CONDITIONING)));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = sgp4x_i2c_read(device, rx_buffer, BIT24_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(SGP4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, execute compensated conditioning failed" );

    /* validate crc from rx result - little-endian order */
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(rx_buffer) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8, execute compensated conditioning failed" );

    /* set output parameter */
    *sraw_voc = (uint16_t)rx_buffer[0] << 8 | (uint16_t)rx_buffer[1];

    return ESP_OK;
}

esp_err_t sgp4x_execute_conditioning(sgp4x_handle_t handle, uint16_t *sraw_voc) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to execute compensated conditioning */
    ESP_RETURN_ON_ERROR( sgp4x_execute_compensated_conditioning(handle, SGP4X_TEMPERATURE_COMP_DEFAULT, SGP4X_HUMIDITY_COMP_DEFAULT, sraw_voc), TAG, "unable to write to i2c device handle, execute conditioning failed");

    return ESP_OK;
}

esp_err_t sgp4x_measure_compensated_signals(sgp4x_handle_t handle, const float temperature, const float humidity, uint16_t *sraw_voc, uint16_t *sraw_nox) {
    const uint8_t               rx_retry_max   = 5;
    const bytes_to_uint16_t     command        = { .value = SGP4X_CMD_MEAS_RAW_SIGNALS };
    esp_err_t                   ret            = ESP_OK;
    uint8_t                     rx_retry_count = 0;
    bytes_to_uint16_t           crc8_buffer    = { .value = 0 };
    bit64_uint8_buffer_t        tx_buffer      = { 0 };
    bit48_uint8_buffer_t        rx_buffer      = { 0 };
    sgp4x_device_t*             device         = (sgp4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    // validate range of temperature compensation parameter
    if(temperature > SGP4X_TEMPERATURE_MAX || temperature < SGP4X_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, measure compensated raw signals failed");
    }

    // validate range of humidity compensation parameter
    if(humidity > SGP4X_HUMIDITY_MAX || humidity < SGP4X_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, measure compensated raw signals failed");
    }

    /* convert compensation parameters to ticks */
    const bytes_to_uint16_t temperature_ticks = sgp4x_temperature_to_ticks(temperature);
    const bytes_to_uint16_t humidity_ticks    = sgp4x_humidity_to_ticks(humidity);

    /* calculate crc8 for compensation parameter ticks - big-endian order */
    crc8_buffer.bytes[0] = temperature_ticks.bytes[1];
    crc8_buffer.bytes[1] = temperature_ticks.bytes[0];
    const uint8_t temperature_ticks_crc8 = sgp4x_calculate_crc8(crc8_buffer.bytes);
    crc8_buffer.bytes[0] = humidity_ticks.bytes[1];
    crc8_buffer.bytes[1] = humidity_ticks.bytes[0];
    const uint8_t humidity_ticks_crc8 = sgp4x_calculate_crc8(crc8_buffer.bytes);

    /* construct tx packet - big-endian order */
    tx_buffer[0] = command.bytes[1];
    tx_buffer[1] = command.bytes[0];
    tx_buffer[2] = humidity_ticks.bytes[1];
    tx_buffer[3] = humidity_ticks.bytes[0];
    tx_buffer[4] = humidity_ticks_crc8;
    tx_buffer[5] = temperature_ticks.bytes[1];
    tx_buffer[6] = temperature_ticks.bytes[0];
    tx_buffer[7] = temperature_ticks_crc8;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_write(device, tx_buffer, BIT64_UINT8_BUFFER_SIZE), TAG, "unable to write to i2c device handle, measure compensated raw signals failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(sgp4x_get_command_duration(SGP4X_CMD_MEAS_RAW_SIGNALS)));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = sgp4x_i2c_read(device, rx_buffer, BIT48_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(SGP4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, measure compensated raw signals failed" );

    /* validate crc from rx result - little-endian order */
    crc8_buffer.bytes[0] = rx_buffer[0];
    crc8_buffer.bytes[1] = rx_buffer[1];
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(crc8_buffer.bytes) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 for sraw_voc, measure compensated raw signals failed" );
    crc8_buffer.bytes[0] = rx_buffer[3];
    crc8_buffer.bytes[1] = rx_buffer[4];
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(crc8_buffer.bytes) == rx_buffer[5]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8 for sraw_nox, measure compensated raw signals failed" );

    /* set output parameters */
    *sraw_voc = (uint16_t)rx_buffer[0] << 8 | (uint16_t)rx_buffer[1];
    *sraw_nox = (uint16_t)rx_buffer[3] << 8 | (uint16_t)rx_buffer[4];

    return ESP_OK;
}

esp_err_t sgp4x_measure_signals(sgp4x_handle_t handle, uint16_t *sraw_voc, uint16_t *sraw_nox) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to execute compensated raw signals */
    ESP_RETURN_ON_ERROR(sgp4x_measure_compensated_signals(handle, 25.0f, 50.0f, sraw_voc, sraw_nox), TAG, "unable to write to i2c device handle, measure raw signals failed");

    return ESP_OK;
}

esp_err_t sgp4x_execute_self_test(sgp4x_handle_t handle, sgp4x_self_test_result_t *const result) {
    const uint8_t rx_retry_max     = 5;
    esp_err_t     ret              = ESP_OK;
    uint8_t       rx_retry_count   = 0;
    bit24_uint8_buffer_t rx_buffer = { 0 };
    sgp4x_device_t* device         = (sgp4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_write_command(device, SGP4X_CMD_EXEC_SELF_TEST), TAG, "unable to write to i2c device handle, execute self-test failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(sgp4x_get_command_duration(SGP4X_CMD_EXEC_SELF_TEST)));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = sgp4x_i2c_read(device, rx_buffer, BIT24_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(SGP4X_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, execute self-test failed" );

    /* validate crc from result */
    ESP_RETURN_ON_FALSE( (sgp4x_calculate_crc8(rx_buffer) == rx_buffer[2]), ESP_ERR_INVALID_CRC, TAG, "invalid crc8, execute self-test failed" );

    /* set results - lsb */
    result->integrity = rx_buffer[0];

    return ESP_OK;
}

esp_err_t sgp4x_disable_heater(sgp4x_handle_t handle) {
    sgp4x_device_t* device = (sgp4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_write_command(device, SGP4X_CMD_TURN_HEATER_OFF), TAG, "unable to write to i2c device handle, turn off heater failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(sgp4x_get_command_duration(SGP4X_CMD_TURN_HEATER_OFF)));

    return ESP_OK;
}

esp_err_t sgp4x_reset(sgp4x_handle_t handle) {
    sgp4x_device_t* device = (sgp4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sgp4x_i2c_set_reset(device), TAG, "unable to write to i2c device handle, soft-reset failed");

    return ESP_OK;
}

esp_err_t sgp4x_remove(sgp4x_handle_t handle) {
    sgp4x_device_t* device = (sgp4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate handle instance */
    if(device->i2c_handle) {
        /* remove device from i2c master bus */
        esp_err_t ret = i2c_master_bus_rm_device(device->i2c_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_rm_device failed");
            return ret;
        }
        device->i2c_handle = NULL;
    }

    return ESP_OK;
}

esp_err_t sgp4x_delete(sgp4x_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    esp_err_t ret = sgp4x_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}

const char* sgp4x_get_fw_version(void) {
    return (const char*)SGP4X_FW_VERSION_STR;
}

int32_t sgp4x_get_fw_version_number(void) {
    return (int32_t)SGP4X_FW_VERSION_INT32;
}