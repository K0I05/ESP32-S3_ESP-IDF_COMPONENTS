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
 * @file sht4x.c
 *
 * ESP-IDF driver for SHT4x air temperature and relative humidity sensor
 * 
 * https://github.com/Sensirion/embedded-sht/releases
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/sht4x.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * SHT4X definitions
*/

#define SHT4X_CRC8_MASK             UINT8_C(0x80)   /*!< sht4x I2C CRC8 mask */
#define SHT4X_CRC8_INIT             UINT8_C(0xff)   /*!< sht4x I2C CRC8 initialization */
#define SHT4X_CRC8_POLYNOM          UINT8_C(0x31)   //!< sht4x I2C CRC8 polynomial

#define SHT4X_CMD_RESET             UINT8_C(0x94)   //!< sht4x I2C soft-reset command 
#define SHT4X_CMD_SERIAL            UINT8_C(0x89)   //!< sht4x I2C serial number request command
#define SHT4X_CMD_MEAS_HIGH         UINT8_C(0xFD)   //!< sht4x I2C high resolution measurement command
#define SHT4X_CMD_MEAS_MED          UINT8_C(0xF6)   //!< sht4x I2C medium resolution measurement command
#define SHT4X_CMD_MEAS_LOW          UINT8_C(0xE0)   //!< sht4x I2C low resolution measurement command
#define SHT4X_CMD_MEAS_H_HIGH_LONG  UINT8_C(0x39)   //!< sht4x I2C high resolution measurement command with heater enabled long pulse
#define SHT4X_CMD_MEAS_H_HIGH_SHORT UINT8_C(0x32)   //!< sht4x I2C high resolution measurement command with heater enabled short pulse
#define SHT4X_CMD_MEAS_H_MED_LONG   UINT8_C(0x2F)   //!< sht4x I2C medium resolution measurement command with heater enabled long pulse
#define SHT4X_CMD_MEAS_H_MED_SHORT  UINT8_C(0x24)   //!< sht4x I2C medium resolution measurement command with heater enabled short pulse
#define SHT4X_CMD_MEAS_H_LOW_LONG   UINT8_C(0x1E)   //!< sht4x I2C low resolution measurement command with heater enabled long pulse
#define SHT4X_CMD_MEAS_H_LOW_SHORT  UINT8_C(0x15)   //!< sht4x I2C low resolution measurement command with heater enabled short pulse

#define SHT4X_TEMPERATURE_MAX       (float)(125.0)  //!< sht4x maximum temperature range
#define SHT4X_TEMPERATURE_MIN       (float)(-40.0)  //!< sht4x minimum temperature range
#define SHT4X_HUMIDITY_MAX          (float)(100.0)  //!< sht4x maximum humidity range
#define SHT4X_HUMIDITY_MIN          (float)(0.0)    //!< sht4x minimum humidity range

#define SHT4X_POWERUP_DELAY_MS      UINT16_C(5)     /*!< sht4x delay on power-up before attempting I2C transactions */
#define SHT4X_APPSTART_DELAY_MS     UINT16_C(10)    /*!< sht4x delay after initialization before application start-up */
#define SHT4X_RESET_DELAY_MS        UINT16_C(25)    /*!< sht4x delay before attempting I2C transactions after a reset is issued */
#define SHT4X_CMD_DELAY_MS          UINT16_C(5)     /*!< sht4x delay before attempting I2C transactions after a command is issued */
#define SHT4X_RETRY_DELAY_MS        UINT16_C(2)     /*!< sht4x delay between an I2C receive transaction retry */
#define SHT4X_TX_RX_DELAY_MS        UINT16_C(10)    /*!< sht4x delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief SHT4X device descriptor structure definition.
 */
typedef struct sht4x_device_s {
    sht4x_config_t           config;          /*!< sht4x device configuration */
    i2c_master_dev_handle_t  i2c_handle;      /*!< sht4x i2c device handle */
    uint32_t                 serial_number;   /*!< sht4x device serial number */
} sht4x_device_t;

/*
* static constant declarations
*/
static const char *TAG = "sht4x";

/*
* functions and subroutines
*/



/**
 * @brief SHT4X I2C HAL read transaction.
 * 
 * @param device SHT4X device descriptor.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sht4x_i2c_read(sht4x_device_t *const device, uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read failed" );

    return ESP_OK;
}

/**
 * @brief SHT4X I2C HAL write transaction.
 * 
 * @param device SHT4X device descriptor.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sht4x_i2c_write(sht4x_device_t *const device, const uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief SHT4X I2C HAL write command to register address transaction.
 * 
 * @param device SHT4X device descriptor.
 * @param reg_addr SHT4X command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sht4x_i2c_write_command(sht4x_device_t *const device, const uint8_t reg_addr) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Calculates SHT4X CRC8 value.  See datasheet for details.
 *
 * @param[in] data[] Data buffer to perform CRC8 calculation against.
 * @param[in] len Length of data buffer.
 * @return uint8_t Calculated CRC8 value.
 */
static inline uint8_t sht4x_calculate_crc8(const uint8_t data[], const uint8_t len) {
    uint8_t crc = SHT4X_CRC8_INIT; /* crc initial value */
    for (uint8_t byte = 0; byte < len; byte++) {
        crc ^= data[byte];
        for (uint8_t i = 0; i < 8; i++) {
            crc = crc & SHT4X_CRC8_MASK ? (uint8_t)(crc << 1) ^ SHT4X_CRC8_POLYNOM : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Gets SHT4X measurement duration in milliseconds from device handle.  See datasheet for details.
 *
 * @param[in] device SHT4X device descriptor.
 * @return size_t Measurement duration in milliseconds.
 */
static inline size_t sht4x_get_duration(sht4x_device_t *const device) {
    /* validate arguments */
    if (!device) return 2;
    switch (device->config.heater_mode) {
        case SHT4X_HEATER_HIGH_LONG:
        case SHT4X_HEATER_MEDIUM_LONG:
        case SHT4X_HEATER_LOW_LONG:
            return 1100;
        case SHT4X_HEATER_HIGH_SHORT:
        case SHT4X_HEATER_MEDIUM_SHORT:
        case SHT4X_HEATER_LOW_SHORT:
            return 110;
        default:
            switch (device->config.repeat_mode) {
                case SHT4X_REPEAT_HIGH:
                    return 10;
                case SHT4X_REPEAT_MEDIUM:
                    return 5; 
                default:
                    return 2;
            }
    }
}

/**
 * @brief Gets SHT4X measurement tick duration from device handle.
 *
 * @param[in] device SHT4X device descriptor.
 * @return size_t Measurement duration in ticks.
 */
static inline size_t sht4x_get_tick_duration(sht4x_device_t *const device) {
    /* validate arguments */
    if (!device) return 1;
    size_t res = pdMS_TO_TICKS(sht4x_get_duration(device));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets SHT4X measurement command from device handle parameters.  See datasheet for details.
 *
 * @param[in] device SHT4X device descriptor.
 * @return uint8_t SHT4X command value.
 */
static inline uint8_t sht4x_get_command(sht4x_device_t *const device) {
    /* validate arguments */
    if (!device) return SHT4X_CMD_MEAS_LOW;
    switch (device->config.heater_mode) {
        case SHT4X_HEATER_HIGH_LONG:
            return SHT4X_CMD_MEAS_H_HIGH_LONG;
        case SHT4X_HEATER_HIGH_SHORT:
            return SHT4X_CMD_MEAS_H_HIGH_SHORT;
        case SHT4X_HEATER_MEDIUM_LONG:
            return SHT4X_CMD_MEAS_H_MED_LONG;
        case SHT4X_HEATER_MEDIUM_SHORT:
            return SHT4X_CMD_MEAS_H_MED_SHORT;
        case SHT4X_HEATER_LOW_LONG:
            return SHT4X_CMD_MEAS_H_LOW_LONG;
        case SHT4X_HEATER_LOW_SHORT:
            return SHT4X_CMD_MEAS_H_LOW_SHORT;
        default:
            switch (device->config.repeat_mode) {
                case SHT4X_REPEAT_HIGH:
                    return SHT4X_CMD_MEAS_HIGH;
                case SHT4X_REPEAT_MEDIUM:
                    return SHT4X_CMD_MEAS_MED;
                default:
                    return SHT4X_CMD_MEAS_LOW;
            }
    }
}

/**
 * @brief Calculates dew-point temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humidity in percent.
 * @param[out] dewpoint calculated dew-point temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sht4x_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK(dewpoint);

    // validate range of temperature parameter
    if(temperature > SHT4X_TEMPERATURE_MAX || temperature < SHT4X_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate dew-point failed");
    }

    // validate range of humidity parameter
    if(humidity > SHT4X_HUMIDITY_MAX || humidity < SHT4X_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate dew-point failed");
    }
    
    // calculate dew-point temperature
    float H = (log10f(humidity)-2)/0.4343f + (17.62f*temperature)/(243.12f+temperature);
    *dewpoint = 243.12f*H/(17.62f-H);
    
    return ESP_OK;
}

/**
 * @brief Reads serial number from SHT4X.
 *
 * @param[in] device SHT4X device descriptor.
 * @param[out] serial_number SHT4X serial number. 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t sht4x_i2c_get_serial_number(sht4x_device_t *const device, uint32_t *const serial_number) {
    const bit8_uint8_buffer_t tx = { SHT4X_CMD_SERIAL };
    bit48_uint8_buffer_t      rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sht4x_i2c_write(device, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "unable to write to i2c device handle, get serial number failed");
	
    /* delay before attempting i2c read transaction */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( sht4x_i2c_read(device, rx, BIT48_UINT8_BUFFER_SIZE), TAG, "unable to read to i2c device handle, get serial number failed");
	
    /* set serial number */
    *serial_number = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[3] << 8) | rx[4];

    return ESP_OK;
}

static inline esp_err_t sht4x_i2c_set_reset(sht4x_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( sht4x_i2c_write_command(device, SHT4X_CMD_RESET), TAG, "unable to write to device handle, device reset failed");

    /* delay before next command - soft-reset */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t sht4x_init(i2c_master_bus_handle_t master_handle, const sht4x_config_t *sht4x_config, sht4x_handle_t *sht4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && (sht4x_config || sht4x_handle) );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, sht4x_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, device handle initialization failed", sht4x_config->i2c_address);

    /* validate memory availability for handle */
    sht4x_device_t* device = (sht4x_device_t*)calloc(1, sizeof(sht4x_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for device, device handle initialization failed");

    /* copy configuration */
    device->config = *sht4x_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "unable to add device to master bus, device handle initialization failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_CMD_DELAY_MS));

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(sht4x_i2c_set_reset(device), err_handle, TAG, "unable to soft-reset device, device handle initialization failed");

    /* sht4x attempt to read device serial number */
    ESP_GOTO_ON_ERROR(sht4x_i2c_get_serial_number(device, &device->serial_number), err_handle, TAG, "unable to read serial number, device reset failed");

    /* set device handle */
    *sht4x_handle = (sht4x_handle_t)device;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t sht4x_get_measurement(sht4x_handle_t handle, float *const temperature, float *const humidity) {
    const uint8_t rx_retry_max  = 5;
    uint8_t rx_retry_count      = 0;
    size_t delay_ticks 			= 0;
    esp_err_t ret               = ESP_OK;
    bit8_uint8_buffer_t tx      = { 0 };
    bit48_uint8_buffer_t rx     = { 0 };
    sht4x_device_t* device      = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && (temperature || humidity) );
    
    /* get command and measurement duration from handle settings */
    tx[0]       = sht4x_get_command(device);
    delay_ticks = sht4x_get_tick_duration(device);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( sht4x_i2c_write(device, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "unable to write to i2c device handle, get measurement failed");
	
	/* delay task - allow time for the sensor to process measurement request */
    if(delay_ticks) vTaskDelay(delay_ticks);

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = sht4x_i2c_read(device, rx, BIT48_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(SHT4X_RETRY_DELAY_MS));
    } while (++rx_retry_count <= rx_retry_max && ret != ESP_OK );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, get measurement failed" );
	
    /* validate crc values */
    if (rx[2] != sht4x_calculate_crc8(rx, 2) || rx[5] != sht4x_calculate_crc8(rx + 3, 2)) {
        return ESP_ERR_INVALID_CRC;
    }

	// convert sht4x results to engineering units of measure (C and %)
    *temperature = (float)((uint16_t)rx[0] << 8 | rx[1]) * 175.0f / 65535.0f - 45.0f;
    *humidity    = (float)((uint16_t)rx[3] << 8 | rx[4]) * 125.0f / 65535.0f - 6.0f;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t sht4x_get_measurements(sht4x_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && (temperature || humidity || dewpoint) );

    ESP_RETURN_ON_ERROR( sht4x_get_measurement(handle, temperature, humidity), TAG, "unable to read measurement, read measurements failed" );

    ESP_RETURN_ON_ERROR( sht4x_calculate_dewpoint(*temperature, *humidity, dewpoint), TAG, "unable to calculate dew-point, read measurements failed" );

    return ESP_OK;
}

esp_err_t sht4x_get_repeat_mode(sht4x_handle_t handle, sht4x_repeat_modes_t *const mode) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *mode = device->config.repeat_mode;

    return ESP_OK;
}

esp_err_t sht4x_set_repeat_mode(sht4x_handle_t handle, const sht4x_repeat_modes_t mode) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set handle setting */
    device->config.repeat_mode = mode;

    return ESP_OK;
}

esp_err_t sht4x_get_heater_mode(sht4x_handle_t handle, sht4x_heater_modes_t *const mode) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *mode = device->config.heater_mode;

    return ESP_OK;
}

esp_err_t sht4x_set_heater_mode(sht4x_handle_t handle, const sht4x_heater_modes_t mode) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set handle setting */
    device->config.heater_mode = mode;

    return ESP_OK;
}

esp_err_t sht4x_reset(sht4x_handle_t handle) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( sht4x_i2c_set_reset(device), TAG, "unable to write to device handle, device reset failed");

    /* sht4x attempt to read device serial number */
    ESP_RETURN_ON_ERROR(sht4x_i2c_get_serial_number(device, &device->serial_number), TAG, "unable to read serial number, device reset failed");

    return ESP_OK;
}

esp_err_t sht4x_remove(sht4x_handle_t handle) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    return i2c_master_bus_rm_device(device->i2c_handle);
}

esp_err_t sht4x_delete(sht4x_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( sht4x_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* sht4x_get_fw_version(void) {
    return (const char*)SHT4X_FW_VERSION_STR;
}

int32_t sht4x_get_fw_version_number(void) {
    return (int32_t)SHT4X_FW_VERSION_INT32;
}