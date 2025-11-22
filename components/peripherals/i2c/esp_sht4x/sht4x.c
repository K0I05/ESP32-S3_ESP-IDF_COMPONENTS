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
 * @file sht4x.c
 *
 * ESP-IDF driver for SHT4x air temperature and relative humidity sensor
 * 
 * https://github.com/Sensirion/embedded-sht/releases
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
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

#define SHT4X_CRC8_MASK             UINT8_C(0x80)   /*!< sht4x CRC8 mask */
#define SHT4X_CRC8_INIT             UINT8_C(0xff)   /*!< sht4x CRC8 initialization */
#define SHT4X_CRC8_POLYNOM          UINT8_C(0x31)   //!< sht4x CRC8 polynomial

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

#define SHT4X_DRYBULB_MAX           (float)(125.0)  //!< sht4x maximum dry-bulb temperature range
#define SHT4X_DRYBULB_MIN           (float)(-40.0)  //!< sht4x minimum dry-bulb temperature range
#define SHT4X_HUMIDITY_MAX          (float)(100.0)  //!< sht4x maximum relative humidity range
#define SHT4X_HUMIDITY_MIN          (float)(0.0)    //!< sht4x minimum relative humidity range

#define SHT4X_POWERUP_DELAY_MS      UINT16_C(5)     /*!< sht4x delay on power-up before attempting I2C transactions */
#define SHT4X_APPSTART_DELAY_MS     UINT16_C(10)    /*!< sht4x delay after initialization before application start-up */
#define SHT4X_RESET_DELAY_MS        UINT16_C(25)    /*!< sht4x delay before attempting I2C transactions after a reset is issued */
#define SHT4X_CMD_DELAY_MS          UINT16_C(5)     /*!< sht4x delay before attempting I2C transactions after a command is issued */
#define SHT4X_RETRY_DELAY_MS        UINT16_C(2)     /*!< sht4x delay between an I2C receive transaction retry */
#define SHT4X_TX_RX_DELAY_MS        UINT16_C(10)    /*!< sht4x delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_XFR_TIMEOUT_MS          UINT16_C(500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief SHT4X device descriptor structure definition.
 */
typedef struct sht4x_device_s {
    sht4x_config_t  config;          /*!< sht4x device configuration */
    void*           hal_handle;      /*!< sht4x HAL device handle */
    uint32_t        serial_number;   /*!< sht4x device serial number */
} sht4x_device_t;

/*
* static constant declarations
*/
static const char *TAG = "sht4x";

/*
* functions and subroutines
*/


/**
 * @brief HAL device probe on master communication bus.
 * 
 * @param master_handle Pointer to HAL master communication bus handle.
 * @param device Pointer to SHT4X device descriptor.
 * @return esp_err_t ESP_OK on success.  ESP_ERR_NOT_FOUND: probe failed, 
 * doesn't find the device with specific address you gave.  ESP_ERR_TIMEOUT: 
 * Operation timeout(larger than xfer_timeout_ms) because the bus is busy or 
 * hardware crash.
 */
static inline esp_err_t hal_master_probe(const void* master_handle, sht4x_device_t *const device) {
    /* cast to i2c master bus handle */
    i2c_master_bus_handle_t hal_master = (i2c_master_bus_handle_t)master_handle;

    /* validate arguments */
    ESP_ARG_CHECK(hal_master && device );

    /* attempt to probe device on i2c master bus */
    esp_err_t ret = i2c_master_probe(hal_master, device->config.i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_RETURN_ON_ERROR(ret, TAG, "device does not exist at address 0x%02x, device probe on HAL master communication bus failed", device->config.i2c_address);

    return ESP_OK;
}

/**
 * @brief HAL device initialization on master communication bus.
 * 
 * @param master_handle Pointer to HAL master communication bus handle.
 * @param device Pointer to SHT4X device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_init(const void* master_handle, sht4x_device_t *const device) {
    /* cast to i2c master bus handle */
    i2c_master_bus_handle_t hal_master = (i2c_master_bus_handle_t)master_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_master && device );

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed
    };

    /* attempt to add device to i2c master bus */
    ESP_RETURN_ON_ERROR( i2c_master_bus_add_device(hal_master, &i2c_dev_conf, (i2c_master_dev_handle_t*)&(device->hal_handle)), TAG, "unable to add device to HAL master communication bus, HAL device initialization failed");

    return ESP_OK;
}


/**
 * @brief HAL read from device transaction.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_read(const void* device_handle, uint8_t *buffer, const uint8_t size) {
    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(hal_device, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, HAL read from device failed" );

    return ESP_OK;
}

/**
 * @brief HAL write to device transaction.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write(const void* device_handle, const uint8_t *buffer, const uint8_t size) {
    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(hal_device, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, HAL write to device failed" );
                        
    return ESP_OK;
}

/**
 * @brief HAL write command to device register address transaction.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @param reg_addr Command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_command(const void* device_handle, const uint8_t reg_addr) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(hal_device, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, HAL write command to device failed" );
                        
    return ESP_OK;
}

/**
 * @brief Remove device from HAL master communication bus and free resources.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_remove(const void* device_handle) {
    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* remove device from i2c master bus */
    esp_err_t ret = i2c_master_bus_rm_device(hal_device);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_rm_device failed");
        return ret;
    }
    hal_device = NULL;

    return ESP_OK;
}

/**
 * @brief Calculates SHT4X CRC8 value.  See datasheet for details.
 *
 * @param[in] data[] Data buffer to perform CRC8 calculation against.
 * @param[in] len Length of data buffer.
 * @return uint8_t Calculated CRC8 value.
 */
static inline uint8_t calculate_crc8(const uint8_t data[], const uint8_t len) {
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
 * @brief Helper: map heater mode to fixed measurement duration (ms); returns 0 if heater not used.
 */
static inline uint16_t get_heater_duration(const sht4x_heater_modes_t hm) {
    switch (hm) {
        case SHT4X_HEATER_HIGH_LONG:
        case SHT4X_HEATER_MEDIUM_LONG:
        case SHT4X_HEATER_LOW_LONG:
            return 1100;
        case SHT4X_HEATER_HIGH_SHORT:
        case SHT4X_HEATER_MEDIUM_SHORT:
        case SHT4X_HEATER_LOW_SHORT:
            return 110;
        default:
            return 0;
    }
}

/**
 * @brief Helper: map repeat mode to measurement duration (ms) for no-heater operation.
 */
static inline uint8_t get_repeat_duration(const sht4x_repeat_modes_t rm) {
    switch (rm) {
        case SHT4X_REPEAT_HIGH:   return 10;
        case SHT4X_REPEAT_MEDIUM: return 5;
        default:                  return 2;
    }
}

/**
 * @brief Gets SHT4X measurement duration in milliseconds from device handle.  See datasheet for details.
 *
 * @param[in] device SHT4X device descriptor.
 * @return uint16_t Measurement duration in milliseconds.
 */
static inline uint16_t get_measurement_duration(sht4x_device_t *const device) {
    if (!device) return 2;
    const uint16_t heater_ms = get_heater_duration(device->config.heater_mode);
    return heater_ms ? heater_ms : get_repeat_duration(device->config.repeat_mode);
}

/**
 * @brief Gets SHT4X measurement tick duration from device handle.
 *
 * @param[in] device SHT4X device descriptor.
 * @return uint32_t Measurement duration in ticks.
 */
static inline uint32_t get_measurement_tick_duration(sht4x_device_t *const device) {
    /* validate arguments */
    if (!device) return 1;
    uint32_t res = pdMS_TO_TICKS(get_measurement_duration(device));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets SHT4X measurement command from device handle parameters.  See datasheet for details.
 *
 * @param[in] device SHT4X device descriptor.
 * @return uint8_t SHT4X command value.
 */
static inline uint8_t map_heater_command(const sht4x_heater_modes_t hm) {
    switch (hm) {
        case SHT4X_HEATER_HIGH_LONG:   return SHT4X_CMD_MEAS_H_HIGH_LONG;
        case SHT4X_HEATER_HIGH_SHORT:  return SHT4X_CMD_MEAS_H_HIGH_SHORT;
        case SHT4X_HEATER_MEDIUM_LONG: return SHT4X_CMD_MEAS_H_MED_LONG;
        case SHT4X_HEATER_MEDIUM_SHORT:return SHT4X_CMD_MEAS_H_MED_SHORT;
        case SHT4X_HEATER_LOW_LONG:    return SHT4X_CMD_MEAS_H_LOW_LONG;
        case SHT4X_HEATER_LOW_SHORT:   return SHT4X_CMD_MEAS_H_LOW_SHORT;
        default:                       return 0;
    }
}

/**
 * @brief Gets SHT4X measurement command from device handle parameters.  See datasheet for details.
 *
 * @param[in] device SHT4X device descriptor.
 * @return uint8_t SHT4X command value.
 */
static inline uint8_t get_command(sht4x_device_t *const device) {
    if (!device) return SHT4X_CMD_MEAS_LOW;
    const uint8_t heater_cmd = map_heater_command(device->config.heater_mode);
    if (heater_cmd) return heater_cmd;
    switch (device->config.repeat_mode) {
        case SHT4X_REPEAT_HIGH:   return SHT4X_CMD_MEAS_HIGH;
        case SHT4X_REPEAT_MEDIUM: return SHT4X_CMD_MEAS_MED;
        default:                  return SHT4X_CMD_MEAS_LOW;
    }
}

/**
 * @brief Helper: check if value is in range.
 * 
 * @param value value to check.
 * @param min minimum acceptable value.
 * @param max maximum acceptable value.
 * @return true if value is in range, false otherwise.
 */
static inline bool is_value_in_range(const float value, const float min, const float max) {
    return (value <= max && value >= min);
}

/**
 * @brief Helper: check if dry-bulb temperature is in range.
 * 
 * @param drybulb dry-bulb temperature in degrees Celsius.
 * @return true if dry-bulb temperature is in range, false otherwise.
 */
static inline bool is_drybulb_in_range(const float drybulb) {
    return is_value_in_range(drybulb, SHT4X_DRYBULB_MIN, SHT4X_DRYBULB_MAX);
}

/**
 * @brief Helper: check if relative humidity is in range.
 * 
 * @param humidity relative humidity in percent.
 * @return true if relative humidity is in range, false otherwise.
 */
static inline bool is_humidity_in_range(const float humidity) {
    return is_value_in_range(humidity, SHT4X_HUMIDITY_MIN, SHT4X_HUMIDITY_MAX);
}

/**
 * @brief Gets calculated dew-point temperature from dry-bulb temperature and relative humidity.
 *
 * @param[in] drybulb Dry-bulb temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percent.
 * @param[out] dewpoint Calculated dew-point temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t get_dewpoint(const float drybulb, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK(dewpoint);

    /* validate range of dry-bulb temperature parameter */
    ESP_RETURN_ON_FALSE( is_drybulb_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated dew-point temperature failed");

    /* validate range of relative humidity parameter */
    ESP_RETURN_ON_FALSE( is_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated dew-point temperature failed");
    
    /* calculate dew-point temperature */
    const float H = (log10f(humidity)-2)/0.4343f + (17.62f*drybulb)/(243.12f+drybulb);
    *dewpoint = 243.12f*H/(17.62f-H);
    
    return ESP_OK;
}

/**
 * @brief Gets calculated wet-bulb temperature from dry-bulb temperature and relative humidity.
 *
 * @param[in] drybulb Dry-bulb temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percent.
 * @param[out] wetbulb Calculated wet-bulb temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t get_wetbulb(const float drybulb, const float humidity, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK(wetbulb);

    /* validate range of dry-bulb temperature parameter */
    ESP_RETURN_ON_FALSE( is_drybulb_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated wet-bulb temperature failed");

    /* validate range of relative humidity parameter */
    ESP_RETURN_ON_FALSE( is_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated wet-bulb temperature failed");
    
    /* calculate wet-bulb temperature */
    *wetbulb = drybulb * atanf( 0.151977f * powf( (humidity + 8.313659f), 1.0f/2.0f ) ) + atanf(drybulb + humidity) - atanf(humidity - 1.676331f) + 0.00391838f * powf(humidity, 3.0f/2.0f) * atanf(0.023101f * humidity) - 4.686035f;
    
    return ESP_OK;
}

/**
 * @brief HAL read serial number register from SHT4X.
 *
 * @param[in] device SHT4X device descriptor.
 * @param[out] serial_number SHT4X serial number. 
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_serial_number_register(sht4x_device_t *const device, uint32_t *const serial_number) {
    const bit8_uint8_buffer_t tx = { SHT4X_CMD_SERIAL };
    bit48_uint8_buffer_t      rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "unable to write to device handle, get serial number failed");
	
    /* delay before attempting read transaction */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_TX_RX_DELAY_MS));

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read(device->hal_handle, rx, BIT48_UINT8_BUFFER_SIZE), TAG, "unable to read to device handle, get serial number failed");

    /**
    * The serial number is returned as two 16-bit words, each with its own CRC.
    * [MSB1, LSB1, CRC1, MSB2, LSB2, CRC2]
    */
    if (rx[2] != calculate_crc8(rx, 2) || rx[5] != calculate_crc8(rx + 3, 2)) {
        return ESP_ERR_INVALID_CRC;
    }
	
    /* set serial number */
    *serial_number = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[3] << 8) | (uint32_t)rx[4];

    return ESP_OK;
}

static inline esp_err_t hal_set_reset_register(sht4x_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( hal_master_write_command(device->hal_handle, SHT4X_CMD_RESET), TAG, "unable to write to device handle, device reset failed");

    /* delay before next command - soft-reset */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_RESET_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Converts SHT4X ADC temperature signal to temperature in degrees Celsius.
 *
 * @param[in] adc_signal ADC temperature signal from SHT4X.
 * @return float temperature in degrees Celsius.
 */
static inline float convert_adc_signal_to_temperature(const uint16_t adc_signal) {
    return (float)adc_signal * 175.0f / 65535.0f - 45.0f;
}

/**
 * @brief Converts SHT4X ADC humidity signal to relative humidity percentage.
 * 
 * @param[in] adc_signal ADC humidity signal from SHT4X.
 * @return float relative humidity in percent.
 */
static inline float convert_adc_signal_to_humidity(const uint16_t adc_signal) {
    return (float)adc_signal * 125.0f / 65535.0f - 6.0f;
}

/**
 * @brief HAL read raw ADC temperature and humidity signals from SHT4X.
 *
 * @param[in] device SHT4X device descriptor.
 * @param[out] temperature raw temperature signal from SHT4X.
 * @param[out] humidity raw humidity signal from SHT4X.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_adc_signals(sht4x_device_t *const device, uint16_t *const temperature, uint16_t *const humidity) {
    const uint8_t rx_retry_max  = 5;
    uint8_t rx_retry_count      = 0;
    esp_err_t ret               = ESP_OK;
    bit48_uint8_buffer_t rx     = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );
    
    /* get command and measurement duration from handle settings */
    const bit8_uint8_buffer_t tx = { get_command(device) };
    const uint32_t delay_ticks   = get_measurement_tick_duration(device);

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "unable to write to device handle, get measurement failed");
	
	/* delay task - allow time for the sensor to process measurement request */
    if(delay_ticks) vTaskDelay(delay_ticks);

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt read transaction */
        ret = hal_master_read(device->hal_handle, rx, BIT48_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(SHT4X_RETRY_DELAY_MS));
    } while (++rx_retry_count <= rx_retry_max && ret != ESP_OK );

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to device handle, get measurement failed" );
	
    /* validate crc values */
    if (rx[2] != calculate_crc8(rx, 2) || rx[5] != calculate_crc8(rx + 3, 2)) {
        return ESP_ERR_INVALID_CRC;
    }

	// convert sht4x results to engineering units of measure (C and %)
    *temperature = ((uint16_t)rx[0] << 8 | (uint16_t)rx[1]);
    *humidity    = ((uint16_t)rx[3] << 8 | (uint16_t)rx[4]);

    return ESP_OK;
}

esp_err_t sht4x_init(const void* hal_master_handle, const sht4x_config_t *sht4x_config, sht4x_handle_t *const sht4x_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hal_master_handle && (sht4x_config || sht4x_handle) );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_POWERUP_DELAY_MS));

    /* validate memory availability for handle */
    esp_err_t ret;
    sht4x_device_t* device = (sht4x_device_t*)calloc(1, sizeof(sht4x_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for device, device handle initialization failed");

    /* copy configuration */
    device->config = *sht4x_config;

    /* validate device exists on the hal master communication bus */
    ret = hal_master_probe(hal_master_handle, device);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal device does not exist, device handle initialization failed");

    /* initialize device onto the hal master communication bus */
    ret = hal_master_init(hal_master_handle, device);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal initialization failed, device handle initialization failed");

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_CMD_DELAY_MS));

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(hal_set_reset_register(device), err_handle, TAG, "unable to soft-reset device register, device handle initialization failed");

    /* sht4x attempt to read device serial number */
    ESP_GOTO_ON_ERROR(hal_get_serial_number_register(device, &device->serial_number), err_handle, TAG, "unable to read serial number from device register, device reset failed");

    /* set device handle */
    *sht4x_handle = (sht4x_handle_t)device;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(SHT4X_APPSTART_DELAY_MS));

    return ESP_OK;

    /* something went wrong - after device descriptor instantiation */
    err_handle:
        /* remove device from hal master communication bus */
        ret = hal_master_remove(device->hal_handle);

        /* free device handle */
        free(device);
    err:
        return ret;
}

esp_err_t sht4x_get_measurement(sht4x_handle_t handle, float *const temperature, float *const humidity) {
    uint16_t temp_signal = 0;
    uint16_t hum_signal  = 0;
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_get_adc_signals(device, &temp_signal, &hum_signal), TAG, "unable to read measurement, get measurement failed" );

    /* convert adc signals to engineering units of measure */
    *temperature = convert_adc_signal_to_temperature(temp_signal);
    *humidity    = convert_adc_signal_to_humidity(hum_signal);

    return ESP_OK;
}

esp_err_t sht4x_get_measurements(sht4x_handle_t handle, float *const drybulb, float *const humidity, float *const dewpoint, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && drybulb && humidity && dewpoint && wetbulb );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( sht4x_get_measurement(handle, drybulb, humidity), TAG, "unable to read measurement, read measurements failed" );

    /* get calculated dew-point and wet-bulb */
    ESP_RETURN_ON_ERROR( get_dewpoint(*drybulb, *humidity, dewpoint), TAG, "unable to calculate dew-point, read measurements failed" );
    ESP_RETURN_ON_ERROR( get_wetbulb(*drybulb, *humidity, wetbulb), TAG, "unable to calculate wet-bulb, read measurements failed" );

    return ESP_OK;
}

esp_err_t sht4x_get_measurement_record(sht4x_handle_t handle, sht4x_data_record_t *const data_record) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && data_record );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( sht4x_get_measurements(handle, &data_record->drybulb, 
                                                        &data_record->humidity, 
                                                        &data_record->dewpoint, 
                                                        &data_record->wetbulb), 
    TAG, "unable to read measurement record, read measurement record failed" );

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
    ESP_RETURN_ON_ERROR( hal_set_reset_register(device), TAG, "unable to write to device reset register, device reset failed" );

    /* sht4x attempt to read device serial number */
    ESP_RETURN_ON_ERROR( hal_get_serial_number_register(device, &device->serial_number), TAG, "unable to read serial number from device register, device reset failed" );

    return ESP_OK;
}

esp_err_t sht4x_remove(sht4x_handle_t handle) {
    sht4x_device_t* device = (sht4x_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from hal bus */
    return hal_master_remove(device->hal_handle);
}

esp_err_t sht4x_delete(sht4x_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from hal bus */
    esp_err_t ret = sht4x_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}

const char* sht4x_get_fw_version(void) {
    return (const char*)SHT4X_FW_VERSION_STR;
}

int32_t sht4x_get_fw_version_number(void) {
    return (int32_t)SHT4X_FW_VERSION_INT32;
}