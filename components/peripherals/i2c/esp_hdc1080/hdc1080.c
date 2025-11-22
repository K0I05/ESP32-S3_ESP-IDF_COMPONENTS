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
 * @file hdc1080.c
 *
 * ESP-IDF driver for HDC1080 temperature and humdity sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/hdc1080.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * HDC1080 definitions
*/

#define HDC1080_REG_TEMPERATURE         UINT8_C(0x00)   //!< hdc1080 I2C temperature measurement output 
#define HDC1080_REG_HUMIDITY            UINT8_C(0x01)   //!< hdc1080 I2C relative humidity measurement ouput  
#define HDC1080_REG_CONFIGURATION       UINT8_C(0x02)   //!< hdc1080 I2C configuration and status
#define HDC1080_REG_SERIAL_ID_FBP       UINT8_C(0xFB)   //!< hdc1080 I2C first 2 bytes of the serial ID of the part
#define HDC1080_REG_SERIAL_ID_MBP       UINT8_C(0xFC)   //!< hdc1080 I2C mid 2 bytes of the serial ID of the part
#define HDC1080_REG_SERIAL_ID_LBP       UINT8_C(0xFD)   //!< hdc1080 I2C last byte bit of the serial ID of the part 
#define HDC1080_REG_MANUFACTURER_ID     UINT8_C(0xFE)   //!< hdc1080 I2C ID of Texas Instruments
#define HDC1080_REG_DEVICE_ID           UINT8_C(0xFF)   //!< hdc1080 I2C ID of the device

#define HDC1080_MANUFACTURER_ID         UINT16_C(0x5449)
#define HDC1080_DEVICE_ID               UINT16_C(0x1050)

#define HDC1080_DRYBULB_MAX             (float)(125.0)  //!< hdc1080 maximum dry-bulb temperature range
#define HDC1080_DRYBULB_MIN             (float)(-40.0)  //!< hdc1080 minimum dry-bulb temperature range
#define HDC1080_HUMIDITY_MAX            (float)(100.0)  //!< hdc1080 maximum relative humidity range
#define HDC1080_HUMIDITY_MIN            (float)(0.0)    //!< hdc1080 minimum relative humidity range

#define HDC1080_POWERUP_DELAY_MS        UINT16_C(30)    //!< hdc1080 I2C power-up delay in milliseconds
#define HDC1080_APPSTART_DELAY_MS       UINT16_C(15)    //!< hdc1080 I2C application start delay in milliseconds
#define HDC1080_RESET_DELAY_MS          UINT16_C(20)
#define HDC1080_CMD_DELAY_MS            UINT16_C(5)
#define HDC1080_RETRY_DELAY_MS          UINT16_C(2)
#define HDC1080_TX_RX_DELAY_MS          UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS              UINT16_C(500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief HDC1080 device configuration register structure definition.
 */
typedef union __attribute__((packed)) hdc1080_config_register_u {
    struct REG_CFG_BITS_TAG {
        uint8_t                           reserved1:8;               /*!< reserved and set to 0              (bit:0-7) */
        hdc1080_humidity_resolutions_t    humidity_resolution:2;     /*!< humidity measurement resolution    (bit:8-9) */
        hdc1080_temperature_resolutions_t temperature_resolution:1;  /*!< temperature measurement resolution (bit:10) */
        hdc1080_battery_states_t          battery_state:1;           /*!< battery status                     (bit:11) */
        hdc1080_acquisition_modes_t       acquisition_mode:1;        /*!< acquisition mode                   (bit:12) */
        bool                              heater_enabled:1;          /*!< heater enabled when true           (bit:13) */
        uint8_t                           reserved2:1;               /*!< reserved and set to 0              (bit:14) */
        bool                              reset_enabled:1;           /*!< software reset when true           (bit:15) */
    } bits;          /*!< represents the 16-bit configuration register parts in bits. */
    uint16_t reg;   /*!< represents the 16-bit configuration register as `uint16_t` */
} hdc1080_config_register_t;

/**
 * @brief HDC1080 temperature or humidity measurement register structure definition.
 */
typedef union __attribute__((packed)) hdc1080_measurement_register_u {
    struct REG_MEAS_BITS_TAG {
        uint8_t        reserved:2;  /*!< reserved and set to 0        (bit:0-1) */
        uint16_t       value:14;    /*!< measurement value            (bit:2-14) */
    } bits;         /*!< represents the 16-bit measurement register parts in bits. */
    uint16_t reg;   /*!< represents the 16-bit measurement register as `uint16_t` */
} hdc1080_measurement_register_t;

/**
 * @brief HDC1080 serial number register structure definition.
 */
typedef union __attribute__((packed)) hdc1080_serial_number_register_u {
    struct REG_SN_BITS_TAG {
        uint8_t        reserved:7;      /*!< reserved and set to 0        (bit:0-6)   */
        uint16_t       serial_id_0:9;   /*!< serial id 0                  (bit:7-15)  */
        uint16_t       serial_id_1:16;  /*!< serial id 1                  (bit:16-31) */
        uint16_t       serial_id_2:16;  /*!< serial id 2                  (bit:32-47) */
    } bits;          /*!< represents the 64-bit measurement register parts in bits. */
    uint64_t reg;   /*!< represents the 64-bit measurement register as `uint64_t` */
} hdc1080_serial_number_register_t;

/**
 * @brief HDC1080 device descriptor structure definition.
 */
typedef struct hdc1080_device_s {
    hdc1080_config_t                    config;                 /*!< hdc1080 device configuration */ 
    void*                               hal_handle;             /*!< hdc1080 HAL device handle */
    uint64_t                            serial_number;          /*!< hdc1080 device serial number */
    uint16_t                            manufacturer_id;        /*!< hdc1080 device manufacturer identifier */
    uint16_t                            id;                     /*!< hdc1080 device device identifier */
} hdc1080_device_t;

/*
* static constant declarations
*/
static const char *TAG = "hdc1080";

/*
* functions and subroutines
*/



/**
 * @brief HAL device probe on master communication bus.
 * 
 * @param master_handle Pointer to HAL master communication bus handle.
 * @param device Pointer to HDC1080 device descriptor.
 * @return esp_err_t ESP_OK on success.  ESP_ERR_NOT_FOUND: probe failed, 
 * doesn't find the device with specific address you gave.  ESP_ERR_TIMEOUT: 
 * Operation timeout(larger than xfer_timeout_ms) because the bus is busy or 
 * hardware crash.
 */
static inline esp_err_t hal_master_probe(const void* master_handle, hdc1080_device_t *const device) {
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
 * @param device Pointer to HDC1080 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_init(const void* master_handle, hdc1080_device_t *const device) {
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
 * @brief HAL write command to register address transaction.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @param reg_addr HDC1080 command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_command(const void* device_handle, const uint8_t reg_addr) {
    const bit8_uint8_buffer_t tx = { reg_addr }; // lsb, msb

    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(hal_device, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, hal device write failed" );
                        
    return ESP_OK;
}

/**
 * @brief HAL write word to register address transaction.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @param reg_addr HDC1080 register address to write to.
 * @param word HDC1080 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_write_word_to(const void* device_handle, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) };

    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(hal_device, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, hal device write failed" );
                        
    return ESP_OK;
}

/**
 * @brief HAL read word from register address transaction.
 * 
 * @param device_handle Pointer to HAL communication device handle.
 * @param reg_addr HDC1080 register address to read from.
 * @param word HDC1080 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_master_read_word_from(const void* device_handle, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* cast to i2c master device handle */
    i2c_master_dev_handle_t hal_device = (i2c_master_dev_handle_t)device_handle;

    /* validate arguments */
    ESP_ARG_CHECK( hal_device );

    /* attempt i2c write transaction */
    //ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read from failed" );

    /* delay task before next i2c transaction */
    //vTaskDelay(pdMS_TO_TICKS(HDC1080_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    //ESP_RETURN_ON_ERROR( i2c_master_receive(handle->i2c_handle, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read from failed" );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(hal_device, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit_receive, hal device read word from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief HAL read transaction.
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
    ESP_RETURN_ON_ERROR( i2c_master_receive(hal_device, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, HAL device read failed" );

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
 * @brief Gets HDC1080 millisecond duration from humidity measurement resolution.  See datasheet for details.
 *
 * @param[in] resolution HDC1080 humidity measurement resolution.
 * @return uint8_t Measurement duration in milliseconds.
 */
static inline uint8_t get_humidity_duration(const hdc1080_humidity_resolutions_t resolution) {
    switch (resolution) {
        case HDC1080_HUMIDITY_RESOLUTION_14BIT:
            return 7;
        case HDC1080_HUMIDITY_RESOLUTION_11BIT:
            return 4;
        case HDC1080_HUMIDITY_RESOLUTION_8BIT:
            return 3;
        default:
            return 7;
    }
}

/**
 * @brief Gets HDC1080 tick duration from humidity measurement resolution.
 *
 * @param[in] resolution HDC1080 humidity measurement resolution.
 * @return uint16_t Measurement duration in ticks.
 */
static inline uint16_t get_humidity_tick_duration(const hdc1080_humidity_resolutions_t resolution) {
    uint16_t res = pdMS_TO_TICKS(get_humidity_duration(resolution));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets HDC1080 millisecond duration from temperature measurement resolution.  See datasheet for details.
 *
 * @param[in] resolution HDC1080 temperature measurement resolution.
 * @return uint8_t Measurement duration in milliseconds.
 */
static inline uint8_t get_temperature_duration(const hdc1080_temperature_resolutions_t resolution) {
    switch (resolution) {
        case HDC1080_TEMPERATURE_RESOLUTION_14BIT:
            return 7;
        case HDC1080_TEMPERATURE_RESOLUTION_11BIT:
            return 4;
        default:
            return 7;
    }
}

/**
 * @brief Gets HDC1080 tick duration from temperature measurement resolution.
 *
 * @param[in] resolution HDC1080 temperature measurement resolution.
 * @return uint16_t Measurement duration in ticks.
 */
static inline uint16_t get_temperature_tick_duration(const hdc1080_temperature_resolutions_t resolution) {
    uint16_t res = pdMS_TO_TICKS(get_temperature_duration(resolution));
    return res == 0 ? 1 : res;
}

/**
 * @brief Converts temperature ADC signal to engineering units of measure.
 * 
 * @param adc_signal Temperature ADC signal to convert.
 * @return float Converted temperature in degrees Celsius.
 */
static inline float convert_adc_signal_to_temperature(const uint16_t adc_signal) {
    return (float)adc_signal / 65536.0f * 165.0f - 40.0f;
}

/**
 * @brief Converts relative humidity ADC signal to engineering units of measure.
 * 
 * @param adc_signal Relative humidity ADC signal to convert.
 * @return float Converted humidity in percent.
 */
static inline float convert_adc_signal_to_humidity(const uint16_t adc_signal) {
    return (float)adc_signal / 65536.0f * 100.0f;
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
    return is_value_in_range(drybulb, HDC1080_DRYBULB_MIN, HDC1080_DRYBULB_MAX);
}

/**
 * @brief Helper: check if relative humidity is in range.
 * 
 * @param humidity relative humidity in percent.
 * @return true if relative humidity is in range, false otherwise.
 */
static inline bool is_humidity_in_range(const float humidity) {
    return is_value_in_range(humidity, HDC1080_HUMIDITY_MIN, HDC1080_HUMIDITY_MAX);
}

/**
 * @brief Gets calculated dew-point temperature from dry-bulb temperature and relative humidity.
 *
 * @param[in] drybulb Dry-bulb temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dew-point temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t get_dewpoint(const float drybulb, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( dewpoint );

    /* validate temperature argument */
    ESP_RETURN_ON_FALSE( is_drybulb_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated dew-point failed");

    /* validate humidity argument */
    ESP_RETURN_ON_FALSE( is_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated dew-point failed");
    
    // calculate dew-point temperature
    const double H = (log10(humidity)-2)/0.4343 + (17.62*drybulb)/(243.12+drybulb);
    *dewpoint = 243.12*H/(17.62-H);
    
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

    // validate range of temperature parameter
    ESP_RETURN_ON_FALSE( is_drybulb_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated wet-bulb failed");

    // validate range of humidity parameter
    ESP_RETURN_ON_FALSE( is_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated wet-bulb failed");
    
    // calculate wet-bulb temperature
    *wetbulb = drybulb * atanf( 0.151977f * powf( (humidity + 8.313659f), 1.0f/2.0f ) ) + atanf(drybulb + humidity) - atanf(humidity - 1.676331f) + 0.00391838f * powf(humidity, 3.0f/2.0f) * atanf(0.023101f * humidity) - 4.686035f;
    
    return ESP_OK;
}

/**
 * @brief HAL reads unique serial number register.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 serial number register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_serial_number_register(hdc1080_device_t *const device, uint64_t *const reg) {
    //uint16_t serial_id_fbp, serial_id_mbp, serial_id_lbp; // serial identifier parts
    //i2c_hdc1080_serial_number_register_t sn_reg; // this structure may not be needed
    // hdc1080_handle->dev_params->serial_number

    /* validate arguments */
    ESP_ARG_CHECK( device );

    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief HAL read manufacturer identifier register.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @param[out] reg HDC1080 manufacturer identifier register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_manufacturer_id_register(hdc1080_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read manufacturer identifier */
    ESP_RETURN_ON_ERROR( hal_master_read_word_from(device->hal_handle, HDC1080_REG_MANUFACTURER_ID, reg), TAG, "read manufacturer identifier failed" );

    return ESP_OK;
}

/**
 * @brief HAL reads device identifier register.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @param[out] reg HDC1080 device identifier register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_device_id_register(hdc1080_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device identifier */
    ESP_RETURN_ON_ERROR( hal_master_read_word_from(device->hal_handle, HDC1080_REG_DEVICE_ID, reg), TAG, "read device identifier failed" );

    return ESP_OK;
}

/**
 * @brief HAL read configuration register.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @param[out] reg HDC1080 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_config_register(hdc1080_device_t *const device, hdc1080_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    uint16_t config;
    ESP_RETURN_ON_ERROR( hal_master_read_word_from(device->hal_handle, HDC1080_REG_CONFIGURATION, &config), TAG, "read configuration register failed" );

    /* set output parameter */
    reg->reg = config;

    return ESP_OK;
}

/**
 * @brief HAL write configuration register.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @param[in] reg HDC1080 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_set_config_register(hdc1080_device_t *const device, const hdc1080_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    hdc1080_config_register_t config = { .reg = reg.reg };

    /* set configuration reserved fields to 0 */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_master_write_word_to(device->hal_handle, HDC1080_REG_CONFIGURATION, config.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

/**
 * @brief HAL setup and configuration registers.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_setup_registers(hdc1080_device_t *const device) {
    hdc1080_config_register_t config_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hal_get_config_register(device->hal_handle, &config_reg), TAG, "unable to read configuration register, setup failed");

    /* configure device */
    config_reg.bits.acquisition_mode       = HDC1080_ACQUISITION_SEQUENCED;
    config_reg.bits.temperature_resolution = device->config.temperature_resolution;
    config_reg.bits.humidity_resolution    = device->config.humidity_resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR(hal_set_config_register(device->hal_handle, config_reg), TAG, "unable to write configuration register, setup failed");

    return ESP_OK;
}

/**
 * @brief HAL write reset to configuration register to reset device with restart delay
 * 
 * @param device HDC1080 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_set_reset_register(hdc1080_device_t *const device) {
    hdc1080_config_register_t config_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hal_get_config_register(device->hal_handle, &config_reg), TAG, "unable to read configuration register, write reset register failed");

    /* set soft-reset bit */
    config_reg.bits.reset_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device->hal_handle, config_reg), TAG, "unable to write configuration register, write reset register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(HDC1080_RESET_DELAY_MS) );

    return ESP_OK;
}

/**
 * @brief HAL read ADC temperature and humidity signals.
 * 
 * @param device HDC1080 device descriptor.
 * @param temperature Raw adc temperature signal.
 * @param humidity Raw adc humidity signal.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_adc_signals(hdc1080_device_t *const device, uint16_t *const temperature, uint16_t *const humidity) {
    const uint8_t rx_retry_max   = 5;
    esp_err_t     ret            = ESP_OK;
    uint8_t       rx_retry_count = 0;
    bit16_uint8_buffer_t rx      = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write_command(device->hal_handle, HDC1080_REG_TEMPERATURE), TAG, "unable to write to device handle, write to trigger temperature measurement failed");

    /* delay before next transaction */
    vTaskDelay(get_temperature_tick_duration(device->config.temperature_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt read transaction */
        ret = hal_master_read(device->hal_handle, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to device handle, read temperature failed" );

    /* concat temperature bytes */
    *temperature = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write_command(device->hal_handle, HDC1080_REG_HUMIDITY), TAG, "unable to write to device handle, write to trigger humidity measurement failed");

    /* delay before next transaction */
    vTaskDelay(get_humidity_tick_duration(device->config.humidity_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    rx_retry_count = 0;
    do {
        /* attempt read transaction */
        ret = hal_master_read(device->hal_handle, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to device handle, read humidity failed" );

    /* concat humidity bytes */
    *humidity = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    return ESP_OK;
}


esp_err_t hdc1080_init(const void* hal_master_handle, const hdc1080_config_t *hdc1080_config, hdc1080_handle_t *hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( hal_master_handle && hdc1080_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_POWERUP_DELAY_MS));

    /* validate memory availability for handle */
    esp_err_t ret;
    hdc1080_device_t* device = (hdc1080_device_t*)calloc(1, sizeof(hdc1080_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hdc1080 device");

    /* copy configuration */
    device->config = *hdc1080_config;

    /* validate device exists on the hal master communication bus */
    ret = hal_master_probe(hal_master_handle, device);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal device does not exist, device handle initialization failed");

    /* initialize device onto the hal master communication bus */
    ret = hal_master_init(hal_master_handle, device);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal initialization failed, device handle initialization failed");

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(hal_set_reset_register(device), err_handle, TAG, "hdc1080 soft-reset device failed");

    /* attempt to setup the device */
    ESP_GOTO_ON_ERROR(hal_setup_registers(device), err_handle, TAG, "hdc1080 setup device failed");

    /* attempt to read manufacturer identifier */
    ESP_GOTO_ON_ERROR(hal_get_manufacturer_id_register(device, &device->manufacturer_id), err_handle, TAG, "hdc1080 read manufacturer identifier failed");

    /* attempt to read device identifier */
    ESP_GOTO_ON_ERROR(hal_get_device_id_register(device, &device->id), err_handle, TAG, "hdc1080 read device identifier failed");

    /* attempt to read device serial number */

    /* set device handle */
    *hdc1080_handle = (hdc1080_handle_t)device;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_APPSTART_DELAY_MS));

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


esp_err_t hdc1080_get_measurement(hdc1080_handle_t handle, float *const temperature, float *const humidity) {
    uint16_t     temp_signal = 0;
    uint16_t      hum_signal = 0;
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hal_get_adc_signals(device, &temp_signal, &hum_signal), TAG, "unable to read to device handle, read measurements failed" );

    /* convert temperature and set output parameter */
    *temperature = convert_adc_signal_to_temperature(temp_signal);

    /* convert humidity and set output parameter */
    *humidity = convert_adc_signal_to_humidity(hum_signal);

    return ESP_OK;
}

esp_err_t hdc1080_get_measurements(hdc1080_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && temperature && humidity && dewpoint && wetbulb );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( hdc1080_get_measurement(handle, temperature, humidity), TAG, "unable to read to i2c device handle, read measurements failed" );

    /* calculate dew-point */
    ESP_RETURN_ON_ERROR( get_dewpoint(*temperature, *humidity, dewpoint), TAG, "unable to get calculated dew-point, read measurements failed");

    /* calculate wet-bulb */
    ESP_RETURN_ON_ERROR( get_wetbulb(*temperature, *humidity, wetbulb), TAG, "unable to get calculated wet-bulb, read measurements failed");

    return ESP_OK;
}

esp_err_t hdc1080_get_measurement_record(hdc1080_handle_t handle, hdc1080_data_record_t *const data_record) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && data_record );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( hdc1080_get_measurements(handle, &data_record->drybulb, 
                                                        &data_record->humidity, 
                                                        &data_record->dewpoint, 
                                                        &data_record->wetbulb), 
    TAG, "unable to read measurement record, read measurement record failed" );

    return ESP_OK;
}

esp_err_t hdc1080_enable_heater(hdc1080_handle_t handle) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, enable heater failed");

    /* set heater state */
    config_reg.bits.heater_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "unable to write configuration register, enable heater failed" );

    return ESP_OK;
}

esp_err_t hdc1080_disable_heater(hdc1080_handle_t handle) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, disable heater failed");

    /* set heater state */
    config_reg.bits.heater_enabled = false;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "unable to write configuration register, disable heater failed" );

    return ESP_OK;
}

esp_err_t hdc1080_get_temperature_resolution(hdc1080_handle_t handle, hdc1080_temperature_resolutions_t *const resolution) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, get temperature resolution failed");

    /* set output parameter */
    *resolution = config_reg.bits.temperature_resolution;

    return ESP_OK;
}

esp_err_t hdc1080_set_temperature_resolution(hdc1080_handle_t handle, const hdc1080_temperature_resolutions_t resolution) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, set temperature resolution failed");

    /* set temperature resolution */
    config_reg.bits.temperature_resolution = resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "unable to write configuration register, set temperature resolution failed" );

    return ESP_OK;
}

esp_err_t hdc1080_get_humidity_resolution(hdc1080_handle_t handle, hdc1080_humidity_resolutions_t *const resolution) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, get humidity resolution failed");

    /* set output parameter */
    *resolution = config_reg.bits.humidity_resolution;

    return ESP_OK;
}

esp_err_t hdc1080_set_humidity_resolution(hdc1080_handle_t handle, const hdc1080_humidity_resolutions_t resolution) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, set humidity resolution failed");

    /* set humidity resolution */
    config_reg.bits.humidity_resolution = resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "unable to write configuration register, set humidity resolution failed" );

    return ESP_OK;
}

esp_err_t hdc1080_reset(hdc1080_handle_t handle) {
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( hal_set_reset_register(device), TAG, "unable to write reset register, reset failed" );

    /* attempt to setup device */
    ESP_RETURN_ON_ERROR( hal_setup_registers(device), TAG, "unable to setup device, reset failed" );

    return ESP_OK;
}

esp_err_t hdc1080_remove(hdc1080_handle_t handle) {
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from hal bus */
    return hal_master_remove(device->hal_handle);
}

esp_err_t hdc1080_delete(hdc1080_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    esp_err_t ret = hdc1080_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}

const char* hdc1080_get_fw_version(void) {
    return (const char*)HDC1080_FW_VERSION_STR;
}

int32_t hdc1080_get_fw_version_number(void) {
    return (int32_t)HDC1080_FW_VERSION_INT32;
}