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
    hal_master_dev_handle_t             hal_handle;             /*!< hdc1080 HAL device communication handle */
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
static inline bool is_drybulb_temperature_in_range(const float drybulb) {
    return is_value_in_range(drybulb, HDC1080_DRYBULB_MIN, HDC1080_DRYBULB_MAX);
}

/**
 * @brief Helper: check if relative humidity is in range.
 * 
 * @param humidity relative humidity in percent.
 * @return true if relative humidity is in range, false otherwise.
 */
static inline bool is_relative_humidity_in_range(const float humidity) {
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
static inline esp_err_t get_dewpoint_temperature(const float drybulb, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( dewpoint );

    /* validate temperature argument */
    ESP_RETURN_ON_FALSE( is_drybulb_temperature_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated dew-point temperature failed");

    /* validate humidity argument */
    ESP_RETURN_ON_FALSE( is_relative_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated dew-point temperature failed");
    
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
static inline esp_err_t get_wetbulb_temperature(const float drybulb, const float humidity, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK(wetbulb);

    // validate range of temperature parameter
    ESP_RETURN_ON_FALSE( is_drybulb_temperature_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated wet-bulb temperature failed");

    // validate range of humidity parameter
    ESP_RETURN_ON_FALSE( is_relative_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated wet-bulb temperature failed");
    
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
    ESP_RETURN_ON_ERROR(hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, setup failed");

    /* configure device */
    config_reg.bits.acquisition_mode       = HDC1080_ACQUISITION_SEQUENCED;
    config_reg.bits.temperature_resolution = device->config.temperature_resolution;
    config_reg.bits.humidity_resolution    = device->config.humidity_resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR(hal_set_config_register(device, config_reg), TAG, "unable to write configuration register, setup failed");

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
    ESP_RETURN_ON_ERROR(hal_get_config_register(device, &config_reg), TAG, "unable to read configuration register, write reset register failed");

    /* set soft-reset bit */
    config_reg.bits.reset_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "unable to write configuration register, write reset register failed" );

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
    vTaskDelay(pdMS_TO_TICKS(get_temperature_duration(device->config.temperature_resolution) + 20)); // add 20 ms margin

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt read transaction */
        ret = hal_master_read(device->hal_handle, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to device handle, read temperature failed" );

    /* concat temperature bytes(big-endian) */
    *temperature = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_CMD_DELAY_MS));

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write_command(device->hal_handle, HDC1080_REG_HUMIDITY), TAG, "unable to write to device handle, write to trigger humidity measurement failed");

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(get_humidity_duration(device->config.humidity_resolution) + 20)); // add 20 ms margin

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

    /* concat humidity bytes (big-endian) */
    *humidity = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    return ESP_OK;
}


esp_err_t hdc1080_init(const void* master_handle, const hdc1080_config_t *hdc1080_config, hdc1080_handle_t *hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && hdc1080_config );

    /* instantiate and validate memory availability for handle */
    esp_err_t ret = ESP_OK;
    hdc1080_device_t* device = (hdc1080_device_t*)calloc(1, sizeof(hdc1080_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hdc1080 device");

    /* copy configuration */
    device->config = *hdc1080_config;

    /* validate hal master bus interface */
    ESP_GOTO_ON_FALSE( hdc1080_config->hal_bif == HAL_MASTER_BIF_I2C, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid HAL master bus interface, device handle initialization failed");

    /* cast i2c_master_bus_handle_t to void pointer */
    i2c_master_bus_handle_t i2c_master_handle = (i2c_master_bus_handle_t)master_handle;

    /* validate i2c master bus handle */
    ESP_GOTO_ON_FALSE( i2c_master_handle, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid master bus handle, device handle initialization failed");

    /* cast hal_config to i2c_device_config_t pointer */
    const i2c_device_config_t* i2c_dev_conf_ptr = (const i2c_device_config_t*)device->config.hal_config;

    /* validate i2c device configuration */
    ESP_GOTO_ON_FALSE(i2c_dev_conf_ptr, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid i2c device configuration, device handle initialization failed");
    
    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_dev_conf_ptr->device_address,
        .scl_speed_hz    = i2c_dev_conf_ptr->scl_speed_hz
    };

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_POWERUP_DELAY_MS));

    /* create hal master device */
    ret = hal_master_new_i2c_device(i2c_master_handle, &i2c_dev_conf, &device->hal_handle);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal_master_new_i2c_device failed, device handle initialization failed");

    /* validate device exists on the hal master communication bus */
    ret = hal_master_probe(device->hal_handle, i2c_dev_conf_ptr->device_address);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "device does not exist on the hal master communication bus, device handle initialization failed");

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_CMD_DELAY_MS));

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
        if (device->hal_handle) {
            hal_master_delete(device->hal_handle);
        }

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

esp_err_t hdc1080_get_measurements(hdc1080_handle_t handle, float *const drybulb, float *const humidity, float *const dewpoint, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && drybulb && humidity && dewpoint && wetbulb );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( hdc1080_get_measurement(handle, drybulb, humidity), TAG, "unable to read to device handle, read measurements failed" );

    /* calculate dew-point */
    ESP_RETURN_ON_ERROR( get_dewpoint_temperature(*drybulb, *humidity, dewpoint), TAG, "unable to get calculated dew-point temperature, read measurements failed");

    /* calculate wet-bulb */
    ESP_RETURN_ON_ERROR( get_wetbulb_temperature(*drybulb, *humidity, wetbulb), TAG, "unable to get calculated wet-bulb temperature, read measurements failed");

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
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* delete device handle */
    if (device->hal_handle) {
        hal_master_delete(device->hal_handle);
    }

    free(device);
    return ESP_OK;
}

const char* hdc1080_get_fw_version(void) {
    return (const char*)HDC1080_FW_VERSION_STR;
}

int32_t hdc1080_get_fw_version_number(void) {
    return (int32_t)HDC1080_FW_VERSION_INT32;
}