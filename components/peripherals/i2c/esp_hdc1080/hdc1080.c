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

#define HDC1080_TEMPERATURE_MAX         (float)(125.0)  //!< hdc1080 maximum temperature range
#define HDC1080_TEMPERATURE_MIN         (float)(-40.0)  //!< hdc1080 minimum temperature range
#define HDC1080_HUMIDITY_MAX            (float)(100.0)  //!< hdc1080 maximum humidity range
#define HDC1080_HUMIDITY_MIN            (float)(0.0)    //!< hdc1080 minimum humidity range

#define HDC1080_POWERUP_DELAY_MS        UINT16_C(30)    //!< hdc1080 I2C power-up delay in milliseconds
#define HDC1080_APPSTART_DELAY_MS       UINT16_C(15)    //!< hdc1080 I2C application start delay in milliseconds
#define HDC1080_RESET_DELAY_MS          UINT16_C(20)
#define HDC1080_CMD_DELAY_MS            UINT16_C(5)
#define HDC1080_RETRY_DELAY_MS          UINT16_C(2)
#define HDC1080_TX_RX_DELAY_MS          UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

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
    i2c_master_dev_handle_t             i2c_handle;             /*!< hdc1080 i2c device handle */
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
 * @brief HDC1080 I2C HAL write command to register address transaction.
 * 
 * @param device HDC1080 device descriptor.
 * @param reg_addr HDC1080 command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_write_command(hdc1080_device_t *const device, const uint8_t reg_addr) {
    const bit8_uint8_buffer_t tx = { reg_addr }; // lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL write word to register address transaction.
 * 
 * @param device HDC1080 device descriptor.
 * @param reg_addr HDC1080 register address to write to.
 * @param word HDC1080 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_write_word_to(hdc1080_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL read word from register address transaction.
 * 
 * @param device HDC1080 device descriptor.
 * @param reg_addr HDC1080 register address to read from.
 * @param word HDC1080 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_read_word_from(hdc1080_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    //ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read from failed" );

    /* delay task before next i2c transaction */
    //vTaskDelay(pdMS_TO_TICKS(HDC1080_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    //ESP_RETURN_ON_ERROR( i2c_master_receive(handle->i2c_handle, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read from failed" );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "hdc1080_i2c_read_word_from failed" );


    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL read transaction.
 * 
 * @param device HDC1080 device descriptor.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_read(hdc1080_device_t *const device, uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read failed" );

    return ESP_OK;
}


/**
 * @brief Gets HDC1080 millisecond duration from humidity measurement resolution.  See datasheet for details.
 *
 * @param[in] resolution HDC1080 humidity measurement resolution.
 * @return uint8_t Measurement duration in milliseconds.
 */
static inline uint8_t hdc1080_get_humidity_duration(const hdc1080_humidity_resolutions_t resolution) {
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
static inline uint16_t hdc1080_get_humidity_tick_duration(const hdc1080_humidity_resolutions_t resolution) {
    uint16_t res = pdMS_TO_TICKS(hdc1080_get_humidity_duration(resolution));
    return res == 0 ? 1 : res;
}

/**
 * @brief Gets HDC1080 millisecond duration from temperature measurement resolution.  See datasheet for details.
 *
 * @param[in] resolution HDC1080 temperature measurement resolution.
 * @return uint8_t Measurement duration in milliseconds.
 */
static inline uint8_t hdc1080_get_temperature_duration(const hdc1080_temperature_resolutions_t resolution) {
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
static inline uint16_t hdc1080_get_temperature_tick_duration(const hdc1080_temperature_resolutions_t resolution) {
    uint16_t res = pdMS_TO_TICKS(hdc1080_get_temperature_duration(resolution));
    return res == 0 ? 1 : res;
}

/**
 * @brief Converts temperature signal to engineering units of measure.
 * 
 * @param temperature Temperature signal to convert.
 * @return float Converted temperature in degrees Celsius.
 */
static inline float hdc1080_convert_temperature_signal(const uint16_t temperature) {
    return (float)temperature / 65536.0f * 165.0f - 40.0f;
}

/**
 * @brief Converts humidity signal to engineering units of measure.
 * 
 * @param humidity Humidity signal to convert.
 * @return float Converted humidity in percent.
 */
static inline float hdc1080_convert_humidity_signal(const uint16_t humidity) {
    return (float)humidity / 65536.0f * 100.0f;
}

/**
 * @brief Calculates dew-point temperature from air temperature and relative humidity.
 *
 * @param[in] temperature Air temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dew-point temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_calculate_dewpoint(const float temperature, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( dewpoint );

    /* validate temperature argument */
    if(temperature > HDC1080_TEMPERATURE_MAX || temperature < HDC1080_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate dew-point failed");
    }

    /* validate humidity argument */
    if(humidity > HDC1080_HUMIDITY_MAX || humidity < HDC1080_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate dew-point failed");
    }
    
    // calculate dew-point temperature
    const double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    *dewpoint = 243.12*H/(17.62-H);
    
    return ESP_OK;
}

/**
 * @brief Calculates wet-bulb temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humidity in percent.
 * @param[out] wetbulb calculated wet-bulb temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_calculate_wetbulb(const float temperature, const float humidity, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK(wetbulb);

    // validate range of temperature parameter
    if(temperature > HDC1080_TEMPERATURE_MAX || temperature < HDC1080_TEMPERATURE_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate wet-bulb failed");
    }

    // validate range of humidity parameter
    if(humidity > HDC1080_HUMIDITY_MAX || humidity < HDC1080_HUMIDITY_MIN) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate wet-bulb failed");
    }
    
    // calculate wet-bulb temperature
    *wetbulb = temperature * atanf( 0.151977f * powf( (humidity + 8.313659f), 1.0f/2.0f ) ) + atanf(temperature + humidity) - atanf(humidity - 1.676331f) + 0.00391838f * powf(humidity, 3.0f/2.0f) * atanf(0.023101f * humidity) - 4.686035f;
    
    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL reads unique serial number register.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 serial number register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_get_serial_number_register(hdc1080_device_t *const device, uint64_t *const reg) {
    //uint16_t serial_id_fbp, serial_id_mbp, serial_id_lbp; // serial identifier parts
    //i2c_hdc1080_serial_number_register_t sn_reg; // this structure may not be needed
    // hdc1080_handle->dev_params->serial_number

    /* validate arguments */
    ESP_ARG_CHECK( device );

    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief HDC1080 I2C HAL read manufacturer identifier register.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 manufacturer identifier register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_get_manufacturer_id_register(hdc1080_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read manufacturer identifier */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_read_word_from(device, HDC1080_REG_MANUFACTURER_ID, reg), TAG, "read manufacturer identifier failed" );

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL reads device identifier register.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 device identifier register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_get_device_id_register(hdc1080_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device identifier */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_read_word_from(device, HDC1080_REG_DEVICE_ID, reg), TAG, "read device identifier failed" );

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL read configuration register.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_get_config_register(hdc1080_device_t *const device, hdc1080_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    uint16_t config;
    ESP_RETURN_ON_ERROR( hdc1080_i2c_read_word_from(device, HDC1080_REG_CONFIGURATION, &config), TAG, "read configuration register failed" );

    /* set output parameter */
    reg->reg = config;

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL write configuration register.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @param[in] reg HDC1080 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_set_config_register(hdc1080_device_t *const device, const hdc1080_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    hdc1080_config_register_t config = { .reg = reg.reg };

    /* set configuration reserved fields to 0 */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_write_word_to(device, HDC1080_REG_CONFIGURATION, config.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL setup and configuration.
 * 
 * @param[in] device HDC1080 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_setup(hdc1080_device_t *const device) {
    hdc1080_config_register_t config_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, setup failed");

    /* configure device */
    config_reg.bits.acquisition_mode       = HDC1080_ACQUISITION_SEQUENCED;
    config_reg.bits.temperature_resolution = device->config.temperature_resolution;
    config_reg.bits.humidity_resolution    = device->config.humidity_resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_set_config_register(device, config_reg), TAG, "unable to write configuration register, setup failed");

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL write reset to configuration register to reset device with restart delay
 * 
 * @param device HDC1080 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_set_reset_register(hdc1080_device_t *const device) {
    hdc1080_config_register_t config_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, write reset register failed");

    /* set soft-reset bit */
    config_reg.bits.reset_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_set_config_register(device, config_reg), TAG, "unable to write configuration register, write reset register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay( pdMS_TO_TICKS(HDC1080_RESET_DELAY_MS) );

    return ESP_OK;
}

/**
 * @brief HDC1080 I2C HAL read raw adc temperature and humidity signals.
 * 
 * @param device HDC1080 device descriptor.
 * @param temperature Raw adc temperature signal.
 * @param humidity Raw adc humidity signal.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hdc1080_i2c_get_adc_signals(hdc1080_device_t *const device, uint16_t *const temperature, uint16_t *const humidity) {
    const uint8_t rx_retry_max   = 5;
    esp_err_t     ret            = ESP_OK;
    uint8_t       rx_retry_count = 0;
    bit16_uint8_buffer_t rx      = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_write_command(device, HDC1080_REG_TEMPERATURE), TAG, "unable to write to i2c device handle, write to trigger temperature measurement failed");

    /* delay before next i2c transaction */
    vTaskDelay(hdc1080_get_temperature_tick_duration(device->config.temperature_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = hdc1080_i2c_read(device, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, read temperature failed" );

    /* concat temperature bytes */
    *temperature = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_write_command(device, HDC1080_REG_HUMIDITY), TAG, "unable to write to i2c device handle, write to trigger humidity measurement failed");

    /* delay before next i2c transaction */
    vTaskDelay(hdc1080_get_humidity_tick_duration(device->config.humidity_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    rx_retry_count = 0;
    do {
        /* attempt i2c read transaction */
        ret = hdc1080_i2c_read(device, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, read humidity failed" );

    /* concat humidity bytes */
    *humidity = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    return ESP_OK;
}


esp_err_t hdc1080_init(i2c_master_bus_handle_t master_handle, const hdc1080_config_t *hdc1080_config, hdc1080_handle_t *hdc1080_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && hdc1080_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, hdc1080_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, hdc1080 device handle initialization failed", hdc1080_config->i2c_address);

    /* validate memory availability for handle */
    hdc1080_device_t* device = (hdc1080_device_t*)calloc(1, sizeof(hdc1080_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hdc1080 device");

    /* copy configuration */
    device->config = *hdc1080_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(hdc1080_i2c_set_reset_register(device), err_handle, TAG, "i2c hdc1080 soft-reset device failed");

    /* attempt to setup the device */
    ESP_GOTO_ON_ERROR(hdc1080_i2c_setup(device), err_handle, TAG, "i2c hdc1080 setup device failed");

    /* attempt to read manufacturer identifier */
    ESP_GOTO_ON_ERROR(hdc1080_i2c_get_manufacturer_id_register(device, &device->manufacturer_id), err_handle, TAG, "i2c hdc1080 read manufacturer identifier failed");

    /* attempt to read device identifier */
    ESP_GOTO_ON_ERROR(hdc1080_i2c_get_device_id_register(device, &device->id), err_handle, TAG, "i2c hdc1080 read device identifier failed");

    /* attempt to read device serial number */

    /* set device handle */
    *hdc1080_handle = (hdc1080_handle_t)device;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(HDC1080_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t hdc1080_get_measurement__(hdc1080_handle_t handle, float *const temperature, float *const humidity) {
    const uint8_t rx_retry_max   = 5;
    esp_err_t     ret            = ESP_OK;
    uint8_t       rx_retry_count = 0;
    bit16_uint8_buffer_t rx      = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_write_command(device, HDC1080_REG_TEMPERATURE), TAG, "unable to write to i2c device handle, write to trigger temperature measurement failed");

    /* delay before next i2c transaction */
    vTaskDelay(hdc1080_get_temperature_tick_duration(device->config.temperature_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = hdc1080_i2c_read(device, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, read temperature failed" );

    /* concat temperature bytes */
    const uint16_t t_raw = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    /* convert temperature and set output parameter */
    *temperature = hdc1080_convert_temperature_signal(t_raw);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_write_command(device, HDC1080_REG_HUMIDITY), TAG, "unable to write to i2c device handle, write to trigger humidity measurement failed");

    /* delay before next i2c transaction */
    vTaskDelay(hdc1080_get_humidity_tick_duration(device->config.humidity_resolution));

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    rx_retry_count = 0;
    do {
        /* attempt i2c read transaction */
        ret = hdc1080_i2c_read(device, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(HDC1080_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, read humidity failed" );

    /* concat humidity bytes */
    const uint16_t h_raw = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    /* convert humidity and set output parameter */
    *humidity = hdc1080_convert_humidity_signal(h_raw);

    return ESP_OK;
}

esp_err_t hdc1080_get_measurement(hdc1080_handle_t handle, float *const temperature, float *const humidity) {
    uint16_t     temp_signal = 0;
    uint16_t      hum_signal = 0;
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_get_adc_signals(device, &temp_signal, &hum_signal), TAG, "unable to read to i2c device handle, read measurements failed" );

    /* convert temperature and set output parameter */
    *temperature = hdc1080_convert_temperature_signal(temp_signal);

    /* convert humidity and set output parameter */
    *humidity = hdc1080_convert_humidity_signal(hum_signal);

    return ESP_OK;
}

esp_err_t hdc1080_get_measurements(hdc1080_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && temperature && humidity && dewpoint && wetbulb );

    /* attempt to read measurements */
    ESP_RETURN_ON_ERROR( hdc1080_get_measurement(handle, temperature, humidity), TAG, "unable to read to i2c device handle, read measurements failed" );

    /* calculate dew-point */
    ESP_RETURN_ON_ERROR( hdc1080_calculate_dewpoint(*temperature, *humidity, dewpoint), TAG, "unable to calculate dew-point, read measurements failed");

    /* calculate wet-bulb */
    ESP_RETURN_ON_ERROR( hdc1080_calculate_wetbulb(*temperature, *humidity, wetbulb), TAG, "unable to calculate wet-bulb, read measurements failed");

    return ESP_OK;
}

esp_err_t hdc1080_enable_heater(hdc1080_handle_t handle) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, enable heater failed");

    /* set heater state */
    config_reg.bits.heater_enabled = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_set_config_register(device, config_reg), TAG, "unable to write configuration register, enable heater failed" );

    return ESP_OK;
}

esp_err_t hdc1080_disable_heater(hdc1080_handle_t handle) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, disable heater failed");

    /* set heater state */
    config_reg.bits.heater_enabled = false;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_set_config_register(device, config_reg), TAG, "unable to write configuration register, disable heater failed" );

    return ESP_OK;
}

esp_err_t hdc1080_get_temperature_resolution(hdc1080_handle_t handle, hdc1080_temperature_resolutions_t *const resolution) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, get temperature resolution failed");

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
    ESP_RETURN_ON_ERROR( hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, set temperature resolution failed");

    /* set temperature resolution */
    config_reg.bits.temperature_resolution = resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_set_config_register(device, config_reg), TAG, "unable to write configuration register, set temperature resolution failed" );

    return ESP_OK;
}

esp_err_t hdc1080_get_humidity_resolution(hdc1080_handle_t handle, hdc1080_humidity_resolutions_t *const resolution) {
    hdc1080_config_register_t config_reg = { 0 };
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, get humidity resolution failed");

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
    ESP_RETURN_ON_ERROR( hdc1080_i2c_get_config_register(device, &config_reg), TAG, "unable to read configuration register, set humidity resolution failed");

    /* set humidity resolution */
    config_reg.bits.humidity_resolution = resolution;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_set_config_register(device, config_reg), TAG, "unable to write configuration register, set humidity resolution failed" );

    return ESP_OK;
}

esp_err_t hdc1080_reset(hdc1080_handle_t handle) {
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR(hdc1080_i2c_set_reset_register(device), TAG, "unable to write reset register, reset failed");

    /* attempt to setup device */
    ESP_RETURN_ON_ERROR( hdc1080_i2c_setup(device), TAG, "unable to setup device, reset failed" );

    return ESP_OK;
}

esp_err_t hdc1080_remove(hdc1080_handle_t handle) {
    hdc1080_device_t* device = (hdc1080_device_t*)handle;

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