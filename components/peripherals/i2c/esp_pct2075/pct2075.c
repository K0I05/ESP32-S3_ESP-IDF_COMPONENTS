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
// https://github.com/RobTillaart/INA228/blob/master/INA228.cpp
/**
 * @file pct2075.c
 *
 * ESP-IDF driver for PCT2075 temperature sensor
 * 
 * 
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/pct2075.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define PCT2075_SAMPLING_PERIOD_MIN_MS      UINT16_C(100)       /*!< pct2075 minimum sampling period in milliseconds */
#define PCT2075_SAMPLING_PERIOD_MAX_MS      UINT16_C(3100)      /*!< pct2075 maximum sampling period in milliseconds */

#define PCT2075_TEMP_SET_POINT_MIN_C        (-55.0f)            /*!< pct2075 Tos and Thys minimum set point in degrees Celsius */
#define PCT2075_TEMP_SET_POINT_MAX_C        (125.0f)            /*!< pct2075 Tos and Thys maximum set point in degrees Celsius */

#define PCT2075_REG_CONFIG                  UINT8_C(0x01)       // POR State: 0x00
#define PCT2075_REG_TEMP                    UINT8_C(0x00)       // POR State: 0x0000
#define PCT2075_REG_OVER_TEMP_SHTDWN        UINT8_C(0x03)       // POR State: 0x5000 / 0x6E00 / 0xFB00
#define PCT2075_REG_TEMP_HYST               UINT8_C(0x02)       // POR State: 0x4B00 / 0x6900 / 0F600
#define PCT2075_REG_TEMP_IDLE               UINT8_C(0x04)       // POR State: 0x00

#define PCT2075_POWERUP_DELAY_MS            UINT16_C(25)
#define PCT2075_APPSTART_DELAY_MS           UINT16_C(25)
#define PCT2075_CMD_DELAY_MS                UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief PCT2075 configuration register structure definition.
 */
typedef union __attribute__((packed)) pct2075_config_register_u {
    struct {
        bool                            shutdown_enabled:1; /*!< pct2075 is shutdown/disabled when enabled (true)   (bit:0)  */
        pct2075_os_operation_modes_t    operation_mode:1;   /*!< pct2075 os operation mode  (bit:1) */
        pct2075_os_polarities_t         polarity:1;         /*!< pct2075 os polarity    (bit:2) */
        pct2075_os_fault_queues_t       fault_queue:2;      /*!< pct2075 os fault queue programming   (bit:3-4) */
        uint8_t                         reserved:3;         /*!< reserved        (bit:5-7) */
    } bits;
    uint8_t reg;
} pct2075_config_register_t;

/**
 * @brief PCT2075 temperature idle sampling period register structure definition.
 */
typedef union __attribute__((packed)) pct2075_sampling_period_register_u {
    struct {
        uint8_t                         tidle:5;        /*!< pct2075 sampling period (tidle * 100ms) (bit:0-4) */
        uint8_t                         reserved:3;     /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} pct2075_sampling_period_register_t;

/**
 * @brief PCT2075 device descriptor structure definition.
 */
typedef struct pct2075_device_s {
    pct2075_config_t        config;     /*!< pct2075 device configuration */
    i2c_master_dev_handle_t i2c_handle; /*!< pct2075 i2c device handle */
} pct2075_device_t;

/*
* static constant declarations
*/
static const char *TAG = "pct2075";

/**
 * @brief PCT2075 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device PCT2075 device descriptor.
 * @param reg_addr PCT2075 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_read_from(pct2075_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write/read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL read word (two's compliment) from register address transaction.
 * 
 * @param device PCT2075 device descriptor.
 * @param reg_addr PCT2075 register address to read from.
 * @param word PCT2075 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_read_word_from(pct2075_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write/read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_read_word_from failed" );

    /* set output parameter */
    *word = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write word (two's compliment) to register address transaction.
 * 
 * @param device PCT2075 device descriptor.
 * @param reg_addr PCT2075 register address to write to.
 * @param word PCT2075 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_write_word_to(pct2075_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)((word >> 8) & 0xff), (uint8_t)(word & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write byte to register address transaction.
 * 
 * @param device PCT2075 device descriptor.
 * @param reg_addr PCT2075 register address to write to.
 * @param byte PCT2075 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_write_byte_to(pct2075_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte }; // register, byte

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_write_byte_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write transaction.
 * 
 * @param device PCT2075 device descriptor.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_write(pct2075_device_t *const device, uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_write failed" );

    return ESP_OK;
}

/**
 * @brief Converts a 11-bit value, using two's compliment, to a signed 16-bit integer value.
 * 
 * @param buffer Buffer containing the register's value as byte array (2-bytes).
 * @return int16_t Signed 16-bit integer value representing the 11-bit two's complement value.
 */
static inline int16_t pct2075_convert_11bit_twos_int16(const bit16_uint8_buffer_t buffer) {
    // convert bytes to unsigned 16-bit integer using two's complement
    const uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    // shift right by 5, i.e. 11-bits are of interest
    int16_t value = (int16_t)(raw >> 5);
    // sign-extend from 11-bits if necessary
    if (value & 0x0400) {
        value |= 0xF800;
    }
    return value;
}

/**
 * @brief Converts a 9-bit value, using two's compliment, to a signed 16-bit integer value.
 * 
 * @param buffer Buffer containing the register's value as byte array (2-bytes).
 * @return int16_t Signed 16-bit integer value representing the 9-bit two's complement value.
 */
static inline int16_t pct2075_convert_9bit_twos_int16(const bit16_uint8_buffer_t buffer) {
    // convert bytes to unsigned 16-bit integer using two's complement
    const uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    // shift right by 7, i.e. 9-bits are of interest
    int16_t value = (int16_t)(raw >> 7);
    // sign-extend from 9-bits if necessary
    if (value & 0x0100) {
        value |= 0xFE00;
    }
    return value;
}

/**
 * @brief Converts a signed 16-bit integer value to a 9-bit two's complement value and stores it in a buffer.
 * 
 * @param value Signed 16-bit integer value to convert to 9-bit two's complement. 
 * @param buffer Buffer to store the 9-bit two's complement value as byte array (2-bytes).
 */
static inline void pct2075_convert_int16_twos_9bit(const int16_t value, bit16_uint8_buffer_t buffer) {
    // Convert int16_t value to 9-bit two's complement and store in buffer
    uint16_t temp = ((uint16_t)(value & 0x01FF)) << 7; // mask to 9 bits, then shift
    buffer[0] = (uint8_t)((temp >> 8) & 0xFF);
    buffer[1] = (uint8_t)(temp & 0xFF);
}

/**
 * @brief Converts sampling period in milliseconds to time steps at 100-ms increments.
 * 
 * @param period Sampling period in milliseconds to convert.
 * @return uint8_t Sampling period converted to time steps.
 */
static inline uint8_t pct2075_convert_sampling_period_to_steps(const uint16_t period) {
    if(period == 0) return (uint8_t)1; // default to 100-ms
    return (uint8_t)(uint16_t)(period / (uint16_t)100); // convert to steps (tidle)
}

/**
 * @brief Converts time steps at 100-ms increments to sampling period in milliseconds.
 * 
 * @param steps Time steps to convert.
 * @return uint16_t Time steps converted to sampling period.
 */
static inline uint16_t pct2075_convert_steps_to_sampling_period(const uint8_t steps) {
    if(steps == 0) return 100; // default to 100-ms
    return (uint16_t)(steps * 100); // convert to sampling period
}

/**
 * @brief Converts hysteresis or overtemperature int16 signal to temperature in degrees Celsius.
 * 
 * @param signal Hysteresis or overtemperature temperature int16 signal to convert.
 * @return float Converted Hysteresis or overtemperature temperature in degrees Celsius.
 */
static inline float pct2075_convert_int16_signal_to_temperature(const int16_t signal) {
    /* convert - 0.5 degrees Celsius resolution */
    return (float)signal * 0.5f;
}

/**
 * @brief Converts hysteresis or overtemperature 9-bit signal to temperature in degrees Celsius.
 * 
 * @param signal Hysteresis or overtemperature temperature signal byte array to convert.
 * @return float Converted Hysteresis or overtemperature temperature in degrees Celsius.
 */
static inline float pct2075_convert_9bit_signal_to_temperature(const bit16_uint8_buffer_t signal) {
    /* convert to signed 9-bit two's complement to temperature - 0.5 degrees Celsius resolution */
    return (float)pct2075_convert_9bit_twos_int16(signal) * 0.5f;
}

/**
 * @brief Converts hysteresis or overtemperature temperature in degrees Celsius to 9-bit signal temperature.
 * 
 * @param temperature Temperature to convert in degrees Celsius.
 * @return uint8_t* Converted 9-bit byte array temperature.
 */
static inline uint8_t* pct2075_convert_temperature_to_9bit_signal(const float temperature) {
    static bit16_uint8_buffer_t signal = { 0 };
    /* set temperature to signed decimal value - 0.5 degrees Celsius resolution */
    const int16_t ots_temperature = (int16_t)(temperature / 0.5f);
    /* convert signed decimal value to two's complement byte array */
    pct2075_convert_int16_twos_9bit(ots_temperature, signal);
    return signal;
}

/**
 * @brief Converts int16 temperature signal to temperature in degrees Celsius.
 * 
 * @param temperature Temperature to convert in degrees Celsius.
 * @return int16_t Converted temperature signal.
 */
static inline int16_t pct2075_convert_temperature_to_int16_signal(const float temperature) {
    /* set temperature to signed decimal value - 0.5 degrees Celsius resolution */
    return (int16_t)(temperature / 0.5f);
}

/**
 * @brief Converts temperature byte array signal to temperature in degrees Celsius.
 * 
 * @param signal Temperature byte array signal to convert.
 * @return float Converted temperature in degrees Celsius.
 */
static inline float pct2075_convert_11bit_signal_to_temperature(const bit16_uint8_buffer_t signal) {
    /* convert to signed 11-bit two's complement to temperature - 0.125 degrees Celsius resolution */
    return (float)pct2075_convert_11bit_twos_int16(signal) * 0.125f;
}

/**
 * @brief PCT2075 I2C HAL read configuration register from PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_get_config_register(pct2075_device_t *const device, pct2075_config_register_t *const reg) {
    uint16_t cfg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_word_from(device, PCT2075_REG_CONFIG, &cfg), TAG, "read configuration register failed" );

    /* convert to configuration register */
    reg->reg = cfg;
    
    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write configuration register to PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_set_config_register(pct2075_device_t *const device, const pct2075_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write_word_to(device, PCT2075_REG_CONFIG, reg.reg), TAG, "write configuration register failed" );
    
    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL read sampling period register from PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 sampling period register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_get_sampling_period_register(pct2075_device_t *const device, pct2075_sampling_period_register_t *const reg) {
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(device, PCT2075_REG_TEMP_IDLE, rx, BIT8_UINT8_BUFFER_SIZE), TAG, "read sampling period register failed" );

    /* convert to sampling period register */
    reg->reg = rx[0];

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write sampling period register to PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 sampling period register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_set_sampling_period_register(pct2075_device_t *const device, const pct2075_sampling_period_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write_byte_to(device, PCT2075_REG_TEMP_IDLE, reg.reg), TAG, "write sampling period register failed" );

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL read hysteresis temperature register from PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 hysteresis temperature register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_get_hys_temperature_register(pct2075_device_t *const device, int16_t *const reg) {
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(device, PCT2075_REG_TEMP_HYST, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read hysteresis temperature register failed" );

    /* convert to signed 9-bit two's complement */
    *reg = pct2075_convert_9bit_twos_int16(rx);

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL read overtemperature shutdown register from PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 overtemperature shutdown register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_get_ots_temperature_register(pct2075_device_t *const device, int16_t *const reg) {
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(device, PCT2075_REG_OVER_TEMP_SHTDWN, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read overtemperature shutdown register failed" );

    /* convert to signed 9-bit two's complement */
    *reg = pct2075_convert_9bit_twos_int16(rx);

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write hysteresis temperature register to PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 hysteresis temperature register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_i2c_set_hys_temperature_register(pct2075_device_t *const device, const int16_t reg) {
    bit16_uint8_buffer_t hys_buffer = { 0 };
    int16_t ots = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* get overtemperature shutdown temperature - overtemperature shutdown must be lower than hysteresis temperature */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_ots_temperature_register(device, &ots), TAG, "read overtemperature shutdown temperature register failed" );

    /* validate hysteresis and shutdown temperatures */
    ESP_RETURN_ON_FALSE( reg < ots, ESP_ERR_INVALID_ARG, TAG, "hysteresis temperature must be lower than overtemperature shutdown temperature" );

    /* convert signed decimal value to two's complement byte array */
    pct2075_convert_int16_twos_9bit(reg, hys_buffer);

    /* init ic2 transmission buffer */
    bit24_uint8_buffer_t tx = { PCT2075_REG_TEMP_HYST, hys_buffer[0], hys_buffer[1] }; // register, lsb, msb

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write(device, tx, BIT24_UINT8_BUFFER_SIZE), TAG, "write hysteresis temperature register failed" );

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL write overtemperature shutdown register to PCT2075.
 *
 * @param[in] device PCT2075 device descriptor.
 * @param[out] reg PCT2075 overtemperature shutdown register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_set_ots_temperature_register(pct2075_device_t *const device, const int16_t reg) {
    bit16_uint8_buffer_t ots_buffer = { 0 };
    int16_t hys = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* get hysteresis temperature - overtemperature shutdown must be higher than hysteresis temperature */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_hys_temperature_register(device, &hys), TAG, "read hysteresis temperature register failed" );

    /* validate hysteresis and shutdown temperatures */
    ESP_RETURN_ON_FALSE( reg > hys, ESP_ERR_INVALID_ARG, TAG, "overtemperature shutdown temperature must be higher than hysteresis temperature" );

    /* convert signed decimal value to two's complement byte array */
    pct2075_convert_int16_twos_9bit(reg, ots_buffer);

    /* init ic2 transmission buffer */
    bit24_uint8_buffer_t tx = { PCT2075_REG_OVER_TEMP_SHTDWN, ots_buffer[0], ots_buffer[1] }; // register, lsb, msb

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write(device, tx, BIT24_UINT8_BUFFER_SIZE), TAG, "write overtemperature shutdown temperature register failed" );

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL read ADC signal register.
 *
 * @param[in] device Pointer to the PCT2075 device descriptor.
 * @param[out] signal Pointer to a 2-byte buffer where the raw ADC signal will be stored.
 *                    The buffer will contain the raw 16-bit value read from the temperature register,
 *                    which should be interpreted as a signed 11-bit two's complement value for temperature conversion.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
static inline esp_err_t pct2075_i2c_get_adc_signal_register(pct2075_device_t *const device, bit16_uint8_buffer_t *const signal){
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && signal );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(device, PCT2075_REG_TEMP, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read adc signal register failed" );

    /* set output parameter */
    memcpy(signal, rx, BIT16_UINT8_BUFFER_SIZE);

    return ESP_OK;
}

/**
 * @brief PCT2075 I2C HAL to setup and configuration.
 * 
 * @param[in] device PCT2075 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_i2c_setup(pct2075_device_t *const device) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_sampling_period_register_t smp_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* configure set-point temperatures */
    if(device->config.configure_setpoints == true) {
        const int16_t ots = pct2075_convert_temperature_to_int16_signal(device->config.ots_temperature);
        const int16_t hys = pct2075_convert_temperature_to_int16_signal(device->config.hys_temperature);

        /* set overtemperature shutdown and hysteresis temperatures */
        ESP_RETURN_ON_ERROR( pct2075_i2c_set_ots_temperature_register(device, ots), TAG, "setting overtemperature set-point temperature register failed" );
        ESP_RETURN_ON_ERROR( pct2075_i2c_set_hys_temperature_register(device, hys), TAG, "setting hysteresis set-point temperature register failed" );
    }

    /* configure sampling interval */
    if(device->config.configure_sampling == true) {
        smp_reg.bits.tidle = pct2075_convert_sampling_period_to_steps(device->config.sampling_period);
        ESP_RETURN_ON_ERROR( pct2075_i2c_set_sampling_period_register(device, smp_reg), TAG, "setting sampling period register failed" );
    }

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* configure operation mode */
    if(device->config.operation_mode != PCT2075_OS_OP_MODE_COMPARATOR) {
        cfg_reg.bits.operation_mode = PCT2075_OS_OP_MODE_INTERRUPT; // set operation mode to interrupt
    }

    /* configure polarity */
    if(device->config.polarity != PCT2075_OS_POL_ACTIVE_LOW) {
        cfg_reg.bits.polarity = PCT2075_OS_POL_ACTIVE_HIGH; // set polarity to high
    }

    /* configure fault queue */
    if(device->config.fault_queue != PCT2075_OS_FAULT_QUEUE_1) {
        cfg_reg.bits.fault_queue = device->config.fault_queue; // set fault queue
    }

    /* enable or disable (i.e. shutdown) pct2075 */
    if(device->config.shutdown_enabled == true) {
        cfg_reg.bits.shutdown_enabled = true; // shutdown enabled
    }

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_config_register(device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_init(const i2c_master_bus_handle_t master_handle, const pct2075_config_t *pct2075_config, pct2075_handle_t *const pct2075_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && pct2075_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, pct2075_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ptc2075 device handle initialization failed", pct2075_config->i2c_address);

    /* validate memory availability for handle */
    pct2075_device_t* device = (pct2075_device_t*)calloc(1, sizeof(pct2075_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c pct2075 device");

    /* copy configuration */
    device->config = *pct2075_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_CMD_DELAY_MS));

    /* setup device */
    ESP_RETURN_ON_ERROR( pct2075_i2c_setup(device), TAG, "setup device failed" );

    /* set device handle */
    *pct2075_handle = (pct2075_handle_t)device;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t pct2075_get_temperature(pct2075_handle_t handle, float *const temperature){
    bit16_uint8_buffer_t adc_temp = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_adc_signal_register(device, &adc_temp), TAG, "read adc signal register for get temperature failed" );

    /* convert to signed 11-bit two's complement */
    *temperature = pct2075_convert_11bit_signal_to_temperature(adc_temp);

    return ESP_OK;
}

esp_err_t pct2075_get_ots_temperature(pct2075_handle_t handle, float *const temperature) {
    int16_t ots = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature );

    /* attempt i2c read transaction */
     ESP_RETURN_ON_ERROR( pct2075_i2c_get_ots_temperature_register(device, &ots), TAG, "read overtemperature shutdown register failed" );

    /* set output parameter */
    *temperature = pct2075_convert_int16_signal_to_temperature(ots);

    return ESP_OK;
}

esp_err_t pct2075_set_ots_temperature(pct2075_handle_t handle, const float temperature) {
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set temperature to signed decimal value - 0.5 degrees Celsius resolution */
    int16_t ots_temperature = pct2075_convert_temperature_to_int16_signal(temperature);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_ots_temperature_register(device, ots_temperature), TAG, "write overtemperature shutdown temperature register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_hys_temperature(pct2075_handle_t handle, float *const temperature) {
    int16_t hys = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_hys_temperature_register(device, &hys), TAG, "read hysteresis temperature register failed" );

    /* set output parameter */
    *temperature = pct2075_convert_int16_signal_to_temperature(hys);

    return ESP_OK;
}

esp_err_t pct2075_set_hys_temperature(pct2075_handle_t handle, const float temperature) {
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set temperature to signed decimal value - 0.5 degrees Celsius resolution */
    int16_t hys_temperature = pct2075_convert_temperature_to_int16_signal(temperature);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_hys_temperature_register(device, hys_temperature), TAG, "write hysteresis temperature register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_sampling_period(pct2075_handle_t handle, uint16_t *const period) {
    pct2075_sampling_period_register_t reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && period );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_sampling_period_register(device, &reg), TAG, "read sampling period register failed" );

    /* convert time steps to sampling period */
    *period = pct2075_convert_steps_to_sampling_period(reg.bits.tidle);

    return ESP_OK;
}

esp_err_t pct2075_set_sampling_period(pct2075_handle_t handle, const uint16_t period) {
    pct2075_sampling_period_register_t reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* convert sampling period to time steps */
    reg.bits.tidle = pct2075_convert_sampling_period_to_steps(period);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_sampling_period_register(device, reg), TAG, "write sampling period register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_operation_mode(pct2075_handle_t handle, pct2075_os_operation_modes_t *const operation_mode) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && operation_mode );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set operation mode */
    *operation_mode = (cfg_reg.bits.operation_mode == PCT2075_OS_OP_MODE_COMPARATOR)
        ? PCT2075_OS_OP_MODE_COMPARATOR     // true - set operation mode to comparator
        : PCT2075_OS_OP_MODE_INTERRUPT;     // false - set operation mode to interrupt

    return ESP_OK;
}

esp_err_t pct2075_set_operation_mode(pct2075_handle_t handle, const pct2075_os_operation_modes_t operation_mode) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set operation mode */
    cfg_reg.bits.operation_mode = (operation_mode == PCT2075_OS_OP_MODE_COMPARATOR)
        ? PCT2075_OS_OP_MODE_COMPARATOR     // true - set operation mode to comparator
        : PCT2075_OS_OP_MODE_INTERRUPT;     // false - set operation mode to interrupt

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_config_register(device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_polarity(pct2075_handle_t handle, pct2075_os_polarities_t *const polarity) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && polarity );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set polarity */
    *polarity = (cfg_reg.bits.polarity == PCT2075_OS_POL_ACTIVE_LOW)
        ? PCT2075_OS_POL_ACTIVE_LOW     // true - set polarity to active low
        : PCT2075_OS_POL_ACTIVE_HIGH;   // false - set polarity to active high

    return ESP_OK;
}

esp_err_t pct2075_set_polarity(pct2075_handle_t handle, const pct2075_os_polarities_t polarity) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set polarity */
    cfg_reg.bits.polarity = (polarity == PCT2075_OS_POL_ACTIVE_LOW)
        ? PCT2075_OS_POL_ACTIVE_LOW     // true - set polarity to active low
        : PCT2075_OS_POL_ACTIVE_HIGH;   // false - set polarity to active high

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_config_register(device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_fault_queue(pct2075_handle_t handle, pct2075_os_fault_queues_t *const fault_queue) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && fault_queue );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set fault queue */
    if(cfg_reg.bits.fault_queue == PCT2075_OS_FAULT_QUEUE_1) {
        *fault_queue = PCT2075_OS_FAULT_QUEUE_1; // set fault queue to 1
    } else if(cfg_reg.bits.fault_queue == PCT2075_OS_FAULT_QUEUE_2) {
        *fault_queue = PCT2075_OS_FAULT_QUEUE_2; // set fault queue to 2
    } else if(cfg_reg.bits.fault_queue == PCT2075_OS_FAULT_QUEUE_4) {
        *fault_queue = PCT2075_OS_FAULT_QUEUE_4; // set fault queue to 4
    } else {
        *fault_queue = PCT2075_OS_FAULT_QUEUE_6; // set fault queue to 6
    }

    return ESP_OK;
}

esp_err_t pct2075_set_fault_queue(pct2075_handle_t handle, const pct2075_os_fault_queues_t fault_queue) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set fault queue */
    if(fault_queue == PCT2075_OS_FAULT_QUEUE_1) {
        cfg_reg.bits.fault_queue = PCT2075_OS_FAULT_QUEUE_1; // set fault queue to 1
    } else if(fault_queue == PCT2075_OS_FAULT_QUEUE_2) {
        cfg_reg.bits.fault_queue = PCT2075_OS_FAULT_QUEUE_2; // set fault queue to 2
    } else if(fault_queue == PCT2075_OS_FAULT_QUEUE_4) {
        cfg_reg.bits.fault_queue = PCT2075_OS_FAULT_QUEUE_4; // set fault queue to 4
    } else {
        cfg_reg.bits.fault_queue = PCT2075_OS_FAULT_QUEUE_6; // set fault queue to 6
    }

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_config_register(device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_disable(pct2075_handle_t handle) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set shutdown enabled */
    cfg_reg.bits.shutdown_enabled = true; // shutdown enabled

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_config_register(device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_enable(pct2075_handle_t handle) {
    pct2075_config_register_t cfg_reg = { 0 };
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_get_config_register(device, &cfg_reg), TAG, "read configuration register failed" );

    /* set shutdown enabled */
    cfg_reg.bits.shutdown_enabled = false; // shutdown disabled

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_i2c_set_config_register(device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_remove(pct2075_handle_t handle) {
    pct2075_device_t* device = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(device->i2c_handle);
}

esp_err_t pct2075_delete(pct2075_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( pct2075_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* pct2075_get_fw_version(void) {
    return (const char*)PCT2075_FW_VERSION_STR;
}

int32_t pct2075_get_fw_version_number(void) {
    return (int32_t)PCT2075_FW_VERSION_INT32;
}