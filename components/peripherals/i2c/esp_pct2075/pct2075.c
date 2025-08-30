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

    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "pct2075_i2c_write failed" );

    return ESP_OK;
}

/**
 * @brief Converts a 11-bit value, using two's compliment, to a signed 16-bit integer value.
 * 
 * @param buffer Buffer containing the register's value as byte array (2-bytes).
 * @return int16_t Signed 16-bit integer value representing the 11-bit two's complement value.
 */
static inline int16_t pct2075_11bit_twos_int16(const bit16_uint8_buffer_t buffer) {
    // convert bytes to unsigned 16-bit integer using two's complement
    uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
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
static inline int16_t pct2075_9bit_twos_int16(const bit16_uint8_buffer_t buffer) {
    // convert bytes to unsigned 16-bit integer using two's complement
    uint16_t raw = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
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
static inline void pct2075_int16_twos_9bit(const int16_t value, bit16_uint8_buffer_t buffer) {
    // Convert int16_t value to 9-bit two's complement and store in buffer
    uint16_t temp = ((uint16_t)(value & 0x01FF)) << 7; // mask to 9 bits, then shift
    buffer[0] = (uint8_t)((temp >> 8) & 0xFF);
    buffer[1] = (uint8_t)(temp & 0xFF);
}

/**
 * @brief Sets up the PCT2075 device with the provided configuration.
 * 
 * @param handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t pct2075_setup(pct2075_device_t *const device) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* configure set-point temperatures */
    if(device->config.configure_setpoints == true) {
        ESP_RETURN_ON_ERROR( pct2075_set_ots_temperature((pct2075_handle_t)device, device->config.ots_temperature), TAG, "setting overtemperature set-point temperature failed" );
        ESP_RETURN_ON_ERROR( pct2075_set_hys_temperature((pct2075_handle_t)device, device->config.hys_temperature), TAG, "setting hysteresis set-point temperature failed" );
    }

    /* configure sampling interval */
    if(device->config.configure_sampling == true) {
        ESP_RETURN_ON_ERROR( pct2075_set_sampling_period((pct2075_handle_t)device, device->config.sampling_period), TAG, "setting sampling period failed" );
    }

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register((pct2075_handle_t)device, &cfg_reg), TAG, "read configuration register failed" );

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
    ESP_RETURN_ON_ERROR( pct2075_set_config_register((pct2075_handle_t)device, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_config_register(pct2075_handle_t handle, pct2075_config_register_t *const reg) {
    uint16_t cfg;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_word_from(dev, PCT2075_REG_CONFIG, &cfg), TAG, "read configuration register failed" );

    reg->reg = cfg;
    
    return ESP_OK;
}

esp_err_t pct2075_set_config_register(pct2075_handle_t handle, const pct2075_config_register_t reg) {
    pct2075_config_register_t config = { .reg = reg.reg };
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    config.bits.reserved = 0;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write_word_to(dev, PCT2075_REG_CONFIG, config.reg), TAG, "write configuration register failed" );
    
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
    pct2075_device_t* dev = (pct2075_device_t*)calloc(1, sizeof(pct2075_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c pct2075 device");

    /* copy configuration */
    dev->config = *pct2075_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_CMD_DELAY_MS));

    /* setup device */
    ESP_RETURN_ON_ERROR( pct2075_setup(dev), TAG, "setup device failed" );

    /* set device handle */
    *pct2075_handle = (pct2075_handle_t)dev;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(PCT2075_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t pct2075_get_temperature(pct2075_handle_t handle, float *const temperature){
    bit16_uint8_buffer_t rx;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(dev, PCT2075_REG_TEMP, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read temperature register failed" );

    /* convert to signed 11-bit two's complement */
    *temperature = (float)pct2075_11bit_twos_int16(rx) * 0.125f;

    return ESP_OK;
}

esp_err_t pct2075_get_ots_temperature(pct2075_handle_t handle, float *const temperature) {
    bit16_uint8_buffer_t rx;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(dev, PCT2075_REG_OVER_TEMP_SHTDWN, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read overtemperature shutdown register failed" );

    /* convert to signed 9-bit two's complement */
    *temperature = (float)pct2075_9bit_twos_int16(rx) * 0.5f;

    return ESP_OK;
}

esp_err_t pct2075_set_ots_temperature(pct2075_handle_t handle, const float temperature) {
    bit16_uint8_buffer_t ots_buffer;
    float hys_temperature;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* get hysteresis temperature - overtemperature shutdown must be higher than hysteresis temperature */
    ESP_RETURN_ON_ERROR( pct2075_get_hys_temperature(handle, &hys_temperature), TAG, "read hysteresis temperature failed" );

    /* validate hysteresis and shutdown temperatures */
    ESP_RETURN_ON_FALSE( temperature > hys_temperature, ESP_ERR_INVALID_ARG, TAG, "overtemperature shutdown temperature must be higher than hysteresis temperature" );

    /* set temperature to signed decimal value - 0.5 degrees Celsius resolution */
    int16_t ots_temperature = (int16_t)(temperature / 0.5f);

    /* convert signed decimal value to two's complement byte array */
    pct2075_int16_twos_9bit(ots_temperature, ots_buffer);

    /* init ic2 transmission buffer */
    bit24_uint8_buffer_t tx = { PCT2075_REG_OVER_TEMP_SHTDWN, ots_buffer[0], ots_buffer[1] }; // register, lsb, msb

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write(dev, tx, BIT24_UINT8_BUFFER_SIZE), TAG, "write overtemperature shutdown temperature register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_hys_temperature(pct2075_handle_t handle, float *const temperature) {
    bit16_uint8_buffer_t rx;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(dev, PCT2075_REG_TEMP_HYST, rx, BIT16_UINT8_BUFFER_SIZE), TAG, "read hysteresis temperature register failed" );

    /* convert to signed 9-bit two's complement */
    *temperature = (float)pct2075_9bit_twos_int16(rx) * 0.5f;

    return ESP_OK;
}

esp_err_t pct2075_set_hys_temperature(pct2075_handle_t handle, const float temperature) {
    bit16_uint8_buffer_t hys_buffer;
    float ots_temperature;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* get overtemperature shutdown temperature - overtemperature shutdown must be lower than hysteresis temperature */
    ESP_RETURN_ON_ERROR( pct2075_get_ots_temperature(handle, &ots_temperature), TAG, "read overtemperature shutdown temperature failed" );

    /* validate hysteresis and shutdown temperatures */
    ESP_RETURN_ON_FALSE( temperature < ots_temperature, ESP_ERR_INVALID_ARG, TAG, "hysteresis temperature must be lower than overtemperature shutdown temperature" );

    /* set temperature to signed decimal value - 0.5 degrees Celsius resolution */
    int16_t hys_temperature = (int16_t)(temperature / 0.5f);

    /* convert signed decimal value to two's complement byte array */
    pct2075_int16_twos_9bit(hys_temperature, hys_buffer);

    /* init ic2 transmission buffer */
    bit24_uint8_buffer_t tx = { PCT2075_REG_TEMP_HYST, hys_buffer[0], hys_buffer[1] }; // register, lsb, msb

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write(dev, tx, BIT24_UINT8_BUFFER_SIZE), TAG, "write hysteresis temperature register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_sampling_period(pct2075_handle_t handle, uint16_t *const period) {
    bit8_uint8_buffer_t rx;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_read_from(dev, PCT2075_REG_TEMP_IDLE, rx, BIT8_UINT8_BUFFER_SIZE), TAG, "read sampling period register failed" );

    /* convert to sampling period register */
    pct2075_sampling_period_register_t reg = { .reg = rx[0] };

    /* convert temperature idle to sampling period in milliseconds */
    *period = (uint16_t)(reg.bits.tidle * 100);

    return ESP_OK;
}

esp_err_t pct2075_set_sampling_period(pct2075_handle_t handle, const uint16_t period) {
    pct2075_sampling_period_register_t reg;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    if(period == 0) {
        reg.bits.tidle = 1; // 100ms
    } else {
        reg.bits.tidle = (uint8_t)(period / 100); // convert to tidle
    }

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( pct2075_i2c_write_byte_to(dev, PCT2075_REG_TEMP_IDLE, reg.reg), TAG, "write sampling period register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_operation_mode(pct2075_handle_t handle, pct2075_os_operation_modes_t *const operation_mode) {
    pct2075_config_register_t cfg_reg;
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(dev, &cfg_reg), TAG, "read configuration register failed" );

    /* set operation mode */
    if(cfg_reg.bits.operation_mode == PCT2075_OS_OP_MODE_COMPARATOR) {
        *operation_mode = PCT2075_OS_OP_MODE_COMPARATOR; // set operation mode to comparator
    } else {
        *operation_mode = PCT2075_OS_OP_MODE_INTERRUPT; // set operation mode to interrupt
    }

    return ESP_OK;
}

esp_err_t pct2075_set_operation_mode(pct2075_handle_t handle, const pct2075_os_operation_modes_t operation_mode) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

    /* set operation mode */
    if(operation_mode == PCT2075_OS_OP_MODE_COMPARATOR) {
        cfg_reg.bits.operation_mode = PCT2075_OS_OP_MODE_COMPARATOR; // set operation mode to comparator
    } else {
        cfg_reg.bits.operation_mode = PCT2075_OS_OP_MODE_INTERRUPT; // set operation mode to interrupt
    }

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_set_config_register(handle, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_polarity(pct2075_handle_t handle, pct2075_os_polarities_t *const polarity) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

    /* set polarity */
    if(cfg_reg.bits.polarity == PCT2075_OS_POL_ACTIVE_LOW) {
        *polarity = PCT2075_OS_POL_ACTIVE_LOW; // set polarity to active low
    } else {
        *polarity = PCT2075_OS_POL_ACTIVE_HIGH; // set polarity to active high
    }

    return ESP_OK;
}

esp_err_t pct2075_set_polarity(pct2075_handle_t handle, const pct2075_os_polarities_t polarity) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

    /* set polarity */
    if(polarity == PCT2075_OS_POL_ACTIVE_LOW) {
        cfg_reg.bits.polarity = PCT2075_OS_POL_ACTIVE_LOW; // set polarity to active low
    } else {
        cfg_reg.bits.polarity = PCT2075_OS_POL_ACTIVE_HIGH; // set polarity to active high
    }

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_set_config_register(handle, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_get_fault_queue(pct2075_handle_t handle, pct2075_os_fault_queues_t *const fault_queue) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

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
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

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
    ESP_RETURN_ON_ERROR( pct2075_set_config_register(handle, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_disable(pct2075_handle_t handle) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

    /* set shutdown enabled */
    cfg_reg.bits.shutdown_enabled = true; // shutdown enabled

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_set_config_register(handle, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_enable(pct2075_handle_t handle) {
    pct2075_config_register_t cfg_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* read configuration register */
    ESP_RETURN_ON_ERROR( pct2075_get_config_register(handle, &cfg_reg), TAG, "read configuration register failed" );

    /* set shutdown enabled */
    cfg_reg.bits.shutdown_enabled = false; // shutdown disabled

    /* write configuration register */
    ESP_RETURN_ON_ERROR( pct2075_set_config_register(handle, cfg_reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t pct2075_remove(pct2075_handle_t handle) {
    pct2075_device_t* dev = (pct2075_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(dev->i2c_handle);
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
    return (char *)PCT2075_FW_VERSION_STR;
}

int32_t pct2075_get_fw_version_number(void) {
    return PCT2075_FW_VERSION_INT32;
}