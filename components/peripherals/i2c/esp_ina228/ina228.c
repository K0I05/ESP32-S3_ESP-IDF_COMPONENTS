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
 * @file ina226.c
 *
 * ESP-IDF driver for INA225 current, voltage, and power monitoring sensor
 * 
 * 
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ina228.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define INA228_REG_CONFIG               (0x00)
#define INA228_REG_ADC_CONFIG           (0x01)
#define INA228_REG_SHUNT_CAL            (0x02)
#define INA228_REG_SHUNT_TEMP_COEF      (0x03)
#define INA228_REG_VOLT_SHUNT           (0x04)
#define INA228_REG_VOLT_BUS             (0x05)
#define INA228_REG_DIE_TEMP             (0x06)
#define INA228_REG_CURRENT              (0x07)
#define INA228_REG_POWER                (0x08)
#define INA228_REG_ENERGY               (0x09)
#define INA228_REG_CHARGE               (0x0a)
#define INA228_REG_DIAG_ALERT           (0x0b)
#define INA228_REG_SHUNT_OVL_THRESH     (0x0c)
#define INA228_REG_SHUNT_UVL_THRESH     (0x0d)
#define INA228_REG_BUS_OVL_THRESH       (0x0e)
#define INA228_REG_BUS_UVL_THRESH       (0x0f)
#define INA228_REG_TEMP_OVL_THRESH      (0x10)
#define INA228_REG_POWER_OVL_THRESH     (0x11)
#define INA228_REG_MANU_ID              (0x3e)
#define INA228_REG_DEVICE_ID            (0x3f)


#define INA228_MIN_SHUNT_RESISTANCE     (0.001)         /*!< minimum allowable shunt resistance in ohms */
#define INA228_MAX_SHUNT_VOLTAGE        (81.92 / 1000)  /*!< maximum allowable shunt voltage in volts */

#define INA228_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define INA228_POWERUP_DELAY_MS         UINT16_C(25)    //!< ina226 I2C start-up delay before device accepts transactions
#define INA228_APPSTART_DELAY_MS        UINT16_C(25)            
#define INA228_RESET_DELAY_MS           UINT16_C(250)   //!< ina226 I2C software reset delay before device accepts transactions
#define INA228_DATA_READY_DELAY_MS      UINT16_C(10)
#define INA228_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define INA228_CMD_DELAY_MS             UINT16_C(10)
#define INA228_TX_RX_DELAY_MS           UINT16_C(10)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "ina226";

/*
* functions and subroutines
*/

/**
 * @brief INA228 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle INA228 device handle.
 * @param reg_addr INA228 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina228_i2c_read_from(ina228_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "ina228_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief INA228 I2C read halfword (two's compliment) from register address transaction.
 * 
 * @param handle INA228 device handle.
 * @param reg_addr INA228 register address to read from.
 * @param word INA228 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina228_i2c_read_word_from(ina228_handle_t handle, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ina228_i2c_read_word_from failed" );

    /* set output parameter */
    *word = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    return ESP_OK;
}

/**
 * @brief INA228 I2C write halfword (two's compliment) to register address transaction.
 * 
 * @param handle INA228 device handle.
 * @param reg_addr INA228 register address to write to.
 * @param word INA228 write transaction input halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina228_i2c_write_word_to(ina228_handle_t handle, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)((word >> 8) & 0xff), (uint8_t)(word & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ina228_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

static inline int32_t ina228_20bit_to_int32(const bit24_uint8_buffer_t buffer) {
    // convert bytes to unsigned 32-bit integer using two's complement
    uint32_t sig = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
    // shift right by 12, i.e. 20-bits are interest, and sign value
    int32_t sigd = (int32_t)sig >> 12;
    return sigd;
}


esp_err_t ina228_get_configuration_register(ina228_handle_t handle, ina228_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t cfg;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_word_from(handle, INA228_REG_CONFIG, &cfg), TAG, "read configuration register failed" );

    reg->reg = cfg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_set_configuration_register(ina228_handle_t handle, const ina228_config_register_t reg) {
    ina228_config_register_t config = { .reg = reg.reg };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    config.bits.reserved = 0;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_write_word_to(handle, INA228_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_get_adc_configuration_register(ina228_handle_t handle, ina228_adc_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t cfg;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_word_from(handle, INA228_REG_ADC_CONFIG, &cfg), TAG, "read adc configuration register failed" );

    reg->reg = cfg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_set_adc_configuration_register(ina228_handle_t handle, const ina228_adc_config_register_t reg) {
    ina228_adc_config_register_t config = { .reg = reg.reg };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_write_word_to(handle, INA228_REG_ADC_CONFIG, config.reg), TAG, "write adc configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_get_shunt_calibration_register(ina228_handle_t handle, ina228_shunt_calibration_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t cfg;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_word_from(handle, INA228_REG_SHUNT_CAL, &cfg), TAG, "read shunt calibration register failed" );

    reg->reg = cfg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_set_shunt_calibration_register(ina228_handle_t handle, const ina228_shunt_calibration_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_write_word_to(handle, INA228_REG_SHUNT_CAL, reg.reg), TAG, "write shunt calibration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_get_diagnostic_alert_register(ina228_handle_t handle, ina228_diagnostic_alert_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t mske;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_word_from(handle, INA228_REG_DIAG_ALERT, &mske), TAG, "read mask-enable register failed" );

    reg->reg = mske;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina228_init(i2c_master_bus_handle_t master_handle, const ina228_config_t *ina228_config, ina228_handle_t *ina228_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && ina228_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ina228_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ina228 device handle initialization failed", ina228_config->i2c_address);

    /* validate memory availability for handle */
    ina228_handle_t out_handle;
    out_handle = (ina228_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ini228 device");

    /* copy configuration */
    out_handle->dev_config = *ina228_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* attempt to soft-reset */
    ESP_GOTO_ON_ERROR(ina228_reset(out_handle), err_handle, TAG, "unable to soft-reset, init failed");

    /* set device handle */
    *ina228_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t ina228_shunt_calibration(ina228_handle_t handle, const float max_current, const float shunt_resistance) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_FALSE((max_current > 0.001), ESP_ERR_INVALID_ARG, TAG, "invalid maximum current, too low, shunt calibration failed");
    ESP_RETURN_ON_FALSE((shunt_resistance > INA228_MIN_SHUNT_RESISTANCE), ESP_ERR_INVALID_ARG, TAG, "invalid shunt resistance, too low, shunt calibration failed");

    handle->current_lsb = max_current / powf(2.0f, 19.0f);

    //  PAGE 31 (8.1.2)
    float shunt_cal = 13107.2e6 * handle->current_lsb * shunt_resistance;

    if(handle->dev_config.adc_range == INA228_ADC_RANGE_40_96MV) {
        shunt_cal *= 4;
    }

    ina228_shunt_calibration_register_t shunt_cal_reg;
    shunt_cal_reg.bits.shunt_calibration = (uint16_t)shunt_cal;

    ESP_RETURN_ON_ERROR( ina228_set_shunt_calibration_register(handle, shunt_cal_reg), TAG, "write shunt calibration register failed" );

    return ESP_OK;
}

esp_err_t ina228_get_shunt_voltage(ina228_handle_t handle, float *const voltage) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && voltage );

    bit24_uint8_buffer_t rx = { 0 };

    float shunt_lsb = 312.5e-9;  //  312.5 nV
    if(handle->dev_config.adc_range == INA228_ADC_RANGE_40_96MV) {
        shunt_lsb = 78.125e-9;     //  78.125 nV
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_from(handle, INA228_REG_VOLT_SHUNT, rx, BIT24_UINT8_BUFFER_SIZE), TAG, "read shunt voltage failed" );

    int32_t sig = ina228_20bit_to_int32(rx);

    *voltage =(float)sig * shunt_lsb;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina228_get_bus_voltage(ina228_handle_t handle, float *const voltage) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && voltage );

    bit24_uint8_buffer_t rx = { 0 };

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_from(handle, INA228_REG_VOLT_BUS, rx, BIT24_UINT8_BUFFER_SIZE), TAG, "read bus voltage failed" );
 
    int32_t sig = ina228_20bit_to_int32(rx);

    float bus_lsb  = 195.3125e-6;  //  195.3125 uV

    *voltage = (float)sig * bus_lsb; /* 195.3125 uV */

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina228_get_current(ina228_handle_t handle, float *const current) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && current );

    bit24_uint8_buffer_t rx = { 0 };

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_from(handle, INA228_REG_CURRENT, rx, BIT24_UINT8_BUFFER_SIZE), TAG, "read current failed" );

    int32_t sig = ina228_20bit_to_int32(rx);

    *current = (float)sig * handle->current_lsb;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina228_get_power(ina228_handle_t handle, float *const power) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && power );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_word_from(handle, INA228_REG_POWER, &sig), TAG, "read power failed" );

    *power = (float)sig * handle->current_lsb * 3.2f;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina228_get_temperature(ina228_handle_t handle, float *const temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && temperature );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina228_i2c_read_word_from(handle, INA228_REG_DIE_TEMP, &sig), TAG, "read temperature failed" );

    float lsb = 7.8125e-3;  //  milli degree Celsius

    *temperature = (float)sig * lsb;

    return ESP_OK;
}

esp_err_t ina228_get_mode(ina228_handle_t handle, ina228_operating_modes_t *const mode) {
    ina228_adc_config_register_t adc_config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read adc configuration register */
    ESP_RETURN_ON_ERROR( ina228_get_adc_configuration_register(handle, &adc_config), TAG, "read configuration register failed" );

    /* set mode */
    *mode = adc_config.bits.operating_mode;

    return ESP_OK;
}

esp_err_t ina228_set_mode(ina228_handle_t handle, const ina228_operating_modes_t mode) {
    ina228_adc_config_register_t adc_config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( ina228_get_adc_configuration_register(handle, &adc_config), TAG, "read configuration register failed" );

    /* set mode */
    adc_config.bits.operating_mode = mode;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( ina228_set_adc_configuration_register(handle, adc_config), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t ina228_reset(ina228_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ina228_config_register_t config;
    ina228_adc_config_register_t adc_config;

    /* attempt to read configuration register */

    config.bits.reset_enabled = true;

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR(ina228_set_configuration_register(handle, config), TAG, "unable to write configuration register, reset failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA228_RESET_DELAY_MS));

    /* attempt to configure device */
    ESP_RETURN_ON_ERROR(ina228_get_configuration_register(handle, &config), TAG, "unable to read configuration register, reset failed");

    /* attempt to configure device */
    ESP_RETURN_ON_ERROR(ina228_get_adc_configuration_register(handle, &adc_config), TAG, "unable to read adc configuration register, reset failed");

    config.bits.adc_range                   = handle->dev_config.adc_range;

    adc_config.bits.operating_mode          = handle->dev_config.operating_mode;
    adc_config.bits.averaging_mode          = handle->dev_config.averaging_mode;
    adc_config.bits.bus_voltage_conv_time   = handle->dev_config.bus_voltage_conv_time;
    adc_config.bits.shunt_voltage_conv_time = handle->dev_config.shunt_voltage_conv_time;
    adc_config.bits.temperature_conv_time   = handle->dev_config.temperature_conv_time;

    ESP_RETURN_ON_ERROR(ina228_set_configuration_register(handle, config), TAG, "unable to write configuration register, reset failed");

    ESP_RETURN_ON_ERROR(ina228_set_adc_configuration_register(handle, adc_config), TAG, "unable to write adc configuration register, reset failed");

    ESP_RETURN_ON_ERROR(ina228_shunt_calibration(handle, handle->dev_config.max_current, handle->dev_config.shunt_resistance), TAG, "unable to write shunt calibration register, reset failed");

    return ESP_OK;
}

esp_err_t ina228_remove(ina228_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t ina228_delete(ina228_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( ina228_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* ina228_get_fw_version(void) {
    return (char *)INA228_FW_VERSION_STR;
}

int32_t ina228_get_fw_version_number(void) {
    return INA228_FW_VERSION_INT32;
}
