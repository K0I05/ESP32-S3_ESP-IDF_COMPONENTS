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
// https://github.com/cybergear-robotics/ina226/blob/master/ina226.c
/**
 * @file ina226.c
 *
 * ESP-IDF driver for INA226 current, voltage, and power monitoring sensor
 * 
 * 
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ina226.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define INA226_REG_CONFIG               (0x00)
#define INA226_REG_SHUNT_V              (0x01)
#define INA226_REG_BUS_V                (0x02)
#define INA226_REG_POWER                (0x03)
#define INA226_REG_CURRENT              (0x04)
#define INA226_REG_CALIBRATION          (0x05)
#define INA226_REG_MSK_ENA              (0x06)
#define INA226_REG_MANU_ID              (0xfe)
#define INA226_REG_DIE_ID               (0xff)

#define INA226_DEF_CONFIG               0x399f
#define INA226_ASUKIAAA_DEFAULT_CONFIG  0x4127

#define INA226_MIN_SHUNT_RESISTANCE     (0.001)         /*!< minimum allowable shunt resistance in ohms */
#define INA226_MAX_SHUNT_VOLTAGE        (81.92 / 1000)  /*!< maximum allowable shunt voltage in volts */

#define INA226_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define INA226_POWERUP_DELAY_MS         UINT16_C(25)    //!< ina226 I2C start-up delay before device accepts transactions
#define INA226_APPSTART_DELAY_MS        UINT16_C(25)            
#define INA226_RESET_DELAY_MS           UINT16_C(250)   //!< ina226 I2C software reset delay before device accepts transactions
#define INA226_DATA_READY_DELAY_MS      UINT16_C(10)
#define INA226_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define INA226_CMD_DELAY_MS             UINT16_C(10)
#define INA226_TX_RX_DELAY_MS           UINT16_C(10)

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
 * @brief INA226 I2C read halfword from register address transaction.
 * 
 * @param handle INA226 device handle.
 * @param reg_addr INA226 register address to read from.
 * @param word INA226 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina226_i2c_read_word_from(ina226_handle_t handle, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ina226_i2c_read_word_from failed" );

    /* set output parameter */
    *word = ((uint16_t)rx[0] << 8) | (uint16_t)rx[1];

    return ESP_OK;
}

/**
 * @brief INA226 I2C write halfword to register address transaction.
 * 
 * @param handle INA226 device handle.
 * @param reg_addr INA226 register address to write to.
 * @param word INA226 write transaction input halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ina226_i2c_write_word_to(ina226_handle_t handle, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)((word >> 8) & 0xff), (uint8_t)(word & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ina226_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

esp_err_t ina226_get_configuration_register(ina226_handle_t handle, ina226_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t cfg;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CONFIG, &cfg), TAG, "read configuration register failed" );

    reg->reg = cfg;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_set_configuration_register(ina226_handle_t handle, const ina226_config_register_t reg) {
    ina226_config_register_t config = { .reg = reg.reg };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    config.bits.reserved = 0;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_write_word_to(handle, INA226_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_get_calibration_register(ina226_handle_t handle, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CALIBRATION, reg), TAG, "read calibration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_set_calibration_register(ina226_handle_t handle, const uint16_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_write_word_to(handle, INA226_REG_CALIBRATION, reg), TAG, "write calibration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_get_mask_enable_register(ina226_handle_t handle, ina226_mask_enable_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    uint16_t mske;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_MSK_ENA, &mske), TAG, "read mask-enable register failed" );

    reg->reg = mske;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ina226_init(i2c_master_bus_handle_t master_handle, const ina226_config_t *ina226_config, ina226_handle_t *ina226_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && ina226_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ina226_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ccs811 device handle initialization failed", ina226_config->i2c_address);

    /* validate memory availability for handle */
    ina226_handle_t out_handle;
    out_handle = (ina226_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ini226 device");

    /* copy configuration */
    out_handle->dev_config = *ina226_config;

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
    ESP_GOTO_ON_ERROR(ina226_reset(out_handle), err_handle, TAG, "unable to soft-reset, init failed");

    /* set device handle */
    *ina226_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t ina226_calibrate(ina226_handle_t handle, const float max_current, const float shunt_resistance) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    float shunt_volt = max_current * shunt_resistance;

    ESP_RETURN_ON_FALSE((shunt_volt < INA226_MAX_SHUNT_VOLTAGE), ESP_ERR_INVALID_ARG, TAG, "invalid shunt voltage, too high, calibration failed");
    ESP_RETURN_ON_FALSE((max_current > 0.001), ESP_ERR_INVALID_ARG, TAG, "invalid maximum current, too low, calibration failed");
    ESP_RETURN_ON_FALSE((shunt_resistance > INA226_MIN_SHUNT_RESISTANCE), ESP_ERR_INVALID_ARG, TAG, "invalid shunt resistance, too low, calibration failed");

    float minimum_lsb = max_current / 32767;
    float current_lsb = (uint16_t)(minimum_lsb * 100000000);
    current_lsb /= 100000000;
    current_lsb /= 0.0001;
    current_lsb = ceil(current_lsb);
    current_lsb *= 0.0001;
    handle->current_lsb = current_lsb;
    uint16_t cal = (uint16_t)((0.00512) / (current_lsb * shunt_resistance));

    ESP_RETURN_ON_ERROR(ina226_set_calibration_register(handle, cal), TAG, "unable to write calibration register, calibration failed");

    return ESP_OK;
}

esp_err_t ina226_get_shunt_voltage(ina226_handle_t handle, float *const voltage) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && voltage );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_SHUNT_V, &sig), TAG, "read shunt voltage failed" );

    *voltage = (float)sig * 2.5e-6f; /* fixed to 2.5 uV */

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_triggered_shunt_voltage(ina226_handle_t handle, float *const voltage) {
    esp_err_t     ret           = ESP_OK;
    uint64_t      start_time    = esp_timer_get_time();
    bool          is_data_ready = false;
    uint16_t      sig;

    /* validate arguments */
    ESP_ARG_CHECK( handle && voltage );

    /* attempt to trigger bus voltage */
    ESP_RETURN_ON_ERROR( ina226_set_operating_mode(handle, INA226_OP_MODE_TRIG_SHUNT_VOLT), TAG, "unable to trigger shunt voltage, get triggered shunt voltage failed" );

    /* attempt to poll status until data is available or timeout occurs  */
    do {
        ina226_mask_enable_register_t mske;

        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ina226_get_mask_enable_register(handle, &mske), err, TAG, "unable to read mask-enable register, get triggered shunt voltage failed." );

        is_data_ready = mske.bits.conversion_ready_flag;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(INA226_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (INA226_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (is_data_ready == false);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_SHUNT_V, &sig), TAG, "unable to read shunt voltage, get triggered shunt voltage failed" );

    *voltage = (float)sig * 2.5e-6f; /* fixed to 2.5 uV */

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ina226_get_bus_voltage(ina226_handle_t handle, float *const voltage) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && voltage );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_BUS_V, &sig), TAG, "read bus voltage failed" );

    *voltage = (float)sig * 0.00125f;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_triggered_bus_voltage(ina226_handle_t handle, float *const voltage) {
    esp_err_t     ret           = ESP_OK;
    uint64_t      start_time    = esp_timer_get_time();
    bool          is_data_ready = false;
    uint16_t      sig;

    /* validate arguments */
    ESP_ARG_CHECK( handle && voltage );

    /* attempt to trigger bus voltage */
    ESP_RETURN_ON_ERROR( ina226_set_operating_mode(handle, INA226_OP_MODE_TRIG_BUS_VOLT), TAG, "unable to trigger bus voltage, get triggered bus voltage failed" );

    /* attempt to poll status until data is available or timeout occurs  */
    do {
        ina226_mask_enable_register_t mske;

        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ina226_get_mask_enable_register(handle, &mske), err, TAG, "unable to read mask-enable register, get triggered bus voltage failed." );

        is_data_ready = mske.bits.conversion_ready_flag;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(INA226_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (INA226_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (is_data_ready == false);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_BUS_V, &sig), TAG, "unable to read bus voltage, get triggered bus voltage failed" );

    *voltage = (float)sig * 0.00125f;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ina226_get_current(ina226_handle_t handle, float *const current) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && current );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CURRENT, &sig), TAG, "read bus voltage failed" );

    *current = (float)sig * handle->current_lsb;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_triggered_current(ina226_handle_t handle, float *const current) {
    esp_err_t     ret           = ESP_OK;
    uint64_t      start_time    = esp_timer_get_time();
    bool          is_data_ready = false;
    uint16_t      sig;

    /* validate arguments */
    ESP_ARG_CHECK( handle && current );

    /* attempt to trigger bus voltage */
    ESP_RETURN_ON_ERROR( ina226_set_operating_mode(handle, INA226_OP_MODE_TRIG_SHUNT_BUS), TAG, "unable to trigger current, get triggered current failed" );

    /* attempt to poll status until data is available or timeout occurs  */
    do {
        ina226_mask_enable_register_t mske;

        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ina226_get_mask_enable_register(handle, &mske), err, TAG, "unable to read mask-enable register, get triggered current failed." );

        is_data_ready = mske.bits.conversion_ready_flag;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(INA226_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (INA226_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (is_data_ready == false);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_CURRENT, &sig), TAG, "unable to read current, get triggered current failed" );

    *current = (float)sig * handle->current_lsb;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ina226_get_power(ina226_handle_t handle, float *const power) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && power );

    uint16_t sig;

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ina226_i2c_read_word_from(handle, INA226_REG_POWER, &sig), TAG, "read bus voltage failed" );

    *power = (float)sig * handle->current_lsb * 25;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ina226_get_mode(ina226_handle_t handle, ina226_operating_modes_t *const mode) {
    ina226_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( ina226_get_configuration_register(handle, &config), TAG, "read configuration register failed" );

    /* set mode */
    *mode = config.bits.operating_mode;

    return ESP_OK;
}

esp_err_t ina226_set_mode(ina226_handle_t handle, const ina226_operating_modes_t mode) {
    ina226_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( ina226_get_configuration_register(handle, &config), TAG, "read configuration register failed" );

    /* set mode */
    config.bits.operating_mode = mode;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( ina226_set_configuration_register(handle, config), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t ina226_reset(ina226_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ina226_config_register_t config;

    config.bits.reset_enabled = true;

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR(ina226_set_configuration_register(handle, config), TAG, "unable to write configuration register, reset failed");

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(INA226_RESET_DELAY_MS));

    /* attempt to configure device */
    ESP_RETURN_ON_ERROR(ina226_get_configuration_register(handle, &config), TAG, "unable to read configuration register, reset failed");

    config.bits.operating_mode      = handle->dev_config.operating_mode;
    config.bits.averaging_mode      = handle->dev_config.averaging_mode;
    config.bits.bus_volt_conv_time  = handle->dev_config.bus_voltage_conv_time;
    config.bits.shun_volt_conv_time = handle->dev_config.shunt_voltage_conv_time;

    ESP_RETURN_ON_ERROR(ina226_set_configuration_register(handle, config), TAG, "unable to write configuration register, reset failed");

    /* attempt to write calibration factor */
    ESP_RETURN_ON_ERROR(ina226_calibrate(handle, handle->dev_config.max_current, handle->dev_config.shunt_resistance), TAG, "unable to calibrate device, reset failed");

    return ESP_OK;
}

esp_err_t ina226_remove(ina226_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t ina226_delete(ina226_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( ina226_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* ina226_get_fw_version(void) {
    return (char *)INA226_FW_VERSION_STR;
}

int32_t ina226_get_fw_version_number(void) {
    return INA226_FW_VERSION_INT32;
}
