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
 * @file ak8976.c
 *
 * ESP-IDF driver for AK8975 3-axis electronic compass
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ak8975.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * AK8975 definitions
*/
#define AK8975_REG_DEVICE_ID_R          UINT8_C(0x00)   //!< ak8975 I2C device identifier
#define AK8975_REG_INFO_R               UINT8_C(0x01)   //!< ak8975 I2C device information register
#define AK8975_REG_STATUS_1_R           UINT8_C(0x02)   //!< ak8975 I2C status 1 register
#define AK8975_REG_HXL_DATA_R           UINT8_C(0x03)   //!< ak8975 I2C x-axis low-byte data register
#define AK8975_REG_HXH_DATA_R           UINT8_C(0x04)   //!< ak8975 I2C x-axis high-byte data register
#define AK8975_REG_HYL_DATA_R           UINT8_C(0x05)   //!< ak8975 I2C y-axis low-byte data register
#define AK8975_REG_HYH_DATA_R           UINT8_C(0x06)   //!< ak8975 I2C y-axis high-byte data register
#define AK8975_REG_HZL_DATA_R           UINT8_C(0x07)   //!< ak8975 I2C z-axis low-byte data register
#define AK8975_REG_HZH_DATA_R           UINT8_C(0x08)   //!< ak8975 I2C z-axis high-byte data register
#define AK8975_REG_STATUS_2_R           UINT8_C(0x09)   //!< ak8975 I2C status 2 register
#define AK8975_REG_CONTROL_RW           UINT8_C(0x0a)   //!< ak8975 I2C control register
#define AK8975_REG_SELF_TEST_RW         UINT8_C(0x0c)   //!< ak8975 I2C self-test register
#define AK8975_REG_I2C_DISABLE_RW       UINT8_C(0x0f)   //!< ak8975 I2C disable i2c register
#define AK8975_REG_ASAX_VALUE_R         UINT8_C(0x10)   //!< ak8975 I2C x-axys sensitivity adjustment value register
#define AK8975_REG_ASAY_VALUE_R         UINT8_C(0x11)   //!< ak8975 I2C y-axys sensitivity adjustment value register
#define AK8975_REG_ASAZ_VALUE_R         UINT8_C(0x12)   //!< ak8975 I2C z-axys sensitivity adjustment value register

#define AK8975_DEVICE_ID                UINT8_C(0x48)   //!< ak8975 I2C device identifier (static)

#define AK8975_POWERUP_DELAY_MS         UINT16_C(50)
#define AK8975_APPSTART_DELAY_MS        UINT16_C(10)
#define AK8975_CMD_DELAY_MS             UINT16_C(5)     //!< ak8975 100us when mode is changed
#define AK8975_DATA_READY_DELAY_MS      UINT16_C(1)     //!< ak8975 1ms when checking data ready in a loop
#define AK8975_DATA_POLL_TIMEOUT_MS     UINT16_C(100)   //!< ak8975 9ms max for single measurement
#define AK8975_TX_RX_DELAY_MS           UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "ak8975";

/*
* functions and subroutines
*/

/**
 * @brief Gets the sensitivity adjusted axis value.
 * 
 * @param asa AK8975 sensitivity adjustment value.
 * @param value AK8975 uncompensated magnetic axis value.
 * @return float Adjusted magnetic axis value.
 */
static inline float ak8975_get_sensitivity_adjusted_axis(const uint8_t asa, const int16_t value) {
    /* see datasheet for details: section 8.3.11 */
    return (float)value*((((float)asa-128.0f)*0.5f/128.0f)+1.0f);
}

/**
 * @brief Gets the sensitivity adjusted axes values.
 * 
 * @param handle AK8975 device handle.
 * @param axes_data AK8975 uncompensated magnetic axes data.
 * @param magnetic_axes_data AK8975 magnetic axes data with sensitivity adjustments applied.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_get_sensitivity_adjusted_axes(ak8975_handle_t handle, const ak8975_axes_data_t axes_data, ak8975_magnetic_axes_data_t *const magnetic_axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* apply sensitivity adjustments */
    magnetic_axes_data->x_axis = ak8975_get_sensitivity_adjusted_axis(handle->asa_x_value, axes_data.x_axis.value);
    magnetic_axes_data->y_axis = ak8975_get_sensitivity_adjusted_axis(handle->asa_y_value, axes_data.y_axis.value);
    magnetic_axes_data->z_axis = ak8975_get_sensitivity_adjusted_axis(handle->asa_z_value, axes_data.z_axis.value);

    return ESP_OK;
}

/**
 * @brief AK8975 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle AK8975 device handle.
 * @param reg_addr AK8975 register address to read from.
 * @param buffer AK8975 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_i2c_read_from(ak8975_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "ak8975_i2c_read_byte_from failed" );
    
    return ESP_OK;
}

/**
 * @brief AK8975 I2C read byte from register address transaction.
 * 
 * @param handle AK8975 device handle.
 * @param reg_addr AK8975 register address to read from.
 * @param byte AK8975 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_i2c_read_byte_from(ak8975_handle_t handle, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ak8975_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief AK8975 I2C write byte to register address transaction.
 * 
 * @param handle AK8975 device handle.
 * @param reg_addr AK8975 register address to write to.
 * @param byte AK8975 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_i2c_write_byte_to(ak8975_handle_t handle, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Writes the operating mode to AK8975.
 * 
 * @param handle AK8975 device handle.
 * @param mode AK8975 operating mode setting.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_set_mode(ak8975_handle_t handle, const ak8975_operating_modes_t mode) {
    ak8975_control_register_t ctrl;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ak8975_get_control_register(handle, &ctrl), TAG, "set mode failed" );

    /* set operating mode */
    ctrl.bits.mode = mode;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ak8975_set_control_register(handle, ctrl), TAG, "set mode failed" );

    return ESP_OK;
}

/**
 * @brief Writes the self-test control state (true or false) to AK8975.
 * 
 * @param handle AK8975 device handle.
 * @param state Self-test control state setting, true to enable self-test.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_set_selftest(ak8975_handle_t handle, const bool state) {
    ak8975_selftest_control_register_t selftest;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_get_selftest_control_register(handle, &selftest), TAG, "set mode failed" );

    /* set self-test control register */
    selftest.bits.selftest_state = state;
    selftest.bits.reserved1 = 0;
    selftest.bits.reserved2 = 0;

    /* attempt i2c self test control register write transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_write_byte_to(handle, AK8975_REG_SELF_TEST_RW, selftest.reg), TAG, "write self-test control register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t ak8975_get_device_id_register(ak8975_handle_t handle, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_DEVICE_ID_R, reg), TAG, "ak8975 device identifier register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t ak8975_get_device_information_register(ak8975_handle_t handle, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_INFO_R, reg), TAG, "ak8975 device information register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads uncompensated magnetic axes data from AK8975.
 * 
 * @param handle AK8975 device handle.
 * @param axes_data AK8975 uncompensated magnetic axes data (X, Y, Z).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ak8975_get_fixed_magnetic_axes(ak8975_handle_t handle, ak8975_axes_data_t *const axes_data) {
    esp_err_t ret           = ESP_OK;
    uint64_t  start_time    = esp_timer_get_time();
    bool      data_is_ready = false;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to wait until data is available */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ak8975_get_data_status(handle, &data_is_ready), err, TAG, "data ready read failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AK8975_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (AK8975_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);
    
    bit48_uint8_buffer_t rx = { 0 };

    /* 6-byte i2c read transaction */
    ESP_GOTO_ON_ERROR( ak8975_i2c_read_from(handle, AK8975_REG_HXL_DATA_R, rx, BIT48_UINT8_BUFFER_SIZE), err, TAG, "read axes (x, y, z) bytes failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));
    
    /* set axes data */
    axes_data->x_axis.bytes[0] = rx[0];
    axes_data->x_axis.bytes[1] = rx[1];
    axes_data->y_axis.bytes[0] = rx[2];
    axes_data->y_axis.bytes[1] = rx[3];
    axes_data->z_axis.bytes[0] = rx[4];
    axes_data->z_axis.bytes[1] = rx[5];

    /* debug purposes */
    ESP_LOGW(TAG, "ak8975 x-axis data register: byte[0] %02x | byte[1] %02x | value %d", axes_data->x_axis.bytes[0], axes_data->x_axis.bytes[1], axes_data->x_axis.value);
    ESP_LOGW(TAG, "ak8975 y-axis data register: byte[0] %02x | byte[1] %02x | value %d", axes_data->y_axis.bytes[0], axes_data->y_axis.bytes[1], axes_data->y_axis.value);
    ESP_LOGW(TAG, "ak8975 z-axis data register: byte[0] %02x | byte[1] %02x | value %d", axes_data->z_axis.bytes[0], axes_data->z_axis.bytes[1], axes_data->z_axis.value);
    
    /* validate data status */
    bool data_error    = false;
    bool data_overflow = false;

    ak8975_get_error_status(handle, &data_error);
    ESP_LOGE(TAG, "ak8975 data error    %s", data_error ? "true" : "false");

    ak8975_get_overflow_status(handle, &data_overflow);
    ESP_LOGE(TAG, "ak8975 data overflow %s", data_overflow ? "true" : "false");

    if(data_error == true) return ESP_ERR_INVALID_RESPONSE;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ak8975_get_control_register(ak8975_handle_t handle, ak8975_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_CONTROL_RW, &reg->reg), TAG, "ak8975 read control register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ak8975_set_control_register(ak8975_handle_t handle, const ak8975_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    ak8975_control_register_t ctrl = { .reg = reg.reg };
    ctrl.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_write_byte_to(handle, AK8975_REG_CONTROL_RW, ctrl.reg), TAG, "write control register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ak8975_get_selftest_control_register(ak8975_handle_t handle, ak8975_selftest_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_SELF_TEST_RW, &reg->reg), TAG, "read self-test control register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ak8975_get_status1_register(ak8975_handle_t handle, ak8975_status1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_STATUS_1_R, &reg->reg), TAG, "read status 1 register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ak8975_get_status2_register(ak8975_handle_t handle, ak8975_status2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_STATUS_2_R, &reg->reg), TAG, "read status 2 register failed" );
    
    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ak8975_get_asa_registers(ak8975_handle_t handle, uint8_t *const asa_x_reg, uint8_t *const asa_y_reg, uint8_t *const asa_z_reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to set operating mode */
    ESP_RETURN_ON_ERROR( ak8975_set_mode(handle, AK8975_OPMODE_FUSE_ROM), TAG, "fuse rom access mode failed" );

    /* attempt i2c read transactions */
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_ASAX_VALUE_R, asa_x_reg), TAG, "read asax register failed" );
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_ASAY_VALUE_R, asa_y_reg), TAG, "read asay register failed" );
    ESP_RETURN_ON_ERROR( ak8975_i2c_read_byte_from(handle, AK8975_REG_ASAZ_VALUE_R, asa_z_reg), TAG, "read asaz register failed" );

    /* attempt to set operating mode */
    ESP_RETURN_ON_ERROR( ak8975_set_mode(handle, AK8975_OPMODE_POWER_DOWN), TAG, "power down mode failed" );
    
    return ESP_OK;
}

float ak8975_convert_to_heading(ak8975_magnetic_axes_data_t axes_data) {
    float heading = 0;

    /* honeywell application note AN-203 */
    if(axes_data.y_axis > 0.0f) heading = 90.0f - atanf(axes_data.x_axis/axes_data.y_axis * 180.0f / (float)M_PI);
    if(axes_data.y_axis < 0.0f) heading = 270.0f - atanf(axes_data.x_axis/axes_data.y_axis * 180.0f / (float)M_PI);
    if(axes_data.y_axis == 0.0f && axes_data.x_axis < 0.0f) heading = 180.0f;
    if(axes_data.y_axis == 0.0f && axes_data.x_axis > 0.0f) heading = 180.0f;

    return heading;
}

esp_err_t ak8975_init(i2c_master_bus_handle_t master_handle, const ak8975_config_t *ak8975_config, ak8975_handle_t *ak8975_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && ak8975_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ak8975_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ak8975 device handle initialization failed", ak8975_config->i2c_address);

    /* validate memory availability for handle */
    ak8975_handle_t out_handle;
    out_handle = (ak8975_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ak8975 device");

    /* copy configuration */
    out_handle->dev_config = *ak8975_config;

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

    /* attempt to read device identifier */
    ESP_GOTO_ON_ERROR( ak8975_get_device_id_register(out_handle, &out_handle->device_id), err_handle, TAG, "read device identifier register failed" );

    /* attempt to read device information */
    ESP_GOTO_ON_ERROR( ak8975_get_device_information_register(out_handle, &out_handle->device_info), err_handle, TAG, "read device information register failed" );

    /* attempt to read asa values */
    ESP_GOTO_ON_ERROR( ak8975_get_asa_registers(out_handle, &out_handle->asa_x_value, &out_handle->asa_y_value, &out_handle->asa_z_value), err_handle, TAG, "read asa registers failed" );

    /* set device handle */
    *ak8975_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AK8975_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t ak8975_get_magnetic_axes(ak8975_handle_t handle, ak8975_magnetic_axes_data_t *const axes_data) {
    esp_err_t           ret = ESP_OK;
    ak8975_axes_data_t  data;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to set operating mode */
    ESP_GOTO_ON_ERROR( ak8975_set_mode(handle, AK8975_OPMODE_SINGLE_MEAS), err, TAG, "single measurement mode failed" );

    /* attempt to read uncompensated magnetic axes data */
    ESP_GOTO_ON_ERROR( ak8975_get_fixed_magnetic_axes(handle, &data), err, TAG, "read axes data failed" );

    /* attempt to process compass axes sensitivity adjustments */
    ESP_GOTO_ON_ERROR( ak8975_get_sensitivity_adjusted_axes(handle, data, axes_data), err, TAG, "magnetic axes processing failed" );

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ak8975_selftest(ak8975_handle_t handle, ak8975_magnetic_axes_data_t *const axes_data) {
    esp_err_t           ret = ESP_OK;
    ak8975_axes_data_t  data;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to set operating mode */
    ESP_GOTO_ON_ERROR( ak8975_set_mode(handle, AK8975_OPMODE_POWER_DOWN), err, TAG, "power down mode failed" );

    /* attempt to enable self test control */
    ESP_GOTO_ON_ERROR( ak8975_set_selftest(handle, true), err, TAG, "self test enabled failed" );
    
    /* attempt to set operating mode */
    ESP_GOTO_ON_ERROR( ak8975_set_mode(handle, AK8975_OPMODE_SELF_TEST), err, TAG, "self test mode failed" );

    /* attempt to read uncompensated magnetic axes data */
    ESP_GOTO_ON_ERROR( ak8975_get_fixed_magnetic_axes(handle, &data), err, TAG, "read axes data failed" );

    /* attempt to disable self test control */
    ESP_GOTO_ON_ERROR( ak8975_set_selftest(handle, false), err, TAG, "self test disabled failed" );

    /* attempt to process compass axes sensitivity adjustments */
    ESP_GOTO_ON_ERROR( ak8975_get_sensitivity_adjusted_axes(handle, data, axes_data), err, TAG, "magnetic axes processing failed" );

    return ESP_OK;

    err:
        /* attempt to disable self test control */
        ak8975_set_selftest(handle, false);

        return ret;
}

esp_err_t ak8975_get_data_status(ak8975_handle_t handle, bool *const ready) {
    ak8975_status1_register_t sts;

    ESP_ARG_CHECK( handle );

    /* attempt to read status 1 register */
    ESP_RETURN_ON_ERROR( ak8975_get_status1_register(handle, &sts), TAG, "read status 1 register (data ready state) failed" );

    /* set ready state */
    *ready = sts.bits.data_ready;

    return ESP_OK;
}

esp_err_t ak8975_get_error_status(ak8975_handle_t handle, bool *const error) {
    ak8975_status2_register_t sts;

    ESP_ARG_CHECK( handle );

    /* attempt to read status 2 register */
    ESP_RETURN_ON_ERROR( ak8975_get_status2_register(handle, &sts), TAG, "read status 2 register failed" );

    /* set error state */
    *error = sts.bits.data_error;

    return ESP_OK;
}

esp_err_t ak8975_get_overflow_status(ak8975_handle_t handle, bool *const overflow) {
    ak8975_status2_register_t sts;

    ESP_ARG_CHECK( handle );

    /* attempt to read status 2 register */
    ESP_RETURN_ON_ERROR( ak8975_get_status2_register(handle, &sts), TAG, "read status 2 register failed" );

    /* set overflow state */
    *overflow = sts.bits.sensor_overflow;

    return ESP_OK;
}

esp_err_t ak8975_power_down(ak8975_handle_t handle) {
    ESP_ARG_CHECK( handle );

    /* attempt to set operating mode */
    ESP_RETURN_ON_ERROR( ak8975_set_mode(handle, AK8975_OPMODE_POWER_DOWN), TAG, "power down mode failed" );

    return ESP_OK;
}

esp_err_t ak8975_remove(ak8975_handle_t handle) {
    ESP_ARG_CHECK( handle );

    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t ak8975_delete(ak8975_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( ak8975_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}
