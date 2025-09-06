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
 * @file hmc5883l.c
 *
 * ESP-IDF driver for HMC5883L digital compass sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/hmc5883l.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * HMC5883L definitions
*/

#define HMC5883L_REG_CONFIG_A               UINT8_C(0x00)
#define HMC5883L_REG_CONFIG_B               UINT8_C(0x01)
#define HMC5883L_REG_MODE                   UINT8_C(0x02)
#define HMC5883L_REG_DATA_OUT_X_MSB         UINT8_C(0x03)
#define HMC5883L_REG_DATA_OUT_X_LSB         UINT8_C(0x04)
#define HMC5883L_REG_DATA_OUT_Z_MSB         UINT8_C(0x05)
#define HMC5883L_REG_DATA_OUT_Z_LSB         UINT8_C(0x06)
#define HMC5883L_REG_DATA_OUT_Y_MSB         UINT8_C(0x07)
#define HMC5883L_REG_DATA_OUT_Y_LSB         UINT8_C(0x08)
#define HMC5883L_REG_STATUS                 UINT8_C(0x09)
#define HMC5883L_REG_IDENT_A                UINT8_C(0x0a)
#define HMC5883L_REG_IDENT_B                UINT8_C(0x0b)
#define HMC5883L_REG_IDENT_C                UINT8_C(0x0c)

#define HMC5883L_DEV_ID                     UINT32_C(0x00333448)    //!< Chip ID, "H43"

#define HMC5883L_XY_EXCITATION              (1160)  // The magnetic field excitation in X and Y direction during Self Test (Calibration)
#define HMC5883L_Z_EXCITATION               (1080)  // The magnetic field excitation in Z direction during Self Test (Calibration)

#define HMC5883L_DATA_READY_DELAY_MS        UINT16_C(1)
#define HMC5883L_DATA_POLL_TIMEOUT_MS       UINT16_C(50)
#define HMC5883L_POWERUP_DELAY_MS           UINT16_C(100)
#define HMC5883L_APPSTART_DELAY_MS          UINT16_C(20)
#define HMC5883L_RESET_DELAY_MS             UINT16_C(50)
#define HMC5883L_CMD_DELAY_MS               UINT16_C(5)
#define HMC5883L_TX_RX_DELAY_MS             UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief HMC5883L device descriptor structure definition.
 */
typedef struct hmc5883l_device_s {
    hmc5883l_config_t                       config;
    i2c_master_dev_handle_t                 i2c_handle;     /*!< I2C device handle */
    //i2c_hmc5883l_configuration1_register_t  config1_reg;
    //i2c_hmc5883l_configuration2_register_t  config2_reg;
    //i2c_hmc5883l_mode_register_t            mode_reg;
    //i2c_hmc5883l_status_register_t          status_reg;
    uint32_t                                dev_id;
    float                                   declination;        /*!< magnetic declination angle http://www.magnetic-declination.com/ */
    bool                                    gain_calibrated;
    bool                                    offset_calibrated;
    hmc5883l_offset_axes_data_t         offset_axes;
    hmc5883l_gain_error_axes_data_t     gain_error_axes;
} hmc5883l_device_t;

/*
* static constant declarations
*/
static const char *TAG = "hmc5883l";

/* Gain sensitivity values for HMC5883L */
static const float hmc5883l_gain_values [] = {
    [HMC5883L_GAIN_1370] = 0.73f,
    [HMC5883L_GAIN_1090] = 0.92f,
    [HMC5883L_GAIN_820]  = 1.22f,
    [HMC5883L_GAIN_660]  = 1.52f,
    [HMC5883L_GAIN_440]  = 2.27f,
    [HMC5883L_GAIN_390]  = 2.56f,
    [HMC5883L_GAIN_330]  = 3.03f,
    [HMC5883L_GAIN_230]  = 4.35f
};

/*
* functions and subroutines
*/

/**
 * @brief HMC5883L I2C HAL write byte to register address transaction.
 * 
 * @param device HMC5883L device descriptor.
 * @param reg_addr HMC5883L register address to write to.
 * @param byte HMC5883L write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_write_byte_to(hmc5883l_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read byte from register address transaction.
 * 
 * @param device HMC5883L device descriptor.
 * @param reg_addr HMC5883L register address to read from.
 * @param byte HMC5883L read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_read_byte_from(hmc5883l_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "hmc5883l_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device HMC5883L device descriptor.
 * @param reg_addr HMC5883L register address to read from.
 * @param buffer HMC5883L read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_read_from(hmc5883l_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "hmc5883l_i2c_read_from failed" );

    return ESP_OK;
}

esp_err_t hmc5883l_get_identification_register(hmc5883l_handle_t handle, uint32_t *const reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* hmc5883l attempt to read device identification */
    uint8_t ident_a; uint8_t ident_b; uint8_t ident_c;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_IDENT_A, &ident_a), TAG, "read register IDENT_A failed");
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_IDENT_B, &ident_b), TAG, "read register IDENT_B failed");
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_IDENT_C, &ident_c), TAG, "read register IDENT_C failed");

    /* construct device identification */
    *reg = ident_a | (ident_b << 8) | (ident_c << 16);

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_get_configuration1_register(hmc5883l_handle_t handle, hmc5883l_configuration1_register_t *const reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_CONFIG_A, &reg->reg), TAG, "read configuration 1 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_set_configuration1_register(hmc5883l_handle_t handle, const hmc5883l_configuration1_register_t reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    hmc5883l_configuration1_register_t config1 = { .reg = reg.reg };

    /* set register reserved settings */
    config1.bits.reserved = 0;

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_write_byte_to(dev, HMC5883L_REG_CONFIG_A, config1.reg), TAG, "write configuration 1 register failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_get_configuration2_register(hmc5883l_handle_t handle, hmc5883l_configuration2_register_t *const reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_CONFIG_B, &reg->reg), TAG, "read configuration 2 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_set_configuration2_register(hmc5883l_handle_t handle, const hmc5883l_configuration2_register_t reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    hmc5883l_configuration2_register_t config2 = { .reg = reg.reg };

    /* set register reserved settings */
    config2.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_write_byte_to(dev, HMC5883L_REG_CONFIG_B, config2.reg), TAG, "write configuration 2 register failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_get_mode_register(hmc5883l_handle_t handle, hmc5883l_mode_register_t *const reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_MODE, &reg->reg), TAG, "read mode register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_set_mode_register(hmc5883l_handle_t handle, const hmc5883l_mode_register_t reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    hmc5883l_mode_register_t mode = { .reg = reg.reg };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_write_byte_to(dev, HMC5883L_REG_MODE, mode.reg), TAG, "write mode register failed");

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_get_status_register(hmc5883l_handle_t handle, hmc5883l_status_register_t *const reg) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(dev, HMC5883L_REG_STATUS, &reg->reg), TAG, "read status register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t hmc5883l_init(i2c_master_bus_handle_t master_handle, const hmc5883l_config_t *hmc5883l_config, hmc5883l_handle_t *hmc5883l_handle) {
    hmc5883l_configuration1_register_t config1_reg;
    hmc5883l_configuration2_register_t config2_reg;
    hmc5883l_mode_register_t           mode_reg;

    /* validate arguments */
    ESP_ARG_CHECK( master_handle && hmc5883l_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, hmc5883l_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, hmc5883l device handle initialization failed", hmc5883l_config->i2c_address);

    /* validate memory availability for handle */
    hmc5883l_device_t* dev = (hmc5883l_device_t*)calloc(1, sizeof(hmc5883l_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hmc5883l device");

    /* copy configuration */
    dev->config = *hmc5883l_config;

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

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    /* attempt to read registers */
    ESP_GOTO_ON_ERROR(hmc5883l_get_configuration1_register((hmc5883l_handle_t)dev, &config1_reg), err_handle, TAG, "read configuration 1 register failed");

    ESP_GOTO_ON_ERROR(hmc5883l_get_configuration2_register((hmc5883l_handle_t)dev, &config2_reg), err_handle, TAG, "read configuration 2 register failed");

    ESP_GOTO_ON_ERROR(hmc5883l_get_mode_register((hmc5883l_handle_t)dev, &mode_reg), err_handle, TAG, "read mode register failed");

    ESP_GOTO_ON_ERROR(hmc5883l_get_identification_register((hmc5883l_handle_t)dev, &dev->dev_id), err_handle, TAG, "read identification register failed");
    
    /* validate device identifier */
    if (dev->dev_id != HMC5883L_DEV_ID) {
        ESP_LOGE(TAG, "Unknown ID: %lu (device) != %lu (reference)", dev->dev_id, HMC5883L_DEV_ID);
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_FOUND, err_handle, TAG, "i2c hmc5883l device identifier validation failed");
    }

    /* attempt to write configuration 1 register */
    config1_reg.bits.bias       = hmc5883l_config->bias;
    config1_reg.bits.data_rate  = hmc5883l_config->rate;
    config1_reg.bits.sample_avg = hmc5883l_config->sample;
    ESP_GOTO_ON_ERROR(hmc5883l_set_configuration1_register((hmc5883l_handle_t)dev, config1_reg), err_handle, TAG, "write configuration 1 register failed");

    /* attempt to write configuration 2 register */
    config2_reg.bits.gain       = hmc5883l_config->gain;
    ESP_GOTO_ON_ERROR(hmc5883l_set_configuration2_register((hmc5883l_handle_t)dev, config2_reg), err_handle, TAG, "write configuration 2 register failed");

    /* attempt to write mode register */
    mode_reg.bits.mode          = hmc5883l_config->mode;
    ESP_GOTO_ON_ERROR(hmc5883l_set_mode_register((hmc5883l_handle_t)dev, mode_reg), err_handle, TAG, "write mode register failed");

    /* copy configuration */
    dev->declination     = hmc5883l_config->declination;

    /* set device handle */
    *hmc5883l_handle = (hmc5883l_handle_t)dev;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle_t handle, hmc5883l_axes_data_t *const axes_data) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && axes_data );

    /* initialize local variables */
    esp_err_t    ret       = ESP_OK;
    uint64_t     start     = esp_timer_get_time();
    bool         is_ready  = false;
    bool         is_locked = false;
    bit48_uint8_buffer_t rx = { 0 };

    /* poll data status until data is ready or timeout condition is asserted */
    do {
        /* read data status */
        ESP_GOTO_ON_ERROR( hmc5883l_get_data_status(handle, &is_ready, &is_locked), err, TAG, "data ready ready for get fixed measurement failed" );

        /* delay task before i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(HMC5883L_DATA_READY_DELAY_MS));

        if (ESP_TIMEOUT_CHECK(start, HMC5883L_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (!is_ready);
    
    /* attempt i2c read transaction */
    ESP_GOTO_ON_ERROR( hmc5883l_i2c_read_from(dev, HMC5883L_REG_DATA_OUT_X_MSB, rx, BIT48_UINT8_BUFFER_SIZE), err, TAG, "read uncompensated compass data for get fixed measurement failed" );

    /* convert 2-byte data to int16 data type - 2s complement */
    axes_data->x_axis = (int16_t)(rx[0] << 8) | rx[1];
    axes_data->z_axis = (int16_t)(rx[4] << 8) | rx[5];
    axes_data->y_axis = (int16_t)(rx[2] << 8) | rx[3];

    //ESP_LOGW(TAG, "Raw X-Axis: %d", data->x_axis);
    //ESP_LOGW(TAG, "Raw Y-Axis: %d", data->y_axis);
    //ESP_LOGW(TAG, "Raw Z-Axis: %d", data->z_axis);

    return ESP_OK;

    err:
        return ret;
}

esp_err_t hmc5883l_get_magnetic_axes(hmc5883l_handle_t handle, hmc5883l_magnetic_axes_data_t *const axes_data) {
    hmc5883l_configuration2_register_t config2;
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && axes_data );

    /* attempt i2c read transaction */
    ESP_ERROR_CHECK( hmc5883l_get_configuration2_register(handle, &config2) );

    /* set gain sensitivity */
    const float gain_sensitivity = hmc5883l_gain_values[config2.bits.gain];

    /* attempt to read uncompensated magnetic measurements */
    hmc5883l_axes_data_t raw;
    ESP_ERROR_CHECK( hmc5883l_get_fixed_magnetic_axes(handle, &raw) );

    /* handle calibration corrections and compensation factors */
    if(dev->gain_calibrated == true && dev->offset_calibrated == true) {
        axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity * dev->gain_error_axes.x_axis + dev->offset_axes.x_axis;
        axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity * dev->gain_error_axes.y_axis + dev->offset_axes.y_axis;
        axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity * dev->gain_error_axes.z_axis + dev->offset_axes.z_axis;
    } else if(dev->gain_calibrated == true && dev->offset_calibrated == false) {
        axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity * dev->gain_error_axes.x_axis;
        axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity * dev->gain_error_axes.y_axis;
        axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity * dev->gain_error_axes.z_axis;
    } else if(dev->gain_calibrated == false && dev->offset_calibrated == true) {
        axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity + dev->offset_axes.x_axis;
        axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity + dev->offset_axes.y_axis;
        axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity + dev->offset_axes.z_axis;
    } else {
        axes_data->x_axis  = (float)raw.x_axis * gain_sensitivity;
        axes_data->y_axis  = (float)raw.y_axis * gain_sensitivity;
        axes_data->z_axis  = (float)raw.z_axis * gain_sensitivity;
    }

    axes_data->heading = atan2f(0.0f - axes_data->y_axis, axes_data->x_axis) * 180.0f / (float)M_PI;
    //compass_axes_data->heading = atan2(compass_axes_data->y_axis, compass_axes_data->x_axis);
    //compass_axes_data->heading += (hmc5883l_handle->declination * 180/M_PI);
     // Correct for when signs are reversed.
    //if(compass_axes_data->heading < 0) compass_axes_data->heading += 2*M_PI;
    // Check for wrap due to addition of declination.
    //if(compass_axes_data->heading > 2*M_PI) compass_axes_data->heading -= 2*M_PI;
    // Convert radians to degrees for readability.
    //compass_axes_data->heading = compass_axes_data->heading * 180/M_PI;

    return ESP_OK;
}

// https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/blob/master/Core/Compass_header_example_ver_0_2/compass.cpp

esp_err_t hmc5883l_get_calibrated_offsets(hmc5883l_handle_t handle, const hmc5883l_calibration_options_t option) {
    hmc5883l_configuration1_register_t config1;
    hmc5883l_configuration2_register_t config2;
    hmc5883l_axes_data_t            raw_axes;
    hmc5883l_magnetic_axes_data_t   scaled_axes;
    hmc5883l_gain_error_axes_data_t gain_error_axes;
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* attempt i2c read transaction */
    ESP_ERROR_CHECK( hmc5883l_get_configuration2_register(handle, &config2) );

    /* copy user configured settings */
    hmc5883l_sample_averages_t  sample  = config1.bits.sample_avg;             
    hmc5883l_data_rates_t       rate    = config1.bits.data_rate;         
    hmc5883l_biases_t           bias    = config1.bits.bias;                         
    float              gain_sensitivity = hmc5883l_gain_values[config2.bits.gain];

    /* handle calibration option */
    if(option == HMC5883L_CAL_GAIN_DIFF || option == HMC5883L_CAL_BOTH) {
        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Gain");

        // configuring the control register for positive bias mode
        config1.bits.sample_avg = HMC5883L_SAMPLE_8;
        config1.bits.data_rate  = HMC5883L_DATA_RATE_15_00;
        config1.bits.bias       = HMC5883L_BIAS_POSITIVE;
        ESP_RETURN_ON_ERROR(hmc5883l_set_configuration1_register(handle, config1), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( hmc5883l_get_fixed_magnetic_axes(handle, &raw_axes) );

        // reading the positive biased data
        //while(raw_axes.x_axis<200 || raw_axes.y_axis<200 || raw_axes.z_axis<200){   // making sure the data is with positive biased
        while(raw_axes.x_axis<50 || raw_axes.y_axis<50 || raw_axes.z_axis<50){
            ESP_ERROR_CHECK( hmc5883l_get_fixed_magnetic_axes(handle, &raw_axes) );
        }

        scaled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
        scaled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
        scaled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

        // offset = 1160 - data positive
        gain_error_axes.x_axis = (float)HMC5883L_XY_EXCITATION/scaled_axes.x_axis;
        gain_error_axes.y_axis = (float)HMC5883L_XY_EXCITATION/scaled_axes.y_axis;
        gain_error_axes.z_axis = (float)HMC5883L_Z_EXCITATION/scaled_axes.z_axis;

        // configuring the control register for negative bias mode
        config1.bits.sample_avg = HMC5883L_SAMPLE_8;
        config1.bits.data_rate  = HMC5883L_DATA_RATE_15_00;
        config1.bits.bias       = HMC5883L_BIAS_NEGATIVE;
        ESP_RETURN_ON_ERROR(hmc5883l_set_configuration1_register(handle, config1), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( hmc5883l_get_fixed_magnetic_axes(handle, &raw_axes) );

        // reading the negative biased data
        //while(raw_axes.x_axis>-200 || raw_axes.y_axis>-200 || raw_axes.z_axis>-200){   // making sure the data is with negative biased
        while(raw_axes.x_axis>-50 || raw_axes.y_axis>-50 || raw_axes.z_axis>-50){
            ESP_ERROR_CHECK( hmc5883l_get_fixed_magnetic_axes(handle, &raw_axes) );
        }

        scaled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
        scaled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
        scaled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

        // taking the average of the offsets
        gain_error_axes.x_axis = (float)((HMC5883L_XY_EXCITATION/fabs(scaled_axes.x_axis))+gain_error_axes.x_axis)/2;
        gain_error_axes.y_axis = (float)((HMC5883L_XY_EXCITATION/fabs(scaled_axes.y_axis))+gain_error_axes.y_axis)/2;
        gain_error_axes.z_axis = (float)((HMC5883L_Z_EXCITATION/fabs(scaled_axes.z_axis))+gain_error_axes.z_axis)/2;

        dev->gain_calibrated = true;
        dev->gain_error_axes = gain_error_axes;

        ESP_LOGW(TAG, "Gain Offset X-Axis: %f", gain_error_axes.x_axis);
        ESP_LOGW(TAG, "Gain Offset Y-Axis: %f", gain_error_axes.y_axis);
        ESP_LOGW(TAG, "Gain Offset Z-Axis: %f", gain_error_axes.z_axis);
    }

    // configuring the control register for normal mode
    config1.bits.sample_avg = HMC5883L_SAMPLE_8;
    config1.bits.data_rate  = HMC5883L_DATA_RATE_15_00;
    config1.bits.bias       = HMC5883L_BIAS_NORMAL;
    ESP_RETURN_ON_ERROR(hmc5883l_set_configuration1_register(handle, config1), TAG, "write configuration 1 register failed");

    if(option == HMC5883L_CAL_AXES_MEAN || option == HMC5883L_CAL_BOTH) {
        hmc5883l_offset_axes_data_t     offset_axes = { .x_axis = NAN, .y_axis = NAN, .z_axis = NAN };
        hmc5883l_offset_axes_data_t     max_offset_axes = { .x_axis = NAN, .y_axis = NAN, .z_axis = NAN };
        hmc5883l_offset_axes_data_t     min_offset_axes = { .x_axis = NAN, .y_axis = NAN, .z_axis = NAN };
        uint16_t x_count = 0;
        uint16_t y_count = 0;
        uint16_t z_count = 0;
        bool x_zero = false;
        bool y_zero = false;
        bool z_zero = false;

        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Offset");
        ESP_LOGW(TAG, "Please rotate the magnetometer 2 or 3 times in complete circles within one minute .............");

        while (x_count < 3 || y_count < 3 || z_count < 3) {
            ESP_ERROR_CHECK( hmc5883l_get_fixed_magnetic_axes(handle, &raw_axes) );
            scaled_axes.x_axis  = (float)raw_axes.x_axis * gain_sensitivity;
            scaled_axes.y_axis  = (float)raw_axes.y_axis * gain_sensitivity;
            scaled_axes.z_axis  = (float)raw_axes.z_axis * gain_sensitivity;

            if ((fabs(scaled_axes.x_axis) > 100) || (fabs(scaled_axes.y_axis) > 100) || (fabs(scaled_axes.z_axis) > 100)) {
                continue;
            }

            if (min_offset_axes.x_axis > scaled_axes.x_axis) {
                min_offset_axes.x_axis = scaled_axes.x_axis;
            } else if (max_offset_axes.x_axis < scaled_axes.x_axis) {
                max_offset_axes.x_axis = scaled_axes.x_axis;
            }

            if (min_offset_axes.y_axis > scaled_axes.y_axis) {
                min_offset_axes.y_axis = scaled_axes.y_axis;
            } else if (max_offset_axes.y_axis < scaled_axes.y_axis) {
                max_offset_axes.y_axis = scaled_axes.y_axis;
            }

            if (min_offset_axes.z_axis > scaled_axes.z_axis) {
                min_offset_axes.z_axis = scaled_axes.z_axis;
            } else if (max_offset_axes.z_axis < scaled_axes.z_axis) {
                max_offset_axes.z_axis = scaled_axes.z_axis;
            }

            if (x_zero) {
                if (fabs(scaled_axes.x_axis) > 50) {
                    x_zero = false;
                    x_count++;
                }
            } else {
                if (fabs(scaled_axes.x_axis) < 40) {
                    x_zero = true;
                }
            }

            if (y_zero) {
                if (fabs(scaled_axes.y_axis) > 50) {
                    y_zero = false;
                    y_count++;
                }
            } else {
                if (fabs(scaled_axes.y_axis) < 40) {
                    y_zero = true;
                }
            }

            if (z_zero) {
                if (fabs(scaled_axes.z_axis) > 50) {
                    z_zero = false;
                    z_count++;
                }
            } else {
                if (fabs(scaled_axes.z_axis) < 40) {
                    z_zero = true;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(30));
        }

        offset_axes.x_axis = (max_offset_axes.x_axis + min_offset_axes.x_axis) / 2;
        offset_axes.y_axis = (max_offset_axes.y_axis + min_offset_axes.y_axis) / 2;
        offset_axes.z_axis = (max_offset_axes.z_axis + min_offset_axes.z_axis) / 2;

        dev->offset_calibrated = true;
        dev->offset_axes = offset_axes;

        ESP_LOGW(TAG, "Offset X-Axis: %f", offset_axes.x_axis);
        ESP_LOGW(TAG, "Offset Y-Axis: %f", offset_axes.y_axis);
        ESP_LOGW(TAG, "Offset Z-Axis: %f", offset_axes.z_axis);
    }

    // configuring the control register to user defined settings
    config1.bits.bias       = bias;
    config1.bits.data_rate  = rate;
    config1.bits.sample_avg = sample;
    ESP_RETURN_ON_ERROR(hmc5883l_set_configuration1_register(handle, config1), TAG, "write configuration 1 register failed");

    return ESP_OK;
}



esp_err_t hmc5883l_get_data_status(hmc5883l_handle_t handle, bool *const ready, bool *const locked) {
    hmc5883l_status_register_t status;

    /* validate arguments */
    ESP_ARG_CHECK( handle && ready && locked );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_status_register(handle, &status) );

    /* set output parameters */
    *ready  = status.bits.data_ready;
    *locked = status.bits.data_locked;

    return ESP_OK;
}

esp_err_t hmc5883l_get_mode(hmc5883l_handle_t handle, hmc5883l_modes_t *const mode) {
    hmc5883l_mode_register_t mode_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_mode_register(handle, &mode_reg) );

    /* set output parameter */
    *mode = mode_reg.bits.mode;

    return ESP_OK;
}

esp_err_t hmc5883l_set_mode(hmc5883l_handle_t handle, const hmc5883l_modes_t mode) {
    hmc5883l_mode_register_t mode_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_mode_register(handle, &mode_reg) );

    /* set register setting */
    mode_reg.bits.mode = mode;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_set_mode_register(handle, mode_reg) );

    return ESP_OK;
}

esp_err_t hmc5883l_get_samples_averaged(hmc5883l_handle_t handle, hmc5883l_sample_averages_t *const sample) {
    hmc5883l_configuration1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* set output parameter */
    *sample = config1.bits.sample_avg;

    return ESP_OK;
}

esp_err_t hmc5883l_set_samples_averaged(hmc5883l_handle_t handle, const hmc5883l_sample_averages_t sample) {
    hmc5883l_configuration1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* set register setting */
    config1.bits.sample_avg = sample;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_set_configuration1_register(handle, config1) );

    return ESP_OK;
}

esp_err_t hmc5883l_get_data_rate(hmc5883l_handle_t handle, hmc5883l_data_rates_t *const rate) {
    hmc5883l_configuration1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* set output parameter */
    *rate = config1.bits.data_rate;

    return ESP_OK;
}

esp_err_t hmc5883l_set_data_rate(hmc5883l_handle_t handle, const hmc5883l_data_rates_t rate) {
    hmc5883l_configuration1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* set register setting */
    config1.bits.data_rate = rate;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_set_configuration1_register(handle, config1) );

    return ESP_OK;
}

esp_err_t hmc5883l_get_bias(hmc5883l_handle_t handle, hmc5883l_biases_t *const bias) {
    hmc5883l_configuration1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* set output parameter */
    *bias = config1.bits.bias;

    return ESP_OK;
}

esp_err_t hmc5883l_set_bias(hmc5883l_handle_t handle, const hmc5883l_biases_t bias) {
    hmc5883l_configuration1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration1_register(handle, &config1) );

    /* set register setting */
    config1.bits.bias = bias;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_set_configuration1_register(handle, config1) );

    return ESP_OK;
}

esp_err_t hmc5883l_get_gain(hmc5883l_handle_t handle, hmc5883l_gains_t *const gain) {
    hmc5883l_configuration2_register_t config2;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration2_register(handle, &config2) );

    /* set output parameter */
    *gain = config2.bits.gain;

    return ESP_OK;
}

esp_err_t hmc5883l_set_gain(hmc5883l_handle_t handle, const hmc5883l_gains_t gain) {
    hmc5883l_configuration2_register_t config2;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration2_register(handle, &config2) );

    /* set register setting */
    config2.bits.gain = gain;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_set_configuration2_register(handle, config2) );

    return ESP_OK;
}

esp_err_t hmc5883l_get_gain_sensitivity(hmc5883l_handle_t handle, float *const sensitivity) {
    hmc5883l_configuration2_register_t config2;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_get_configuration2_register(handle, &config2) );

    /* set output parameter */
    *sensitivity = hmc5883l_gain_values[config2.bits.gain];

    return ESP_OK;
}

esp_err_t hmc5883l_remove(hmc5883l_handle_t handle) {
    hmc5883l_device_t* dev = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t hmc5883l_delete(hmc5883l_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( hmc5883l_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* hmc5883l_get_fw_version(void) {
    return (const char*)HMC5883L_FW_VERSION_STR;
}

int32_t hmc5883l_get_fw_version_number(void) {
    return (int32_t)HMC5883L_FW_VERSION_INT32;
}