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
 * matrix library for ESP-IDF: https://github.com/leeebo/esp-gsl/tree/master
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

#define HMC5883L_AXES_BUFFER_SIZE           UINT8_C(50)


#define HMC5883L_EPSILON                    (1e-9)  // Tolerance for convergence

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
 * @brief HMC5883L configuration 1 (a) register structure.
 */
typedef union __attribute__((packed)) hmc5883l_config1_register_u {
    struct HMC5883L_CFG1_REG_BITS_TAG {
        hmc5883l_biases_t           bias:2;       /*!< measurement configuration, measurement bias (bit:0-1)   */
        hmc5883l_data_rates_t       data_rate:3;  /*!< data rate at which data is written          (bit:2-4)   */
        hmc5883l_sample_averages_t  sample_avg:2; /*!< number of samples averaged                  (bit:5-6)   */
        uint8_t                     reserved:1;   /*!< reserved and set to 0                       (bit:7)     */
    } bits;            /*!< represents the 8-bit configuration 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit configuration 1 register as `uint8_t`.   */
} hmc5883l_config1_register_t;

/**
 * @brief HMC5883L configuration 2 (b) register structure.
 */
typedef union __attribute__((packed)) hmc5883l_config2_register_u {
    struct HMC5883L_CFG2_REG_BITS_TAG {
        uint8_t                     reserved:5;   /*!< reserved and set to 0                       (bit:0-4)     */
        hmc5883l_gains_t            gain:3;       /*!< gain configuration for all channels         (bit:5-7)   */
    } bits;            /*!< represents the 8-bit configuration 2 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit configuration 2 register as `uint8_t`.   */
} hmc5883l_config2_register_t;

/**
 * @brief HMC5883L mode register structure.
 */
typedef union __attribute__((packed)) hmc5883l_mode_register_u {
    struct HMC5883L_MODE_REG_BITS_TAG {
        hmc5883l_modes_t            mode:2;        /*!< operation mode                              (bit:0-1)   */
        uint8_t                     high_speed:6;  /*!< set high to enable i2c high speed (3400khz) (bit:2-7)   */
    } bits;            /*!< represents the 8-bit mode register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit mode register as `uint8_t`.   */
} hmc5883l_mode_register_t;

/**
 * @brief HMC5883L status register structure.
 */
typedef union __attribute__((packed)) hmc5883l_status_register_u {
    struct HMC5883L_STATUS_REG_BITS_TAG {
        bool            data_ready:1;     /*!< data is ready when asserted to true        (bit:0)   */
        bool            data_locked:1;    /*!< data is locked when asserted to true        (bit:1)   */
        uint8_t         reserved:6;       /*!< reserved (bit:2-7)   */
    } bits;            /*!< represents the 8-bit status register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status register as `uint8_t`.   */
} hmc5883l_status_register_t;

/**
 * @brief HMC5883L device descriptor structure definition.
 */
typedef struct hmc5883l_device_s {
    hmc5883l_config_t                   config;
    i2c_master_dev_handle_t             i2c_handle;     /*!< I2C device handle */
    uint32_t                            dev_id;
    bool                                gain_calibrated;
    bool                                offset_calibrated;
    hmc5883l_offset_axes_data_t         offset_axes;
    hmc5883l_gain_error_axes_data_t     gain_error_axes;
    //
    // calibration parameters - least-squares fitting or similar optimization techniques

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

/**
 * @brief HMC5883L I2C HAL read identification register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[out] reg HMC5883L identification register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_get_ident_register(hmc5883l_device_t *const device, uint32_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* hmc5883l attempt to read device identification */
    uint8_t ident_a; uint8_t ident_b; uint8_t ident_c;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_IDENT_A, &ident_a), TAG, "read register IDENT_A failed");
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_IDENT_B, &ident_b), TAG, "read register IDENT_B failed");
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_IDENT_C, &ident_c), TAG, "read register IDENT_C failed");

    /* construct device identification */
    *reg = ident_a | (ident_b << 8) | (ident_c << 16);

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read configuration 1 register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[out] reg HMC5883L configuration 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_get_config1_register(hmc5883l_device_t *const device, hmc5883l_config1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_CONFIG_A, &reg->reg), TAG, "read configuration 1 register failed" );

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL write configuration 1 register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[in] reg HMC5883L configuration 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_set_config1_register(hmc5883l_device_t *const device, const hmc5883l_config1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    hmc5883l_config1_register_t config1 = { .reg = reg.reg };

    /* set register reserved settings */
    config1.bits.reserved = 0;

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_write_byte_to(device, HMC5883L_REG_CONFIG_A, config1.reg), TAG, "write configuration 1 register failed");

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read configuration 2 register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[out] reg HMC5883L configuration 2 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_get_config2_register(hmc5883l_device_t *const device, hmc5883l_config2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_CONFIG_B, &reg->reg), TAG, "read configuration 2 register failed" );

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL write configuration 2 register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[in] reg HMC5883L configuration 2 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_set_config2_register(hmc5883l_device_t *const device, const hmc5883l_config2_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    hmc5883l_config2_register_t config2 = { .reg = reg.reg };

    /* set register reserved settings */
    config2.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_write_byte_to(device, HMC5883L_REG_CONFIG_B, config2.reg), TAG, "write configuration 2 register failed");

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read mode register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[out] reg HMC5883L mode register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_get_mode_register(hmc5883l_device_t *const device, hmc5883l_mode_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_MODE, &reg->reg), TAG, "read mode register failed" );

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL write mode register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[in] reg HMC5883L mode register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_set_mode_register(hmc5883l_device_t *const device, const hmc5883l_mode_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    hmc5883l_mode_register_t mode = { .reg = reg.reg };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_write_byte_to(device, HMC5883L_REG_MODE, mode.reg), TAG, "write mode register failed");

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read status register.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @param[out] reg HMC5883L status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_get_status_register(hmc5883l_device_t *const device, hmc5883l_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( hmc5883l_i2c_read_byte_from(device, HMC5883L_REG_STATUS, &reg->reg), TAG, "read status register failed" );

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL setup registers.
 * 
 * @param[in] device HMC5883L device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_setup_registers(hmc5883l_device_t *const device) {
    hmc5883l_config1_register_t config1_reg = { 0 };
    hmc5883l_config2_register_t config2_reg = { 0 };
    hmc5883l_mode_register_t       mode_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read registers */
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_get_config1_register(device, &config1_reg), TAG, "read configuration 1 register failed");

    ESP_RETURN_ON_ERROR(hmc5883l_i2c_get_config2_register(device, &config2_reg), TAG, "read configuration 2 register failed");

    ESP_RETURN_ON_ERROR(hmc5883l_i2c_get_mode_register(device, &mode_reg), TAG, "read mode register failed");

    ESP_RETURN_ON_ERROR(hmc5883l_i2c_get_ident_register(device, &device->dev_id), TAG, "read identification register failed");

    /* validate device identifier */
    if (device->dev_id != HMC5883L_DEV_ID) {
        ESP_LOGE(TAG, "Unknown ID: %lu (device) != %lu (reference)", device->dev_id, HMC5883L_DEV_ID);
        ESP_RETURN_ON_FALSE(false, ESP_ERR_NOT_FOUND, TAG, "hmc5883l device identifier validation failed");
    }

    /* attempt to write configuration 1 register */
    config1_reg.bits.bias       = device->config.bias;
    config1_reg.bits.data_rate  = device->config.rate;
    config1_reg.bits.sample_avg = device->config.sample;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_config1_register(device, config1_reg), TAG, "write configuration 1 register failed");

    /* attempt to write configuration 2 register */
    config2_reg.bits.gain       = device->config.gain;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_config2_register(device, config2_reg), TAG, "write configuration 2 register failed");

    /* attempt to write mode register */
    mode_reg.bits.mode          = device->config.mode;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_mode_register(device, mode_reg), TAG, "write mode register failed");

    return ESP_OK;
}

/**
 * @brief HMC5883L I2C HAL read uncompensated axes measurements from ADC.
 * 
 * @param device HMC5883L device descriptor.
 * @param axes_data Uncompensated ADC axes measurements (x, y, and z axes).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hmc5883l_i2c_get_adc_axes(hmc5883l_device_t *const device, hmc5883l_adc_axes_data_t *const axes_data) {
    /* validate arguments */
    ESP_ARG_CHECK( device && axes_data );

    /* initialize local variables */
    esp_err_t    ret        = ESP_OK;
    uint64_t     start      = esp_timer_get_time();
    bool         is_ready   = false;
    bit48_uint8_buffer_t rx = { 0 };

    /* poll data status until data is ready or timeout condition is asserted */
    do {
        hmc5883l_status_register_t status = { 0 };

        /* read data status */
        ESP_GOTO_ON_ERROR( hmc5883l_i2c_get_status_register(device, &status), err, TAG, "data ready for get fixed measurement failed" );

        /* set data status */
        is_ready = status.bits.data_ready;

        /* delay task before i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(HMC5883L_DATA_READY_DELAY_MS));

        if (ESP_TIMEOUT_CHECK(start, HMC5883L_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (!is_ready);
    
    /* attempt i2c read transaction */
    ESP_GOTO_ON_ERROR( hmc5883l_i2c_read_from(device, HMC5883L_REG_DATA_OUT_X_MSB, rx, BIT48_UINT8_BUFFER_SIZE), err, TAG, "read uncompensated compass data for get fixed measurement failed" );

    /* convert 2-byte data to int16 data type - 2s complement */
    axes_data->x = (int16_t)(rx[0] << 8) | rx[1];
    axes_data->z = (int16_t)(rx[4] << 8) | rx[5];
    axes_data->y = (int16_t)(rx[2] << 8) | rx[3];

    //ESP_LOGW(TAG, "Raw X-Axis: %d", data->x_axis);
    //ESP_LOGW(TAG, "Raw Y-Axis: %d", data->y_axis);
    //ESP_LOGW(TAG, "Raw Z-Axis: %d", data->z_axis);

    return ESP_OK;

    err:
        return ret;
}

/**
 * @brief Calculates heading in degrees from magnetic x and y components.
 * 
 * @param x_axis X axis magnetic component.
 * @param y_axis Y axis magnetic component.
 * @return float Heading in degrees.
 */
static inline float hmc5883l_calculate_heading(const float x_axis, const float y_axis) {
    float heading;

    /* honeywell application note AN-203 */
    if(y_axis > 0.0f) { 
        heading = 90.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else if(y_axis < 0.0f) {
        heading = 270.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else {
        if(x_axis < 0.0f) { 
            heading = 180.0f;
        } else if(x_axis > 0.0f) { 
            heading = 0.0f;
        } else {
            heading = 0.0f;
        }
    }

    /* convert to heading to a 0..360 degree range */
    if (heading < 0.0f) {
        heading += 360.0f;
    } else if (heading > 360.0f) {
        heading -= 360.0f;
    }

    return heading;
}

/**
 * @brief Calculates heading from true north in degrees from magnetic x and y components and magnetic declination.
 * 
 * @param x_axis X axis magnetic component.
 * @param y_axis Y axis magnetic component.
 * @param declination Magnetic declination in degrees (+east / -west).
 * @return float Heading from true north in degrees.
 */
static inline float hmc5883l_calculate_true_heading(const float x_axis, const float y_axis, const float declination) {
    float heading;

    /* honeywell application note AN-203 */
    if(y_axis > 0.0f) { 
        heading = 90.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else if(y_axis < 0.0f) {
        heading = 270.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else {
        if(x_axis < 0.0f) { 
            heading = 180.0f;
        } else if(x_axis > 0.0f) { 
            heading = 0.0f;
        } else {
            heading = 0.0f;
        }
    }

    /* apply magnetic declination (+east | -west) */
    heading += declination;

    /* convert to heading to a 0..360 degree range */
    if (heading < 0.0f) {
        heading += 360.0f;
    } else if (heading > 360.0f) {
        heading -= 360.0f;
    }

    return heading;
}

static inline float hmc5883l_calculate_earth_field(const float x_axis, const float y_axis, const float z_axis) {
    return sqrtf(powf(x_axis, 2) + powf(y_axis, 2) + powf(z_axis, 2));
}

// Function to print a 3x3 matrix
static inline void hmc5883l_print_3x3_matrix(hmc5883l_3x3_matrix_t m) {
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        printf("%s ", TAG);
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            printf("(%12.6f)[%d][%d] ", m.m[i][j], (int16_t)i, (int16_t)j);
        }
        printf("\n");
    }
}

// Function to print a 9x9 matrix
static inline void hmc5883l_print_9x9_matrix(hmc5883l_9x9_matrix_t m) {
    for (uint8_t i = 0; i < HMC5883L_9X9_MATRIX_SIZE; i++) {
        printf("%s ", TAG);
        for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE; j++) {
            printf("(%12.6f)[%d][%d] ", m.m[i][j], (int16_t)i, (int16_t)j);
        }
        printf("\n");
    }
}

/**
 * @brief Adds two 3D vector components together.
 * 
 * @param a Augend 3D vector components to add to b.
 * @param b Addend 3D vector components to add to a.
 * @return hmc5883l_3d_vector_t Sum of a and b 3D vector components.
 */
static inline hmc5883l_3d_vector_t hmc5883l_3d_vector_add(const hmc5883l_3d_vector_t a, const hmc5883l_3d_vector_t b) {
    return (hmc5883l_3d_vector_t){ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z}; 
}

/**
 * @brief Subtracts 3D vector components from another.
 * 
 * @param a Minuend 3D vector components.
 * @param b Subtrahend 3D vector components i.e. 3D vector components to subtract from a.
 * @return hmc5883l_3d_vector_t Difference between a and b 3D vector components.
 */
static inline hmc5883l_3d_vector_t hmc5883l_3d_vector_sub(const hmc5883l_3d_vector_t a, const hmc5883l_3d_vector_t b) { 
    return (hmc5883l_3d_vector_t){ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z}; 
}

/**
 * @brief Scales 3D vector components by a scaling factor.
 * 
 * @param v 3D vector components to scale.
 * @param s Scaling factor for 3D vector components.
 * @return hmc5883l_3d_vector_t Scaled 3D vector components.
 */
static inline hmc5883l_3d_vector_t hmc5883l_3d_vector_scale(const hmc5883l_3d_vector_t v, const double s) { 
    return (hmc5883l_3d_vector_t){ .x = v.x * s, .y = v.y * s, .z = v.z * s}; 
}

/**
 * @brief Scales 3D vector (int16_t) components by a scaling factor.
 * 
 * @param v 3D vector (int16_t) components to scale.
 * @param s Scaling factor for 3D vector (int16_t) components.
 * @return hmc5883l_3d_vector_t Scaled 3D vector components.
 */
static inline hmc5883l_3d_vector_t hmc5883l_3d_vector_int_scale(const hmc5883l_3d_vector_int_t v, const double s) { 
    return (hmc5883l_3d_vector_t){ .x = (double)v.x * s, .y = (double)v.y * s, .z = (double)v.z * s}; 
}

static inline hmc5883l_3x3_matrix_t hmc5883l_3x3_matrix_add(const hmc5883l_3x3_matrix_t a, const hmc5883l_3x3_matrix_t b) {
    hmc5883l_3x3_matrix_t result = {0};
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            result.m[i][j] = a.m[i][j] + b.m[i][j];
        }
    }
    return result;
}

static inline hmc5883l_9x9_matrix_t hmc5883l_9x9_matrix_add(const hmc5883l_9x9_matrix_t a, const hmc5883l_9x9_matrix_t b) {
    hmc5883l_9x9_matrix_t result = {0};
    for (uint8_t i = 0; i < HMC5883L_9X9_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE; j++) {
            result.m[i][j] = a.m[i][j] + b.m[i][j];
        }
    }
    return result;
}

static inline hmc5883l_3d_vector_t hmc5883l_3x3_matrix_multiply_3d_vector(const hmc5883l_3x3_matrix_t matrix, const hmc5883l_3d_vector_t vector) {
    return (hmc5883l_3d_vector_t) {
        .x = matrix.m[0][0] * vector.x + matrix.m[1][0] * vector.y + matrix.m[2][0] * vector.z,
        .y = matrix.m[0][1] * vector.x + matrix.m[1][1] * vector.y + matrix.m[2][1] * vector.z,
        .z = matrix.m[0][2] * vector.x + matrix.m[1][2] * vector.y + matrix.m[2][2] * vector.z
    };
}

static inline hmc5883l_3x3_matrix_t hmc5883l_3x3_matrix_multiply__(const hmc5883l_3x3_matrix_t a, const hmc5883l_3x3_matrix_t b) {
    hmc5883l_3x3_matrix_t result = {0};
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            result.m[i][j] = 0.0;
            for (uint8_t k = 0; k < HMC5883L_3X3_MATRIX_SIZE; k++) {
                result.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }
    return result;
}

static inline hmc5883l_3x3_matrix_t hmc5883l_3x3_matrix_multiply(const hmc5883l_3x3_matrix_t a, const hmc5883l_3x3_matrix_t b) {
    hmc5883l_3x3_matrix_t result = {0};

    float   a00 = a.m[0][0], a01 = a.m[0][1], a02 = a.m[0][2],
            a10 = a.m[1][0], a11 = a.m[1][1], a12 = a.m[1][2],
            a20 = a.m[2][0], a21 = a.m[2][1], a22 = a.m[2][2],

            b00 = b.m[0][0], b01 = b.m[0][1], b02 = b.m[0][2],
            b10 = b.m[1][0], b11 = b.m[1][1], b12 = b.m[1][2],
            b20 = b.m[2][0], b21 = b.m[2][1], b22 = b.m[2][2];

    result.m[0][0] = a00 * b00 + a10 * b01 + a20 * b02;
    result.m[0][1] = a01 * b00 + a11 * b01 + a21 * b02;
    result.m[0][2] = a02 * b00 + a12 * b01 + a22 * b02;
    result.m[1][0] = a00 * b10 + a10 * b11 + a20 * b12;
    result.m[1][1] = a01 * b10 + a11 * b11 + a21 * b12;
    result.m[1][2] = a02 * b10 + a12 * b11 + a22 * b12;
    result.m[2][0] = a00 * b20 + a10 * b21 + a20 * b22;
    result.m[2][1] = a01 * b20 + a11 * b21 + a21 * b22;
    result.m[2][2] = a02 * b20 + a12 * b21 + a22 * b22;

    return result;
}

/**
 * @brief Calculates the inverse of a 3x3 matrix using Gauss-Jordan elimination.
 * 
 * @param source The original matrix.
 * @param inverse The resulting inverse matrix
 * @return int8_t 0 if successful, -1 if the matrix is singular.
 */
static inline int8_t hmc5883l_3x3_matrix_inverse(const hmc5883l_3x3_matrix_t source, hmc5883l_3x3_matrix_t *const inverse) {
    // 1. Create an augmented matrix [A|I]
    double augmented[HMC5883L_3X3_MATRIX_SIZE][HMC5883L_3X3_MATRIX_SIZE * 2] = { 0 };
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            augmented[i][j] = source.m[i][j];
            augmented[i][j + HMC5883L_3X3_MATRIX_SIZE] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 2. Perform Gauss-Jordan elimination
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        // Partial pivoting: Find the row with the largest absolute value in the current column
        uint8_t pivot_row = i;
        for (uint8_t k = i + 1; k < HMC5883L_3X3_MATRIX_SIZE; k++) {
            if (fabs(augmented[k][i]) > fabs(augmented[pivot_row][i])) {
                pivot_row = k;
            }
        }

        // Swap the current row with the pivot row
        if (pivot_row != i) {
            for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE * 2; j++) {
                const double temp = augmented[i][j];
                augmented[i][j] = augmented[pivot_row][j];
                augmented[pivot_row][j] = temp;
            }
        }

        // If the pivot element is zero, the matrix is singular and has no inverse
        if (fabs(augmented[i][i]) < HMC5883L_EPSILON) {
            printf("Matrix is singular, no inverse exists.\n");
            return -1;
        }

        // Normalize the pivot row to make the pivot element 1
        const double pivot_value = augmented[i][i];
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE * 2; j++) {
            augmented[i][j] /= pivot_value;
        }

        // Eliminate other elements in the current column
        for (uint8_t k = 0; k < HMC5883L_3X3_MATRIX_SIZE; k++) {
            if (k != i) {
                const double factor = augmented[k][i];
                for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE * 2; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }

    // 3. Extract the inverse matrix from the augmented matrix
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            inverse->m[i][j] = augmented[i][j + HMC5883L_3X3_MATRIX_SIZE];
        }
    }

    return 0;
}

/**
 * @brief Calculates the inverse of a 9x9 matrix using Gauss-Jordan elimination.
 * 
 * @param source The original matrix.
 * @param inverse The resulting inverse matrix
 * @return int8_t 0 if successful, -1 if the matrix is singular.
 */
static inline int8_t hmc5883l_9x9_matrix_inverse(const hmc5883l_9x9_matrix_t source, hmc5883l_9x9_matrix_t *const inverse) {
    // 1. Create an augmented matrix [A|I]
    double augmented[HMC5883L_9X9_MATRIX_SIZE][HMC5883L_9X9_MATRIX_SIZE * 2] = { 0 };
    for (uint8_t i = 0; i < HMC5883L_9X9_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE; j++) {
            augmented[i][j] = source.m[i][j];
            augmented[i][j + HMC5883L_9X9_MATRIX_SIZE] = (i == j) ? 1.0 : 0.0;
        }
    }

    // 2. Perform Gauss-Jordan elimination
    for (uint8_t i = 0; i < HMC5883L_9X9_MATRIX_SIZE; i++) {
        // Partial pivoting: Find the row with the largest absolute value in the current column
        uint8_t pivot_row = i;
        for (uint8_t k = i + 1; k < HMC5883L_9X9_MATRIX_SIZE; k++) {
            if (fabs(augmented[k][i]) > fabs(augmented[pivot_row][i])) {
                pivot_row = k;
            }
        }

        // Swap the current row with the pivot row
        if (pivot_row != i) {
            for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE * 2; j++) {
                const double temp = augmented[i][j];
                augmented[i][j] = augmented[pivot_row][j];
                augmented[pivot_row][j] = temp;
            }
        }

        // If the pivot element is zero, the matrix is singular and has no inverse
        if (fabs(augmented[i][i]) < HMC5883L_EPSILON) {
            printf("Matrix is singular, no inverse exists.\n");
            return -1;
        }

        // Normalize the pivot row to make the pivot element 1
        const double pivot_value = augmented[i][i];
        for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE * 2; j++) {
            augmented[i][j] /= pivot_value;
        }

        // Eliminate other elements in the current column
        for (uint8_t k = 0; k < HMC5883L_9X9_MATRIX_SIZE; k++) {
            if (k != i) {
                const double factor = augmented[k][i];
                for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE * 2; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }

    // 3. Extract the inverse matrix from the augmented matrix
    for (uint8_t i = 0; i < HMC5883L_9X9_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_9X9_MATRIX_SIZE; j++) {
            inverse->m[i][j] = augmented[i][j + HMC5883L_9X9_MATRIX_SIZE];
        }
    }

    return 0;
}

//static inline int8_t hmc5883l_9x9_matrix_inverse(const hmc5883l_9x9_matrix_t source, hmc5883l_9x9_matrix_t *const inverse) {
//}

static inline hmc5883l_3x3_matrix_t hmc5883l_3x3_matrix_transpose(const hmc5883l_3x3_matrix_t a) {
    hmc5883l_3x3_matrix_t a_T = {0};
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            a_T.m[i][j] = a.m[j][i];
        }
    }
    return a_T;
}

/**
 * @brief Helper function to find the indices of the largest off-diagonal element.
 * 
 * @param source Symmetric 3x3 matrix.
 * @param p 
 * @param q 
 */
static inline void hmc5883l_find_max_off_diagonal(const hmc5883l_3x3_matrix_t source, uint8_t *const p, uint8_t *const q) {
    *p = 0;
    *q = 1;
    double max_val = 0.0;
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = i + 1; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            const double abs_val = fabs(source.m[i][j]);
            if (abs_val > max_val) {
                max_val = abs_val;
                *p = i;
                *q = j;
            }
        }
    }
}

/**
 * @brief Jacobi eigenvalue decomposition for a symmetric 3x3 matrix
 * 
 * @param source 
 * @param eigen_values 
 * @param eigen_vectors 
 */
static inline void hmc5883l_eigen_3x3_matrix_decompose(const hmc5883l_3x3_matrix_t source, hmc5883l_3d_vector_t *const eigen_values, hmc5883l_3x3_matrix_t *const eigen_vectors) {
    const uint8_t max_iterations = 50;
    hmc5883l_3x3_matrix_t A;
    hmc5883l_3x3_matrix_t V;
    hmc5883l_3x3_matrix_t J;
    uint8_t iter = 0;

    // Initialize V (eigenvector matrix) to the identity matrix
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            V.m[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Initialize A (eigenvalue matrix) to the input matrix
    while (iter < max_iterations) {
        hmc5883l_3x3_matrix_t temp;
        uint8_t p, q;

        // Find the largest off-diagonal element
        hmc5883l_find_max_off_diagonal(source, &p, &q);
        const double max_off_diag_val = source.m[p][q];

        // Check for convergence
        if (fabs(max_off_diag_val) < HMC5883L_EPSILON) {
            break;
        }

        // Compute Jacobi rotation angle
        const double diff = source.m[q][q] - source.m[p][p];
        const double theta = 0.5 * atan2(2.0 * max_off_diag_val, diff);
        const double c = cos(theta);
        const double s = sin(theta);

        // Construct Jacobi rotation matrix J
        for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
            for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
                J.m[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        J.m[p][p] = c;
        J.m[q][q] = c;
        J.m[p][q] = -s;
        J.m[q][p] = s;

        // Apply transformation: A = J^T * A * J
        // temp = A * J
        temp = hmc5883l_3x3_matrix_multiply(source, J);

        // A = J^T * temp
        A = hmc5883l_3x3_matrix_multiply(J, temp);

        // Accumulate eigenvectors: V = V * J
        V = hmc5883l_3x3_matrix_multiply(V, J);

        iter++;
    }

    // Extract eigen values from the diagonal of A
    eigen_values->x = A.m[0][0];
    eigen_values->y = A.m[1][1];
    eigen_values->z = A.m[2][2];

    // Extract eigen vectors from V
    *eigen_vectors = V;
}

static inline int8_t hmc5883l_solve_ellipsoid_coefficients(const hmc5883l_calibration_samples_t samples, hmc5883l_ellipsoid_coefficients_t *const coefficients) {
    double D_trans[HMC5883L_ELLIPSOID_COEFF_SIZE][HMC5883L_CAL_SAMPLE_SIZE]; // Transpose of the design matrix
    hmc5883l_9x9_matrix_t D_trans_D = {0}; // D_trans * D
    double D_trans_k[HMC5883L_ELLIPSOID_COEFF_SIZE] = {0}; // D_trans * k
    hmc5883l_9x9_matrix_t inv_D_trans_D; // Inverse of D_trans_D
    double k[HMC5883L_CAL_SAMPLE_SIZE] = {0}; // The target vector k (set to 1)
    hmc5883l_ellipsoid_coefficients_t coeff = {0};

    // 1. Formulate the least-squares problem
    for (uint8_t i = 0; i < HMC5883L_CAL_SAMPLE_SIZE; i++) {
        const double x = samples[i].x;
        const double y = samples[i].y;
        const double z = samples[i].z;
        
        // Populate the transpose of the design matrix (D_trans)
        D_trans[0][i] = x * x;  // A x^2
        D_trans[1][i] = y * y;  // B y^2
        D_trans[2][i] = z * z;  // C z^2
        D_trans[3][i] = x * y;  // D xy
        D_trans[4][i] = x * z;  // E xz
        D_trans[5][i] = y * z;  // F yz
        D_trans[6][i] = x;      // G x
        D_trans[7][i] = y;      // H y
        D_trans[8][i] = z;      // I z

        k[i] = 1.0; // The target value for the general equation 
    }

    // 2. Compute the components for the normal equations
    for (uint8_t i = 0; i < HMC5883L_ELLIPSOID_COEFF_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_ELLIPSOID_COEFF_SIZE; j++) {
            for (uint8_t p = 0; p < HMC5883L_CAL_SAMPLE_SIZE; p++) {
                D_trans_D.m[i][j] += D_trans[i][p] * D_trans[j][p];
            }
        }
        for (uint8_t p = 0; p < HMC5883L_CAL_SAMPLE_SIZE; p++) {
            D_trans_k[i] += D_trans[i][p] * k[p];
        }
    }

    printf("D_trans_D Matrix.\n");
    hmc5883l_print_9x9_matrix(D_trans_D);

    // 3. Solve the normal equations (D_trans_D * v = D_trans_k)
    if (hmc5883l_9x9_matrix_inverse(D_trans_D, &inv_D_trans_D) != 0) {
        printf("Matrix is singular, could not find coefficients.\n");
        return -1;
    }

    printf("Inverse of D_trans_D Matrix.\n");
    hmc5883l_print_9x9_matrix(inv_D_trans_D);

    // Multiply the inverse by D_trans_k to get the coefficients
    for (uint8_t i = 0; i < HMC5883L_ELLIPSOID_COEFF_SIZE; i++) {
        coeff[i] = 0.0;
        for (uint8_t j = 0; j < HMC5883L_ELLIPSOID_COEFF_SIZE; j++) {
            coeff[i] += inv_D_trans_D.m[i][j] * D_trans_k[j];
        }
    }

    memcpy(coefficients, coeff, sizeof(hmc5883l_ellipsoid_coefficients_t));

    return 0;
}

static inline void decompose_ellipsoid(const hmc5883l_ellipsoid_coefficients_t coefficients, hmc5883l_3d_vector_t *const hard_iron, hmc5883l_3x3_matrix_t *const soft_iron) {
    // Step 1: Construct the matrix M and vector N from coefficients
    const hmc5883l_3x3_matrix_t M = {{{
        coefficients[0], coefficients[3]/2, coefficients[4]/2
    }, {
        coefficients[3]/2, coefficients[1], coefficients[5]/2
    }, {
        coefficients[4]/2, coefficients[5]/2, coefficients[2]
    }}};

    const hmc5883l_3d_vector_t N = {
       .x = coefficients[6], .y = coefficients[7], .z = coefficients[8]
    };

    // Step 2: Calculate the hard-iron offset (center of the ellipsoid)
    hmc5883l_3x3_matrix_t M_inv;
    hmc5883l_3x3_matrix_inverse(M, &M_inv);

    const hmc5883l_3d_vector_t M_inv_N = hmc5883l_3x3_matrix_multiply_3d_vector(M_inv, N);

    hard_iron->x = -0.5 * M_inv_N.x;
    hard_iron->y = -0.5 * M_inv_N.y;
    hard_iron->z = -0.5 * M_inv_N.z;

    // Step 3: Eigenvalue decomposition of M
    hmc5883l_3d_vector_t eigen_values;
    hmc5883l_3x3_matrix_t eigen_vectors;
    hmc5883l_eigen_3x3_matrix_decompose(M, &eigen_values, &eigen_vectors);

    // Step 4: Construct the soft-iron matrix from eigen-vectors and eigen-values
    hmc5883l_3x3_matrix_t lambda_inv_sqrt;
    for (uint8_t i = 0; i < HMC5883L_3X3_MATRIX_SIZE; i++) {
        for (uint8_t j = 0; j < HMC5883L_3X3_MATRIX_SIZE; j++) {
            lambda_inv_sqrt.m[i][j] = 0.0;
        }
    }
    if(eigen_values.x > HMC5883L_EPSILON) {  // avoid division by zero
        lambda_inv_sqrt.m[0][0] = 1.0 / sqrt(eigen_values.x);
    }
    if(eigen_values.y > HMC5883L_EPSILON) {  // avoid division by zero
        lambda_inv_sqrt.m[1][1] = 1.0 / sqrt(eigen_values.y);
    }
    if(eigen_values.z > HMC5883L_EPSILON) {  // avoid division by zero
        lambda_inv_sqrt.m[2][2] = 1.0 / sqrt(eigen_values.z);
    }

    // Step 5: Transpose the eigen-vector matrix
    const hmc5883l_3x3_matrix_t eigen_vectors_T = hmc5883l_3x3_matrix_transpose(eigen_vectors);

    // Step 6: Multiply: temp = eigen_vectors * lambda_inv_sqrt
    const hmc5883l_3x3_matrix_t temp = hmc5883l_3x3_matrix_multiply(eigen_vectors, lambda_inv_sqrt);

    // Step 7: Multiply: soft_iron = temp * eigen_vectors_T
    *soft_iron = hmc5883l_3x3_matrix_multiply(temp, eigen_vectors_T);
}


esp_err_t hmc5883l_init(i2c_master_bus_handle_t master_handle, const hmc5883l_config_t *hmc5883l_config, hmc5883l_handle_t *hmc5883l_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && hmc5883l_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, hmc5883l_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, hmc5883l device handle initialization failed", hmc5883l_config->i2c_address);

    /* validate memory availability for handle */
    hmc5883l_device_t* device = (hmc5883l_device_t*)calloc(1, sizeof(hmc5883l_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c hmc5883l device");

    /* copy configuration */
    device->config = *hmc5883l_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate i2c device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_CMD_DELAY_MS));

    /* attempt to i2c setup registers */
    ESP_GOTO_ON_ERROR(hmc5883l_i2c_setup_registers(device), err_handle, TAG, "setup registers failed");

    /* set device handle */
    *hmc5883l_handle = (hmc5883l_handle_t)device;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(HMC5883L_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t hmc5883l_get_magnetic_axes(hmc5883l_handle_t handle, hmc5883l_magnetic_axes_data_t *const axes_data) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && axes_data );

    /* set gain sensitivity */
    const float gain_sensitivity = hmc5883l_gain_values[device->config.gain];

    /* attempt to read uncompensated magnetic measurements */
    hmc5883l_adc_axes_data_t raw;
    ESP_ERROR_CHECK( hmc5883l_i2c_get_adc_axes(device, &raw) );

    /* handle calibration corrections and compensation factors */
    if(device->gain_calibrated == true && device->offset_calibrated == true) {
        axes_data->x_axis  = (float)raw.x * gain_sensitivity * device->gain_error_axes.x + device->offset_axes.x;
        axes_data->y_axis  = (float)raw.y * gain_sensitivity * device->gain_error_axes.y + device->offset_axes.y;
        axes_data->z_axis  = (float)raw.z * gain_sensitivity * device->gain_error_axes.z + device->offset_axes.z;
    } else if(device->gain_calibrated == true && device->offset_calibrated == false) {
        axes_data->x_axis  = (float)raw.x * gain_sensitivity * device->gain_error_axes.x;
        axes_data->y_axis  = (float)raw.y * gain_sensitivity * device->gain_error_axes.y;
        axes_data->z_axis  = (float)raw.z * gain_sensitivity * device->gain_error_axes.z;
    } else if(device->gain_calibrated == false && device->offset_calibrated == true) {
        axes_data->x_axis  = (float)raw.x * gain_sensitivity + device->offset_axes.x;
        axes_data->y_axis  = (float)raw.y * gain_sensitivity + device->offset_axes.y;
        axes_data->z_axis  = (float)raw.z * gain_sensitivity + device->offset_axes.z;
    } else {
        axes_data->x_axis  = (float)raw.x * gain_sensitivity;
        axes_data->y_axis  = (float)raw.y * gain_sensitivity;
        axes_data->z_axis  = (float)raw.z * gain_sensitivity;
    }

    /* honeywell application note AN-203 */
    axes_data->heading = hmc5883l_calculate_heading(axes_data->x_axis, axes_data->y_axis);

    /* honeywell application note AN-203 */
    axes_data->true_heading = hmc5883l_calculate_true_heading(axes_data->x_axis, axes_data->y_axis, device->config.declination);

    /* honeywell application note AN-203 */
    axes_data->earth_field = hmc5883l_calculate_earth_field(axes_data->x_axis, axes_data->y_axis, axes_data->z_axis);

    return ESP_OK;
}

// https://github.com/helscream/HMC5883L_Header_Arduino_Auto_calibration/blob/master/Core/Compass_header_example_ver_0_2/compass.cpp

esp_err_t hmc5883l_get_calibrated_offsets(hmc5883l_handle_t handle, const hmc5883l_calibration_options_t option) {
    hmc5883l_config1_register_t     config1         = { 0 };
    hmc5883l_adc_axes_data_t        raw_axes        = { 0 };
    hmc5883l_axes_data_t            scaled_axes     = { 0 };
    hmc5883l_gain_error_axes_data_t gain_error_axes = { 0 };
    hmc5883l_device_t*              device          = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_config1_register(device, &config1) );

    /* set gain sensitivity */                        
    const float gain_sensitivity = hmc5883l_gain_values[device->config.gain];

    /* handle calibration option */
    if(option == HMC5883L_CAL_GAIN_DIFF || option == HMC5883L_CAL_BOTH) {
        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Gain");

        // configuring the control register for positive bias mode
        config1.bits.sample_avg = HMC5883L_SAMPLE_8;
        config1.bits.data_rate  = HMC5883L_DATA_RATE_15_00;
        config1.bits.bias       = HMC5883L_BIAS_POSITIVE;
        ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_config1_register(device, config1), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( hmc5883l_i2c_get_adc_axes(device, &raw_axes) );

        // reading the positive biased data
        //while(raw_axes.x_axis<200 || raw_axes.y_axis<200 || raw_axes.z_axis<200){   // making sure the data is with positive biased
        while(raw_axes.x<50 || raw_axes.y<50 || raw_axes.z<50){
            ESP_ERROR_CHECK( hmc5883l_i2c_get_adc_axes(device, &raw_axes) );
        }

        scaled_axes.x  = (float)raw_axes.x * gain_sensitivity;
        scaled_axes.y  = (float)raw_axes.y * gain_sensitivity;
        scaled_axes.z  = (float)raw_axes.z * gain_sensitivity;

        // offset = 1160 - data positive
        gain_error_axes.x = (float)HMC5883L_XY_EXCITATION/scaled_axes.x;
        gain_error_axes.y = (float)HMC5883L_XY_EXCITATION/scaled_axes.y;
        gain_error_axes.z = (float)HMC5883L_Z_EXCITATION/scaled_axes.z;

        // configuring the control register for negative bias mode
        config1.bits.sample_avg = HMC5883L_SAMPLE_8;
        config1.bits.data_rate  = HMC5883L_DATA_RATE_15_00;
        config1.bits.bias       = HMC5883L_BIAS_NEGATIVE;
        ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_config1_register(device, config1), TAG, "write configuration 1 register failed");

        // disregarding the first data
        ESP_ERROR_CHECK( hmc5883l_i2c_get_adc_axes(device, &raw_axes) );

        // reading the negative biased data
        //while(raw_axes.x_axis>-200 || raw_axes.y_axis>-200 || raw_axes.z_axis>-200){   // making sure the data is with negative biased
        while(raw_axes.x>-50 || raw_axes.y>-50 || raw_axes.z>-50){
            ESP_ERROR_CHECK( hmc5883l_i2c_get_adc_axes(device, &raw_axes) );
        }

        scaled_axes.x  = (float)raw_axes.x * gain_sensitivity;
        scaled_axes.y  = (float)raw_axes.y * gain_sensitivity;
        scaled_axes.z  = (float)raw_axes.z * gain_sensitivity;

        // taking the average of the offsets
        gain_error_axes.x = (float)((HMC5883L_XY_EXCITATION/fabs(scaled_axes.x))+gain_error_axes.x)/2.0f;
        gain_error_axes.y = (float)((HMC5883L_XY_EXCITATION/fabs(scaled_axes.y))+gain_error_axes.y)/2.0f;
        gain_error_axes.z = (float)((HMC5883L_Z_EXCITATION/fabs(scaled_axes.z))+gain_error_axes.z)/2.0f;

        device->gain_calibrated = true;
        device->gain_error_axes = gain_error_axes;

        ESP_LOGW(TAG, "Gain Offset X-Axis: %f", gain_error_axes.x);
        ESP_LOGW(TAG, "Gain Offset Y-Axis: %f", gain_error_axes.y);
        ESP_LOGW(TAG, "Gain Offset Z-Axis: %f", gain_error_axes.z);
    }

    // configuring the control register for normal mode
    config1.bits.sample_avg = HMC5883L_SAMPLE_8;
    config1.bits.data_rate  = HMC5883L_DATA_RATE_15_00;
    config1.bits.bias       = HMC5883L_BIAS_NORMAL;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_config1_register(device, config1), TAG, "write configuration 1 register failed");

    if(option == HMC5883L_CAL_AXES_MEAN || option == HMC5883L_CAL_BOTH) {
        hmc5883l_offset_axes_data_t offset_axes = { .x = NAN, .y = NAN, .z = NAN };
        hmc5883l_offset_axes_data_t max_offset_axes = { .x = NAN, .y = NAN, .z = NAN };
        hmc5883l_offset_axes_data_t min_offset_axes = { .x = NAN, .y = NAN, .z = NAN };
        uint16_t x_count = 0;
        uint16_t y_count = 0;
        uint16_t z_count = 0;
        bool x_zero = false;
        bool y_zero = false;
        bool z_zero = false;

        ESP_LOGW(TAG, "Calibrating the Magnetometer ....... Offset");
        ESP_LOGW(TAG, "Please rotate the magnetometer 2 or 3 times in complete circles within one minute .............");

        while (x_count < 3 || y_count < 3 || z_count < 3) {
            ESP_ERROR_CHECK( hmc5883l_i2c_get_adc_axes(device, &raw_axes) );
            scaled_axes.x = (float)raw_axes.x * gain_sensitivity;
            scaled_axes.y = (float)raw_axes.y * gain_sensitivity;
            scaled_axes.z = (float)raw_axes.z * gain_sensitivity;

            if ((fabs(scaled_axes.x) > 100) || (fabs(scaled_axes.y) > 100) || (fabs(scaled_axes.z) > 100)) {
                continue;
            }

            if (min_offset_axes.x > scaled_axes.x) {
                min_offset_axes.x = scaled_axes.x;
            } else if (max_offset_axes.x < scaled_axes.x) {
                max_offset_axes.x = scaled_axes.x;
            }

            if (min_offset_axes.y > scaled_axes.y) {
                min_offset_axes.y = scaled_axes.y;
            } else if (max_offset_axes.y < scaled_axes.y) {
                max_offset_axes.y = scaled_axes.y;
            }

            if (min_offset_axes.z > scaled_axes.z) {
                min_offset_axes.z = scaled_axes.z;
            } else if (max_offset_axes.z < scaled_axes.z) {
                max_offset_axes.z = scaled_axes.z;
            }

            if (x_zero) {
                if (fabs(scaled_axes.x) > 50) {
                    x_zero = false;
                    x_count++;
                }
            } else {
                if (fabs(scaled_axes.x) < 40) {
                    x_zero = true;
                }
            }

            if (y_zero) {
                if (fabs(scaled_axes.y) > 50) {
                    y_zero = false;
                    y_count++;
                }
            } else {
                if (fabs(scaled_axes.y) < 40) {
                    y_zero = true;
                }
            }

            if (z_zero) {
                if (fabs(scaled_axes.z) > 50) {
                    z_zero = false;
                    z_count++;
                }
            } else {
                if (fabs(scaled_axes.z) < 40) {
                    z_zero = true;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(30));
        }

        offset_axes.x = (max_offset_axes.x + min_offset_axes.x) / 2.0f;
        offset_axes.y = (max_offset_axes.y + min_offset_axes.y) / 2.0f;
        offset_axes.z = (max_offset_axes.z + min_offset_axes.z) / 2.0f;

        device->offset_calibrated = true;
        device->offset_axes       = offset_axes;

        ESP_LOGW(TAG, "Offset X-Axis: %f", offset_axes.x);
        ESP_LOGW(TAG, "Offset Y-Axis: %f", offset_axes.y);
        ESP_LOGW(TAG, "Offset Z-Axis: %f", offset_axes.z);
    }

    // configuring the control register to user defined settings
    config1.bits.bias       = device->config.bias;
    config1.bits.data_rate  = device->config.rate;
    config1.bits.sample_avg = device->config.sample;
    ESP_RETURN_ON_ERROR(hmc5883l_i2c_set_config1_register(device, config1), TAG, "write configuration 1 register failed");

    return ESP_OK;
}

esp_err_t hmc5883l_calibrate(hmc5883l_handle_t handle) {
    hmc5883l_ellipsoid_coefficients_t coefficients = { 0 };
    hmc5883l_calibration_samples_t scaled_samples = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set gain sensitivity */                        
    const double gain_sensitivity = hmc5883l_gain_values[device->config.gain];

    /* populate scaled samples array */
    for(uint8_t i = 0; i < HMC5883L_CAL_SAMPLE_SIZE; i++) {
        hmc5883l_3d_vector_int_t sample = { 0 };
        ESP_RETURN_ON_ERROR( hmc5883l_i2c_get_adc_axes(device, &sample), TAG, "ADC axes could not be read" );
        scaled_samples[i] = hmc5883l_3d_vector_int_scale(sample, gain_sensitivity);
        ESP_LOGI(TAG, "Sample[%d] x = %.4f | y = %.4f | z = %.4f", (int16_t)i, scaled_samples[i].x, scaled_samples[i].y, scaled_samples[i].z);
    }

    /* solve ellipsoid coefficients */
    if(hmc5883l_solve_ellipsoid_coefficients(scaled_samples, &coefficients) != 0) {
        ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_STATE, TAG, "ellipsoid coefficients could not be solved");
    }

    ESP_LOGI(TAG, "Successfully solved for ellipsoid coefficients:");
    ESP_LOGI(TAG, "A = %12.6f", coefficients[0]);
    ESP_LOGI(TAG, "B = %12.6f", coefficients[1]);
    ESP_LOGI(TAG, "C = %12.6f", coefficients[2]);
    ESP_LOGI(TAG, "D = %12.6f", coefficients[3]);
    ESP_LOGI(TAG, "E = %12.6f", coefficients[4]);
    ESP_LOGI(TAG, "F = %12.6f", coefficients[5]);
    ESP_LOGI(TAG, "G = %12.6f", coefficients[6]);
    ESP_LOGI(TAG, "H = %12.6f", coefficients[7]);
    ESP_LOGI(TAG, "I = %12.6f", coefficients[8]);
    // Note: The constant term J is implicitly -1 due to normalization.
    
    return ESP_OK;
}

esp_err_t hmc5883l_get_data_status(hmc5883l_handle_t handle, bool *const ready, bool *const locked) {
    hmc5883l_status_register_t status = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && ready && locked );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_status_register(device, &status) );

    /* set output parameters */
    *ready  = status.bits.data_ready;
    *locked = status.bits.data_locked;

    return ESP_OK;
}

esp_err_t hmc5883l_get_mode(hmc5883l_handle_t handle, hmc5883l_modes_t *const mode) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *mode = device->config.mode;

    return ESP_OK;
}

esp_err_t hmc5883l_set_mode(hmc5883l_handle_t handle, const hmc5883l_modes_t mode) {
    hmc5883l_mode_register_t mode_reg = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_mode_register(device, &mode_reg) );

    /* set register setting */
    mode_reg.bits.mode = mode;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_i2c_set_mode_register(device, mode_reg) );

    /* set device configuration setting */
    device->config.mode = mode;

    return ESP_OK;
}

esp_err_t hmc5883l_get_samples_averaged(hmc5883l_handle_t handle, hmc5883l_sample_averages_t *const sample) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *sample = device->config.sample;

    return ESP_OK;
}

esp_err_t hmc5883l_set_samples_averaged(hmc5883l_handle_t handle, const hmc5883l_sample_averages_t sample) {
    hmc5883l_config1_register_t config1 = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_config1_register(device, &config1) );

    /* set register setting */
    config1.bits.sample_avg = sample;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_i2c_set_config1_register(device, config1) );

    /* set device configuration setting */
    device->config.sample = sample;

    return ESP_OK;
}

esp_err_t hmc5883l_get_data_rate(hmc5883l_handle_t handle, hmc5883l_data_rates_t *const rate) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *rate = device->config.rate;

    return ESP_OK;
}

esp_err_t hmc5883l_set_data_rate(hmc5883l_handle_t handle, const hmc5883l_data_rates_t rate) {
    hmc5883l_config1_register_t config1 = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_config1_register(device, &config1) );

    /* set register setting */
    config1.bits.data_rate = rate;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_i2c_set_config1_register(device, config1) );

    /* set device configuration setting */
    device->config.rate = rate;

    return ESP_OK;
}

esp_err_t hmc5883l_get_bias(hmc5883l_handle_t handle, hmc5883l_biases_t *const bias) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *bias = device->config.bias;

    return ESP_OK;
}

esp_err_t hmc5883l_set_bias(hmc5883l_handle_t handle, const hmc5883l_biases_t bias) {
    hmc5883l_config1_register_t config1 = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_config1_register(device, &config1) );

    /* set register setting */
    config1.bits.bias = bias;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_i2c_set_config1_register(device, config1) );

    /* set device configuration setting */
    device->config.bias = bias;

    return ESP_OK;
}

esp_err_t hmc5883l_get_gain(hmc5883l_handle_t handle, hmc5883l_gains_t *const gain) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *gain = device->config.gain;

    return ESP_OK;
}

esp_err_t hmc5883l_set_gain(hmc5883l_handle_t handle, const hmc5883l_gains_t gain) {
    hmc5883l_config2_register_t config2 = { 0 };
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read register */
    ESP_ERROR_CHECK( hmc5883l_i2c_get_config2_register(device, &config2) );

    /* set register setting */
    config2.bits.gain = gain;

    /* attempt to write to register */
    ESP_ERROR_CHECK( hmc5883l_i2c_set_config2_register(device, config2) );

    /* set device configuration setting */
    device->config.gain = gain;

    return ESP_OK;
}

esp_err_t hmc5883l_get_gain_sensitivity(hmc5883l_handle_t handle, float *const sensitivity) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *sensitivity = hmc5883l_gain_values[device->config.gain];

    return ESP_OK;
}

esp_err_t hmc5883l_remove(hmc5883l_handle_t handle) {
    hmc5883l_device_t* device = (hmc5883l_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    return i2c_master_bus_rm_device(device->i2c_handle);
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