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
 * @file ltr390uv.c
 *
 * ESP-IDF driver for LTR390UV sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ltr390uv.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * LTR390UV definitions
 */

#define LTR390UV_REG_MAIN_CTRL_RW       UINT8_C(0x00) /* 0x00 */
#define LTR390UV_REG_ALS_UVS_MEAS_RW    UINT8_C(0x04)
#define LTR390UV_REG_ALS_UVS_GAIN_RW    UINT8_C(0x05)
#define LTR390UV_REG_PART_ID_R          UINT8_C(0x06)
#define LTR390UV_REG_MAIN_STS_R         UINT8_C(0x07)

#define LTR390UV_REG_ALS_DATA_0_R       UINT8_C(0x0D) /* LSB */
#define LTR390UV_REG_ALS_DATA_1_R       UINT8_C(0x0E)
#define LTR390UV_REG_ALS_DATA_2_R       UINT8_C(0x0F) /* MSB */

#define LTR390UV_REG_UVS_DATA_0_R       UINT8_C(0x10) /* LSB */
#define LTR390UV_REG_UVS_DATA_1_R       UINT8_C(0x11)
#define LTR390UV_REG_UVS_DATA_2_R       UINT8_C(0x12) /* MSB */

#define LTR390UV_REG_INT_CFG_RW         UINT8_C(0x19)
#define LTR390UV_REG_INT_PST_RW         UINT8_C(0x1A)

#define LTR390UV_REG_ALS_THRES_UP_0_RW  UINT8_C(0x21) /* LSB */
#define LTR390UV_REG_ALS_THRES_UP_1_RW  UINT8_C(0x22)
#define LTR390UV_REG_ALS_THRES_UP_2_RW  UINT8_C(0x23) /* MSB */

#define LTR390UV_REG_ALS_THRES_LO_0_RW  UINT8_C(0x24) /* LSB */
#define LTR390UV_REG_ALS_THRES_LO_1_RW  UINT8_C(0x25)
#define LTR390UV_REG_ALS_THRES_LO_2_RW  UINT8_C(0x26) /* MSB */

#define LTR390UV_SENSITIVITY_MAX        (2300.0f)       /* see datasheet, section 4.5 */
#define LTR390UV_INTEGRATION_TIME_MAX   (4.0f * 100.0f) /* I2C_LTR390UV_SR_20BIT */
#define LTR390UV_GAIN_MAX               (18.0f)         /* I2C_LTR390UV_MG_X18 */

#define LTR390UV_DATA_POLL_TIMEOUT_MS  UINT16_C(500)
#define LTR390UV_DATA_READY_DELAY_MS   UINT16_C(2)
#define LTR390UV_POWERUP_DELAY_MS      UINT16_C(120)
#define LTR390UV_RESET_DELAY_MS        UINT16_C(25)
#define LTR390UV_WAKEUP_DELAY_MS       UINT16_C(15)
#define LTR390UV_APPSTART_DELAY_MS     UINT16_C(10)    /*!< ltr390uv delay after initialization before application start-up */
#define LTR390UV_CMD_DELAY_MS          UINT16_C(5)     /*!< ltr390uv delay before attempting I2C transactions after a command is issued */
#define LTR390UV_TX_RX_DELAY_MS        UINT16_C(10)    /*!< ltr390uv delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "ltr390uv";


/**
 * @brief LTR390UV I2C write byte to register address transaction.
 * 
 * @param handle LTR390UV device handle.
 * @param reg_addr LTR390UV register address to write to.
 * @param byte LTR390UV write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_i2c_write_byte_to(ltr390uv_handle_t handle, uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief LTR390UV I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle LTR390UV device handle.
 * @param reg_addr LTR390UV register address to read from.
 * @param buffer LTR390UV read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_i2c_read_from(ltr390uv_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "ltr390uv_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief LTR390UV I2C read byte from register address transaction.
 * 
 * @param handle LTR390UV device handle.
 * @param reg_addr LTR390UV register address to read from.
 * @param byte LTR390UV read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_i2c_read_byte_from(ltr390uv_handle_t handle, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ltr390uv_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief LTR390UV I2C write transaction.
 * 
 * @param handle LTR390UV device handle.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_i2c_write(ltr390uv_handle_t handle, const uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Determines resolution factor for LTR390UV from configuration settings.
 * 
 * @param handle LTR390UV device handle.
 * @param resolution_factor LTR390UV resolution factor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_get_resolution_factor(ltr390uv_handle_t handle, float *const resolution_factor) {
    /* validate arguments */
    ESP_ARG_CHECK(handle && resolution_factor);

    ltr390uv_sensor_resolutions_t resolution = handle->dev_config.uvs_sensor_resolution;;

    if(handle->dev_config.operation_mode == LTR390UV_OM_ALS) {
        resolution = handle->dev_config.als_sensor_resolution;
    }

    /* determine resolution */
    switch(resolution) {
        case LTR390UV_SR_20BIT:
            *resolution_factor = 20.0f;
            break;
        case LTR390UV_SR_19BIT:
            *resolution_factor = 19.0f;
            break;
        case LTR390UV_SR_18BIT:
            *resolution_factor = 18.0f;
            break;
        case LTR390UV_SR_17BIT:
            *resolution_factor = 17.0f;
            break;
        case LTR390UV_SR_16BIT:
            *resolution_factor = 16.0f;
            break;
        case LTR390UV_SR_13BIT:
            *resolution_factor = 13.0f;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Determines resolution integration time for LTR390UV in seconds from configuration settings.
 * 
 * @param handle LTR390UV device handle.
 * @param resolution_it LTR390UV resolution integration time.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_get_resolution_it(ltr390uv_handle_t handle, float *const resolution_it) {
    /* validate arguments */
    ESP_ARG_CHECK(handle && resolution_it);

    ltr390uv_sensor_resolutions_t resolution = handle->dev_config.uvs_sensor_resolution;;

    if(handle->dev_config.operation_mode == LTR390UV_OM_ALS) {
        resolution = handle->dev_config.als_sensor_resolution;
    }

    /* determine resolution integration time */
    switch(resolution) {
        case LTR390UV_SR_20BIT:
            *resolution_it = 4.0f;
            break;
        case LTR390UV_SR_19BIT:
            *resolution_it = 2.0f;
            break;
        case LTR390UV_SR_18BIT:
            *resolution_it = 1.0f;
            break;
        case LTR390UV_SR_17BIT:
            *resolution_it = 0.5f;
            break;
        case LTR390UV_SR_16BIT:
            *resolution_it = 0.25f;
            break;
        case LTR390UV_SR_13BIT:
            *resolution_it = 0.125f;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Determines gain multiplier for LTR390UV from configuration settings.
 * 
 * @param handle LTR390UV device handle.
 * @param gain_multiplier LTR390UV gain multiplier.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_get_gain_multiplier(ltr390uv_handle_t handle, float *const gain_multiplier) {
    /* validate arguments */
    ESP_ARG_CHECK(handle && gain_multiplier);

    ltr390uv_measurement_gains_t gain = handle->dev_config.uvs_measurement_gain;

    if(handle->dev_config.operation_mode == LTR390UV_OM_ALS)
        gain = handle->dev_config.als_measurement_gain;

    /* determine gain */
    switch(gain) {
        case LTR390UV_MG_X1:
            *gain_multiplier = 1.0f;
            break;
        case LTR390UV_MG_X3:
            *gain_multiplier = 3.0f;
            break;
        case LTR390UV_MG_X6:
            *gain_multiplier = 6.0f;
            break;
        case LTR390UV_MG_X9:
            *gain_multiplier = 9.0f;
            break;
        case LTR390UV_MG_X18:
            *gain_multiplier = 18.0f;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

/**
 * @brief Calculates UV sensitivity value for LTR390UV from configuration settings.
 * 
 * @param handle LTR390UV device handle.
 * @param uv_sensitivity LTR390UV UV sensitivity value.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_calculate_uv_sensitivity(ltr390uv_handle_t handle, float *const uv_sensitivity) {
    float resolution_it, gain_multiplier;

    /* validate arguments */
    ESP_ARG_CHECK(handle && uv_sensitivity);

    /* attempt to determine gain multiplier */
    ESP_RETURN_ON_ERROR( ltr390uv_get_gain_multiplier(handle, &gain_multiplier), TAG, "read gain multiplier for get uv sensitivity failed" );

    /* attempt to determine resolution integration time */
    ESP_RETURN_ON_ERROR( ltr390uv_get_resolution_it(handle, &resolution_it), TAG, "read resolution integration time for get uv sensitivity failed" );

    /* set sensitivity by linearly scaling against known value in the datasheet */
    float gain_scale = gain_multiplier / LTR390UV_GAIN_MAX;
    float intg_scale = (resolution_it * 100.0f) / LTR390UV_INTEGRATION_TIME_MAX;
    *uv_sensitivity  = LTR390UV_SENSITIVITY_MAX * gain_scale * intg_scale;

    return ESP_OK;
}

/**
 * @brief Reads sensor counts based on configured mode from LTR390UV.
 * 
 * @param handle LTR390UV device handle.
 * @param sensor_counts LTR390UV sensor als or uvs counts based on configured mode.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ltr390uv_get_sensor_counts(ltr390uv_handle_t handle, uint32_t *const sensor_counts) {
    /* validate arguments */
    ESP_ARG_CHECK(handle && sensor_counts);

    /* initialize local variables */
    esp_err_t    ret           = ESP_OK;
    uint64_t     start_time    = esp_timer_get_time(); /* set start time for timeout monitoring */
    bit24_uint8_buffer_t rx    = { 0 };
    bool         data_is_ready = false;

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ltr390uv_get_data_status(handle, &data_is_ready), err, TAG, "data ready ready for get light counts failed." );
        
        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(LTR390UV_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, LTR390UV_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* validate operation mode */
    switch (handle->dev_config.operation_mode) {
        case LTR390UV_OM_ALS:
            /* attempt i2c read transaction */
            ESP_GOTO_ON_ERROR( ltr390uv_i2c_read_from(handle, LTR390UV_REG_ALS_DATA_0_R, rx, BIT24_UINT8_BUFFER_SIZE), err, TAG, "read als counts failed" );
            break;
        case LTR390UV_OM_UVS:
            /* attempt i2c read transaction */
            ESP_GOTO_ON_ERROR( ltr390uv_i2c_read_from(handle, LTR390UV_REG_UVS_DATA_0_R, rx, BIT24_UINT8_BUFFER_SIZE), err, TAG, "read uvs counts failed" );
            break;
        default:
            ESP_LOGE(TAG, "Invalid operation mode");
            return ESP_ERR_INVALID_ARG;
    }

    /* concat values */
    *sensor_counts = rx[2] * 65536 + rx[1] * 256 + rx[0];  // (rx[2] << 16)| (rx[1] << 8) | rx[0]

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ltr390uv_get_control_register(ltr390uv_handle_t handle, ltr390uv_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_byte_from(handle, LTR390UV_REG_MAIN_CTRL_RW, &reg->reg), TAG, "read control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_set_control_register(ltr390uv_handle_t handle, const ltr390uv_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy and initialize register */
    ltr390uv_control_register_t ctrl = { .reg = reg.reg };
    ctrl.bits.reserved1 = 0;
    ctrl.bits.reserved2 = 0;
    ctrl.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_write_byte_to(handle, LTR390UV_REG_MAIN_CTRL_RW, ctrl.reg), TAG, "write control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_get_measure_register(ltr390uv_handle_t handle, ltr390uv_measure_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_byte_from(handle, LTR390UV_REG_ALS_UVS_MEAS_RW, &reg->reg), TAG, "read measure register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_set_measure_register(ltr390uv_handle_t handle, const ltr390uv_measure_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy and initialize register */
    ltr390uv_measure_register_t msr = { .reg = reg.reg };
    msr.bits.reserved1 = 0;
    msr.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_write_byte_to(handle, LTR390UV_REG_ALS_UVS_MEAS_RW, msr.reg), TAG, "write measure register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_get_gain_register(ltr390uv_handle_t handle, ltr390uv_gain_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_byte_from(handle, LTR390UV_REG_ALS_UVS_GAIN_RW, &reg->reg), TAG, "read gain register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_set_gain_register(ltr390uv_handle_t handle, const ltr390uv_gain_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy and initialize register */
    ltr390uv_gain_register_t gain = { .reg = reg.reg };
    gain.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_write_byte_to(handle, LTR390UV_REG_ALS_UVS_GAIN_RW, gain.reg), TAG, "write gain register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_get_interrupt_config_register(ltr390uv_handle_t handle, ltr390uv_interrupt_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_byte_from(handle, LTR390UV_REG_INT_CFG_RW, &reg->reg), TAG, "read interrupt configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_set_interrupt_config_register(ltr390uv_handle_t handle, const ltr390uv_interrupt_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy and initialize register */
    ltr390uv_interrupt_config_register_t irq = { .reg = reg.reg };
    irq.bits.reserved1 = 0;
    irq.bits.reserved2 = 0;
    irq.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_write_byte_to(handle, LTR390UV_REG_INT_CFG_RW, irq.reg), TAG, "write interrupt configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_get_status_register(ltr390uv_handle_t handle, ltr390uv_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_byte_from(handle, LTR390UV_REG_MAIN_STS_R, &reg->reg), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_init(i2c_master_bus_handle_t master_handle, const ltr390uv_config_t *ltr390uv_config, ltr390uv_handle_t *ltr390uv_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && ltr390uv_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ltr390uv_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ltr390uv device handle initialization failed", ltr390uv_config->i2c_address);

    /* validate memory availability for handle */
    ltr390uv_handle_t out_handle;
    out_handle = (ltr390uv_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ltr390uv device, init failed");

    /* copy configuration */
    out_handle->dev_config = *ltr390uv_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_CMD_DELAY_MS));

    /* attempt soft-reset */
    ESP_GOTO_ON_ERROR(ltr390uv_reset(out_handle), err_handle, TAG, "soft-reset for init failed");

    /* set device handle */
    *ltr390uv_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_APPSTART_DELAY_MS));

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

esp_err_t ltr390uv_get_ambient_light(ltr390uv_handle_t handle, float *const ambient_light) {
    uint32_t counts;
    float gain_multiplier, resolution_it;

    /* validate arguments */
    ESP_ARG_CHECK(handle && ambient_light);

    /* validate operation mode */
    if(handle->dev_config.operation_mode != LTR390UV_OM_ALS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( ltr390uv_set_mode(handle, LTR390UV_OM_ALS), TAG, "write operation mode for get ambient light failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_sensor_counts(handle, &counts), TAG, "read light counts for get ambient light failed" );

    /* attempt to determine gain multiplier */
    ESP_RETURN_ON_ERROR( ltr390uv_get_gain_multiplier(handle, &gain_multiplier), TAG, "read gain multiplier for get ambient light failed" );

    /* attempt to determine resolution integration time */
    ESP_RETURN_ON_ERROR( ltr390uv_get_resolution_it(handle, &resolution_it), TAG, "read resolution integration time for get ambient light failed" );

    /* convert light counts to lux */
    *ambient_light = ((0.6f * counts) / (gain_multiplier * resolution_it)) * handle->dev_config.window_factor;

    return ESP_OK;
}

esp_err_t ltr390uv_get_als(ltr390uv_handle_t handle, uint32_t *const sensor_counts) {
    uint32_t counts;

    /* validate arguments */
    ESP_ARG_CHECK(handle && sensor_counts);

    /* validate operation mode */
    if(handle->dev_config.operation_mode != LTR390UV_OM_ALS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( ltr390uv_set_mode(handle, LTR390UV_OM_ALS), TAG, "write operation mode for get als failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_sensor_counts(handle, &counts), TAG, "read light counts for get als failed" );

    /* set output parameter */
    *sensor_counts = counts;

    return ESP_OK;
}

esp_err_t ltr390uv_get_ultraviolet_index(ltr390uv_handle_t handle, float *const ultraviolet_index) {
    uint32_t counts;
    float uv_sensitivity;

    /* validate arguments */
    ESP_ARG_CHECK(handle && ultraviolet_index);

    /* validate operation mode */
    if(handle->dev_config.operation_mode != LTR390UV_OM_UVS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( ltr390uv_set_mode(handle, LTR390UV_OM_UVS), TAG, "write operation mode for get ultraviolet index failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_sensor_counts(handle, &counts), TAG, "read light counts for get ultraviolet index failed" );

    /* attempt to determine uv sensitivity */
    ESP_RETURN_ON_ERROR( ltr390uv_calculate_uv_sensitivity(handle, &uv_sensitivity), TAG, "read uv sensitivity for get ultraviolet index failed" );

    /* convert light counts to uvi */
    *ultraviolet_index = (counts / uv_sensitivity) * handle->dev_config.window_factor;

    return ESP_OK;
}

esp_err_t ltr390uv_get_uvs(ltr390uv_handle_t handle, uint32_t *const sensor_counts) {
    uint32_t counts;

    /* validate arguments */
    ESP_ARG_CHECK(handle && sensor_counts);

    /* validate operation mode */
    if(handle->dev_config.operation_mode != LTR390UV_OM_UVS) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( ltr390uv_set_mode(handle, LTR390UV_OM_UVS), TAG, "write operation mode for get uvs failed" );
    }

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_sensor_counts(handle, &counts), TAG, "read light counts for get uvs failed" );

    /* set output parameter */
    *sensor_counts = counts;

    return ESP_OK;
}

esp_err_t ltr390uv_get_data_status(ltr390uv_handle_t handle, bool *const ready) {
    ltr390uv_status_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_status_register(handle, &reg), TAG, "read status register for get data status failed" );

    /* set output parameter */
    *ready = reg.bits.data_status;

    return ESP_OK;
}

esp_err_t ltr390uv_get_power_status(ltr390uv_handle_t handle, bool *const power_on) {
    ltr390uv_status_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_status_register(handle, &reg), TAG, "read status register for get power status failed" );

    /* set output parameter */
    *power_on = reg.bits.power_status;

    return ESP_OK;
}

esp_err_t ltr390uv_get_interrupt_status(ltr390uv_handle_t handle, bool *const interrupt) {
    ltr390uv_status_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_status_register(handle, &reg), TAG, "read status register for get interrupt status failed" );

    /* set output parameter */
    *interrupt = reg.bits.irq_status;

    return ESP_OK;
}

esp_err_t ltr390uv_get_status(ltr390uv_handle_t handle, bool *const data_ready, bool *const power_on, bool *const interrupt) {
    ltr390uv_status_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_status_register(handle, &reg), TAG, "read status register for get status failed" );

    /* set output parameter */
    *data_ready = reg.bits.data_status;
    *power_on   = reg.bits.power_status;
    *interrupt  = reg.bits.irq_status;

    return ESP_OK;
}

esp_err_t ltr390uv_get_thresholds(ltr390uv_handle_t handle, uint32_t *const lower_threshold, uint32_t *const upper_threshold) {
    bit24_uint8_buffer_t lower, upper;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transactions */
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_from(handle, LTR390UV_REG_ALS_THRES_LO_0_RW, lower, BIT24_UINT8_BUFFER_SIZE), TAG, "read lower threshold failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_read_from(handle, LTR390UV_REG_ALS_THRES_UP_0_RW, upper, BIT24_UINT8_BUFFER_SIZE), TAG, "read upper threshold failed" );

    /* set output parameters */  
    *lower_threshold = (lower[2] << 16) | (lower[1] << 8) | lower[0];
    *upper_threshold = (upper[2] << 16) | (upper[1] << 8) | upper[0];

    return ESP_OK;
}

esp_err_t ltr390uv_set_thresholds(ltr390uv_handle_t handle, const uint32_t lower_threshold, const uint32_t upper_threshold) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    bit32_uint8_buffer_t lower;
    lower[0] = LTR390UV_REG_ALS_THRES_LO_0_RW;
    lower[1] = lower_threshold & 0x000000FF; // lsb
    lower[2] = lower_threshold >> 8;
    lower[3] = lower_threshold >> 16;

    bit32_uint8_buffer_t upper;
    upper[0] = LTR390UV_REG_ALS_THRES_UP_0_RW;
    upper[1] = upper_threshold & 0x000000FF; // lsb
    upper[2] = upper_threshold >> 8;
    upper[3] = upper_threshold >> 16;

    ESP_RETURN_ON_ERROR( ltr390uv_i2c_write(handle, lower, BIT32_UINT8_BUFFER_SIZE), TAG, "write lower threshold failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_i2c_write(handle, upper, BIT32_UINT8_BUFFER_SIZE), TAG, "write upper threshold failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_get_mode(ltr390uv_handle_t handle, ltr390uv_operation_modes_t *const mode) {
    ltr390uv_control_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_control_register(handle, &reg), TAG, "read control register for get mode failed" );

    /* set output parameter */
    *mode = reg.bits.operation_mode;

    return ESP_OK;
}

esp_err_t ltr390uv_set_mode(ltr390uv_handle_t handle, const ltr390uv_operation_modes_t mode) {
    ltr390uv_control_register_t c_reg;
    ltr390uv_interrupt_config_register_t ic_reg;
    ltr390uv_measure_register_t m_reg;
    ltr390uv_gain_register_t    g_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_measure_register(handle, &m_reg), TAG, "read measure register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_get_gain_register(handle, &g_reg), TAG, "read gain register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_get_interrupt_config_register(handle, &ic_reg), TAG, "read interrupt configuration register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_get_control_register(handle, &c_reg), TAG, "read register for reset failed" );

    /* attempt device configuration */
    c_reg.bits.operation_mode    = mode;
    ic_reg.bits.irq_enabled      = true;
    ic_reg.bits.irq_light_source = LTR390UV_LSI_ALS;
    m_reg.bits.sensor_resolution = handle->dev_config.als_sensor_resolution;
    m_reg.bits.measurement_rate  = handle->dev_config.als_measurement_rate;
    g_reg.bits.measurement_gain  = handle->dev_config.als_measurement_gain;
    handle->dev_config.operation_mode = LTR390UV_OM_ALS;
    if(mode == LTR390UV_OM_UVS) {
        ic_reg.bits.irq_light_source = LTR390UV_LSI_UVS;
        m_reg.bits.sensor_resolution = handle->dev_config.uvs_sensor_resolution;
        m_reg.bits.measurement_rate  = handle->dev_config.uvs_measurement_rate;
        g_reg.bits.measurement_gain  = handle->dev_config.uvs_measurement_gain;
        handle->dev_config.operation_mode = LTR390UV_OM_UVS;
    }

    /* attempt i2c write transactions */
    ESP_RETURN_ON_ERROR( ltr390uv_set_measure_register(handle, m_reg), TAG, "write measure register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_set_gain_register(handle, g_reg), TAG, "write gain register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_set_interrupt_config_register(handle, ic_reg), TAG, "write interrupt configuration register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_set_control_register(handle, c_reg), TAG, "write control register for reset failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_get_resolution(ltr390uv_handle_t handle, ltr390uv_sensor_resolutions_t *const resolution) {
    ltr390uv_measure_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_measure_register(handle, &reg), TAG, "read measure register for get resolution failed" );

    /* set output parameter */
    *resolution = reg.bits.sensor_resolution;

    return ESP_OK;
}

esp_err_t ltr390uv_set_resolution(ltr390uv_handle_t handle, const ltr390uv_sensor_resolutions_t resolution) {
    ltr390uv_measure_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_measure_register(handle, &reg), TAG, "read measure register for get resolution failed" );

    /* set parameter */
    reg.bits.sensor_resolution = resolution;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_measure_register(handle, reg), TAG, "write measure register for set resolution failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_get_gain(ltr390uv_handle_t handle, ltr390uv_measurement_gains_t *const gain) {
    ltr390uv_gain_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_gain_register(handle, &reg), TAG, "read gain register for get gain failed" );

    /* set output parameter */
    *gain = reg.bits.measurement_gain;

    return ESP_OK;
}

esp_err_t ltr390uv_set_gain(ltr390uv_handle_t handle, const ltr390uv_measurement_gains_t gain) {
    ltr390uv_gain_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_gain_register(handle, &reg), TAG, "read gain register for get gain failed" );

    /* set parameter */
    reg.bits.measurement_gain = gain;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_gain_register(handle, reg), TAG, "write gain register for set gain failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_get_rate(ltr390uv_handle_t handle, ltr390uv_measurement_rates_t *const rate) {
    ltr390uv_measure_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_measure_register(handle, &reg), TAG, "read measure register for get rate failed" );

    /* set output parameter */
    *rate = reg.bits.measurement_rate;

    return ESP_OK;
}

esp_err_t ltr390uv_set_rate(ltr390uv_handle_t handle, const ltr390uv_measurement_rates_t rate) {
    ltr390uv_measure_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_measure_register(handle, &reg), TAG, "read measure register for get rate failed" );

    /* set parameter */
    reg.bits.measurement_rate = rate;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_measure_register(handle, reg), TAG, "write measure register for set rate failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_enable_interrupt(ltr390uv_handle_t handle, const ltr390uv_ls_interrupts_t light_source) {
    ltr390uv_interrupt_config_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_interrupt_config_register(handle, &reg), TAG, "read interrupt configuration register for enable interrupt failed" );

    /* set parameters */
    reg.bits.irq_enabled      = true;
    reg.bits.irq_light_source = light_source;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_interrupt_config_register(handle, reg), TAG, "write interrupt configuration register for enable interrupt failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_disable_interrupt(ltr390uv_handle_t handle) {
    ltr390uv_interrupt_config_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_interrupt_config_register(handle, &reg), TAG, "read interrupt configuration register for disable interrupt failed" );

    /* disable irq */
    reg.bits.irq_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_interrupt_config_register(handle, reg), TAG, "write interrupt configuration register for disable interrupt failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_enable(ltr390uv_handle_t handle) {
    ltr390uv_control_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_control_register(handle, reg), TAG, "read control register for enable failed" );

    /* enable sensor */
    reg.bits.sensor_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_control_register(handle, reg), TAG, "write control register for enable failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_WAKEUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_disable(ltr390uv_handle_t handle) {
    ltr390uv_control_register_t reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_control_register(handle, &reg), TAG, "read control register for disable failed" );

    /* disable sensor */
    reg.bits.sensor_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_control_register(handle, reg), TAG, "write control register for disable failed" );

    return ESP_OK;
}

esp_err_t ltr390uv_reset(ltr390uv_handle_t handle) {
    ltr390uv_control_register_t c_reg;
    ltr390uv_interrupt_config_register_t ic_reg;
    ltr390uv_measure_register_t m_reg;
    ltr390uv_gain_register_t    g_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_control_register(handle, &c_reg), TAG, "read control register for reset failed" );

    /* reset device */
    c_reg.bits.software_reset_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_set_control_register(handle, c_reg), TAG, "write control register for reset failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_RESET_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ltr390uv_get_measure_register(handle, &m_reg), TAG, "read measure register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_get_gain_register(handle, &g_reg), TAG, "read gain register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_get_interrupt_config_register(handle, &ic_reg), TAG, "read interrupt configuration register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_get_control_register(handle, &c_reg), TAG, "write read register for reset failed" );

    /* attempt device configuration */
    c_reg.bits.sensor_enabled    = true;
    ic_reg.bits.irq_enabled      = true;
    ic_reg.bits.irq_light_source = LTR390UV_LSI_ALS;
    m_reg.bits.sensor_resolution = handle->dev_config.als_sensor_resolution;
    m_reg.bits.measurement_rate  = handle->dev_config.als_measurement_rate;
    g_reg.bits.measurement_gain  = handle->dev_config.als_measurement_gain;
    if(handle->dev_config.operation_mode == LTR390UV_OM_UVS) {
        ic_reg.bits.irq_light_source = LTR390UV_LSI_UVS;
        m_reg.bits.sensor_resolution = handle->dev_config.uvs_sensor_resolution;
        m_reg.bits.measurement_rate  = handle->dev_config.uvs_measurement_rate;
        g_reg.bits.measurement_gain  = handle->dev_config.uvs_measurement_gain;
    }

    /* attempt i2c write transactions */
    ESP_RETURN_ON_ERROR( ltr390uv_set_measure_register(handle, m_reg), TAG, "write measure register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_set_gain_register(handle, g_reg), TAG, "write gain register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_set_interrupt_config_register(handle, ic_reg), TAG, "write interrupt configuration register for reset failed" );
    ESP_RETURN_ON_ERROR( ltr390uv_set_control_register(handle, c_reg), TAG, "write control register for reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(LTR390UV_WAKEUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t ltr390uv_remove(ltr390uv_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t ltr390uv_delete(ltr390uv_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( ltr390uv_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* ltr390uv_get_fw_version(void) {
    return LTR390UV_FW_VERSION_STR;
}

int32_t ltr390uv_get_fw_version_number(void) {
    return LTR390UV_FW_VERSION_INT32;
}