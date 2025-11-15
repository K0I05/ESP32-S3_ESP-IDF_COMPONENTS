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
 * @file mmc56x3.c
 *
 * ESP-IDF driver for MMC56X3 Magnetic sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/mmc56x3.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * MMC56X3 definitions
*/

#define MMC56X3_REG_XOUT_0_R            UINT8_C(0x00)   //!< mmc56x3 I2C Xout[19:12]
#define MMC56X3_REG_XOUT_1_R            UINT8_C(0x01)   //!< mmc56x3 I2C Xout[11:4]
#define MMC56X3_REG_YOUT_0_R            UINT8_C(0x02)   //!< mmc56x3 I2C Yout[19:12]
#define MMC56X3_REG_YOUT_1_R            UINT8_C(0x03)   //!< mmc56x3 I2C Yout[11:4]
#define MMC56X3_REG_ZOUT_0_R            UINT8_C(0x04)   //!< mmc56x3 I2C Zout[19:12]
#define MMC56X3_REG_ZOUT_1_R            UINT8_C(0x05)   //!< mmc56x3 I2C Zout[11:4]
#define MMC56X3_REG_XOUT_2_R            UINT8_C(0x06)   //!< mmc56x3 I2C Xout[3:0]
#define MMC56X3_REG_YOUT_2_R            UINT8_C(0x07)   //!< mmc56x3 I2C Yout[3:0]
#define MMC56X3_REG_ZOUT_2_R            UINT8_C(0x08)   //!< mmc56x3 I2C Zout[3:0]
#define MMC56X3_REG_TOUT_R              UINT8_C(0x09)   //!< mmc56x3 I2C temperature output
#define MMC56X3_REG_STATUS_1_R          UINT8_C(0x18)   //!< mmc56x3 I2C device status 1
#define MMC56X3_REG_ODR_W               UINT8_C(0x1a)   //!< mmc56x3 I2C output data rate
#define MMC56X3_REG_CONTROL_0_W         UINT8_C(0x1b)   //!< mmc56x3 I2C control register 0
#define MMC56X3_REG_CONTROL_1_W         UINT8_C(0x1c)   //!< mmc56x3 I2C control register 1
#define MMC56X3_REG_CONTROL_2_W         UINT8_C(0x1d)   //!< mmc56x3 I2C control register 2
#define MMC56X3_REG_ST_X_TH_W           UINT8_C(0x1e)   //!< mmc56x3 I2C x-axis selftest threshold
#define MMC56X3_REG_ST_Y_TH_W           UINT8_C(0x1f)   //!< mmc56x3 I2C y-axis selftest threshold
#define MMC56X3_REG_ST_Z_TH_W           UINT8_C(0x20)   //!< mmc56x3 I2C z-axis selftest threshold
#define MMC56X3_REG_ST_X_SV_RW          UINT8_C(0x27)   //!< mmc56x3 I2C x-axis selftest set-value
#define MMC56X3_REG_ST_Y_SV_RW          UINT8_C(0x28)   //!< mmc56x3 I2C y-axis selftest set-value
#define MMC56X3_REG_ST_Z_SV_RW          UINT8_C(0x29)   //!< mmc56x3 I2C z-axis selftest set-value
#define MMC56X3_REG_PRODUCT_ID_R        UINT8_C(0x39)   //!< mmc56x3 I2C product identifier


#define MMC56X3_POWERUP_DELAY_MS        UINT16_C(50)
#define MMC56X3_APPSTART_DELAY_MS       UINT16_C(10)           //!< mmc56x3 I2C delay in milliseconds app-start
#define MMC56X3_RESET_DELAY_MS          UINT16_C(50)           //!< mmc56x3 I2C delay in milliseconds after reset
#define MMC56X3_SETRESET_DELAY_MS       UINT16_C(1)            //!< mmc56x3 I2C delay in milliseconds after set-reset transaction
#define MMC56X3_WRITE_DELAY_MS          UINT16_C(1)            //!< mmc56x3 I2C delay in milliseconds after write transaction
#define MMC56X3_DATA_READY_DELAY_MS     UINT16_C(1)            //!< mmc56x3 1ms when checking data ready in a loop
#define MMC56X3_DATA_POLL_TIMEOUT_MS    UINT16_C(100)          //!< mmc56x3 100ms timeout when making a measurement
#define MMC56X3_CMD_DELAY_MS            UINT16_C(5)
#define MMC56X3_TX_RX_DELAY_MS          UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief MMC56X3 device descriptor structure definition.
 */
typedef struct mmc56x3_device_s {
    mmc56x3_config_t                config;
    i2c_master_dev_handle_t         i2c_handle;         /*!< I2C device handle */
    uint8_t                         product_id;             /*!< mmc56x3 product identifier */
} mmc56x3_device_t;

/*
* static constant declarations
*/
static const char *TAG = "mmc56x3";

/*
* functions and subroutines
*/



/**
 * @brief MMC56X3 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg_addr MMC56X3 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_read_from(mmc56x3_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "mmc56x3_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL read byte from register address transaction.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg_addr MMC56X3 register address to read from.
 * @param byte MMC56X3 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_read_byte_from(mmc56x3_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "mmc56x3_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL write byte to register address transaction.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg_addr MMC56X3 register address to write to.
 * @param byte MMC56X3 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_write_byte_to(mmc56x3_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL read status register.
 * 
 * @param device MMC56X3 device descriptor.
 * @param[out] reg MMC56X3 status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_get_status_register(mmc56x3_device_t *const device, mmc56x3_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_read_byte_from(device, MMC56X3_REG_STATUS_1_R, &reg->reg), TAG, "mmc56x3 read status register failed" );
    
    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL write control 0 register.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg MMC56X3 control 0 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_set_control0_register(mmc56x3_device_t *const device, const mmc56x3_control0_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set reserved to 0 */
    mmc56x3_control0_register_t control0 = { .reg = reg.reg };
    control0.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_CONTROL_0_W, control0.reg), TAG, "write control 0 register failed" );

    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL write control 1 register.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg MMC56X3 control 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_set_control1_register(mmc56x3_device_t *const device, const mmc56x3_control1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    mmc56x3_control1_register_t control1 = { .reg = reg.reg };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_CONTROL_1_W, control1.reg), TAG, "write control 1 register failed" );

    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL write control 2 register.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg MMC56X3 control 2 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_set_control2_register(mmc56x3_device_t *const device, const mmc56x3_control2_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set reserved to 0 */
    mmc56x3_control2_register_t control2 = { .reg = reg.reg };
    control2.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_CONTROL_2_W, control2.reg), TAG, "write control 2 register failed" );

    return ESP_OK;
}

/**
 * @brief MMC56X3 I2C HAL read product identifier register.
 * 
 * @param device MMC56X3 device descriptor.
 * @param reg MMC56X3 product id register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mmc56x3_i2c_get_product_id_register(mmc56x3_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_read_byte_from(device, MMC56X3_REG_PRODUCT_ID_R, reg), TAG, "mmc56x3 read product identifier register failed" );
    
    return ESP_OK;
}

esp_err_t mmc56x3_i2c_set_reset(mmc56x3_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set register soft-reset to true */
    mmc56x3_control1_register_t ctrl1 = { .bits.sw_reset = true };

    /* attempt control 1 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control1_register(device, ctrl1), TAG, "write control 1 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MMC56X3_RESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t mmc56x3_i2c_setup_registers(mmc56x3_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set register soft-reset to true */
    mmc56x3_control0_register_t ctrl0 = { 0 };
    mmc56x3_control1_register_t ctrl1 = { 0 };
    mmc56x3_control2_register_t ctrl2 = { 0 };

    ctrl0.bits.do_set   = true; // turn on set bit
    ctrl0.bits.do_reset = true; // turn on reset bit

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MMC56X3_SETRESET_DELAY_MS));

    /* validate data rate if continuous mode is enabled */
    if(device->config.continuous_mode_enabled == true) {
        ESP_RETURN_ON_FALSE(device->config.data_rate > 0, ESP_ERR_INVALID_ARG, TAG, "data rate (odr) must be non-zero in continuous measurement mode, set mode failed");
    }

    ctrl0.bits.do_set                  = false; // turn off set bit
    ctrl0.bits.do_reset                = false; // turn off reset bit
    ctrl0.bits.auto_sr_enabled         = device->config.auto_sr_enabled;
    ctrl0.bits.continuous_freq_enabled = false; // turn off cmm_freq_en bit
    ctrl2.bits.continuous_enabled      = false; // turn off cmm_en bit

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register, setup registers failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register, setup registers failed" );

    ctrl1.bits.bandwidth = device->config.measurement_bandwidth;

    if(device->config.continuous_mode_enabled == true) {
        ctrl0.bits.continuous_freq_enabled = true; // turn on cmm_freq_en bit
        ctrl2.bits.continuous_enabled      = true; // turn on cmm_en bit
    }

    /* only 0~255 and 1000 are valid, so just move any high rates to 1000 */
    if(device->config.data_rate > 255) {
        /* set odr range */
        uint8_t odr = 255;

        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ODR_W, odr), TAG, "write odr register failed, setup registers failed" );

        /* enable hpower */
        ctrl2.bits.h_power_enabled = true;
    } else {
        /* set odr range */
        uint8_t odr = (uint8_t)device->config.data_rate;

        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ODR_W, odr), TAG, "write odr register failed, setup registers failed" );

        /* disable hpower */
        ctrl2.bits.h_power_enabled = false;
    }

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register, setup registers failed" );

    /* attempt control 1 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control1_register(device, ctrl1), TAG, "write control 1 register, setup registers mode failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register, setup registers mode failed" );

    return ESP_OK;
}

esp_err_t mmc56x3_init(i2c_master_bus_handle_t master_handle, const mmc56x3_config_t *mmc56x3_config, mmc56x3_handle_t *mmc56x3_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && mmc56x3_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MMC56X3_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, mmc56x3_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, mmc56x3 device handle initialization failed", mmc56x3_config->i2c_address);

    /* validate data rate if continuous mode is enabled */
    if(mmc56x3_config->continuous_mode_enabled == true) {
        ESP_GOTO_ON_FALSE(mmc56x3_config->data_rate > 0, ESP_ERR_INVALID_ARG, err, TAG, "data rate (odr) must be non-zero in continuous measurement mode, init failed");
    }

    /* validate memory availability for handle */
    mmc56x3_device_t* device = (mmc56x3_device_t*)calloc(1, sizeof(mmc56x3_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mmc56x3 device, init failed");

    /* copy configuration */
    device->config = *mmc56x3_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MMC56X3_CMD_DELAY_MS));

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR( mmc56x3_i2c_set_reset(device), err_handle, TAG, "unable to issue soft-reset, init failed" );

    /* attempt to setup the device */
    ESP_GOTO_ON_ERROR( mmc56x3_i2c_setup_registers(device), err_handle, TAG, "unable to setup device, init failed" );

    /* set device handle */
    *mmc56x3_handle = (mmc56x3_handle_t)device;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MMC56X3_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t mmc56x3_get_temperature(mmc56x3_handle_t handle, float *const temperature) {
    esp_err_t ret             = ESP_OK;
    uint64_t  start_time      = 0;
    bool      data_is_ready   = false;
    uint8_t   temp_reg;
    mmc56x3_device_t* device  = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate mode */
    if(device->config.continuous_mode_enabled == false) {
        /* register */
        mmc56x3_control0_register_t ctrl0;
        
        /* trigger temperature measurement */
        ctrl0.bits.sample_t = true;

        /* attempt to write control 0 register */
        ESP_GOTO_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), err, TAG, "write magnetic sample trigger for get magnetic failed." );
    }

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
    do {
        mmc56x3_status_register_t status = { 0 };

        /* attempt to read status register  */
        ESP_GOTO_ON_ERROR( mmc56x3_i2c_get_status_register(device, &status), err, TAG, "read status register for get temperature data status failed" );

        /* set ready state */
        data_is_ready = status.bits.data_ready_t;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(MMC56X3_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (MMC56X3_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c data read transactions */
    ESP_GOTO_ON_ERROR( mmc56x3_i2c_read_byte_from(device, MMC56X3_REG_TOUT_R, &temp_reg), err, TAG, "read temperature failed" );

    *temperature = temp_reg;
    *temperature *= 0.8f;   // 0.8C / LSB
    *temperature -= 75.0f;  // 0 value is -75

    return ESP_OK;

    err:
        return ret;
}

esp_err_t mmc56x3_get_magnetic_axes(mmc56x3_handle_t handle, mmc56x3_magnetic_axes_data_t *const axes_data) {
    esp_err_t                       ret           = ESP_OK;
    uint64_t                        start_time    = 0;
    bool                            data_is_ready = false;
    mmc56x3_control0_register_t     ctrl0         = { 0 };
    bit72_uint8_buffer_t            rx            = { 0 };
    mmc56x3_device_t*               device        = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate mode */
    if(device->config.continuous_mode_enabled == false) {
        /* trigger magnetic measurement */
        ctrl0.bits.sample_m = true;

        /* attempt to write control 0 register */
        ESP_GOTO_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), err, TAG, "write magnetic sample trigger for get magnetic axes failed." );
    }

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
    do {
        mmc56x3_status_register_t status = { 0 };

        /* attempt to read status register  */
        ESP_GOTO_ON_ERROR( mmc56x3_i2c_get_status_register(device, &status), err, TAG, "read status register for get magnetic data status failed" );

        /* set ready state */
        data_is_ready = status.bits.data_ready_m;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(MMC56X3_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (MMC56X3_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c data write-read transactions */
    ESP_GOTO_ON_ERROR( mmc56x3_i2c_read_from(device, MMC56X3_REG_XOUT_0_R, rx, BIT72_UINT8_BUFFER_SIZE), err, TAG, "read magnetic axes failed" );

    // convert bytes (20-bit) to int32_t
    const int32_t x_axis = ((uint32_t)rx[0] << 12) | ((uint32_t)rx[1] << 4) | ((uint32_t)rx[6] >> 4);
    const int32_t y_axis = ((uint32_t)rx[2] << 12) | ((uint32_t)rx[3] << 4) | ((uint32_t)rx[7] >> 4);
    const int32_t z_axis = ((uint32_t)rx[4] << 12) | ((uint32_t)rx[5] << 4) | ((uint32_t)rx[8] >> 4);

    // scale to mG by LSB (0.0625mG per LSB resolution) in datasheet (20-bit 524288 counts)
    axes_data->x_axis = (float)(x_axis - 524288) * 0.0625f;
    axes_data->y_axis = (float)(y_axis - 524288) * 0.0625f;
    axes_data->z_axis = (float)(z_axis - 524288) * 0.0625f;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t mmc56x3_get_magnetic_data_status(mmc56x3_handle_t handle, bool *const ready) {
    mmc56x3_status_register_t status;
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_get_status_register(device, &status), TAG, "read status register for get magnetic data status failed" );

    /* set ready state */
    *ready = status.bits.data_ready_m;

    return ESP_OK;
}

esp_err_t mmc56x3_get_temperature_data_status(mmc56x3_handle_t handle, bool *const ready) {
    mmc56x3_status_register_t status;
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_get_status_register(device, &status), TAG, "read status register for get temperature data status failed" );

    /* set ready state */
    *ready = status.bits.data_ready_t;

    return ESP_OK;
}

esp_err_t mmc56x3_get_data_status(mmc56x3_handle_t handle, bool *const magnetic_ready, bool *const temperature_ready) {
    mmc56x3_status_register_t status;
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register  */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_get_status_register(device, &status), TAG, "read status register for get temperature data status failed" );

    /* set ready state */
    *magnetic_ready     = status.bits.data_ready_m;
    *temperature_ready  = status.bits.data_ready_t;

    return ESP_OK;
}

esp_err_t mmc56x3_set_measure_mode(mmc56x3_handle_t handle, const bool continuous) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate data rate if continuous mode is enabled */
    if(continuous == true) {
        ESP_RETURN_ON_FALSE(device->config.data_rate > 0, ESP_ERR_INVALID_ARG, TAG, "data rate (odr) must be non-zero in continuous measurement mode, set mode failed");
    }

    /* registers */
    mmc56x3_control0_register_t ctrl0;
    mmc56x3_control2_register_t ctrl2;

    ctrl0.bits.auto_sr_enabled = device->config.auto_sr_enabled;

    if(continuous == true) {
        ctrl0.bits.continuous_freq_enabled  = true; // turn on cmm_freq_en bit
        ctrl2.bits.continuous_enabled       = true; // turn on cmm_en bit
    } else {
        ctrl0.bits.continuous_freq_enabled  = false; // turn off cmm_freq_en bit
        ctrl2.bits.continuous_enabled       = false; // turn off cmm_en bit
    }

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register, set mode failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register, set mode failed" );

    device->config.continuous_mode_enabled = continuous;

    return ESP_OK;
}

esp_err_t mmc56x3_set_data_rate(mmc56x3_handle_t handle, const uint16_t rate) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* register */
    mmc56x3_control2_register_t ctrl2;

    /* only 0~255 and 1000 are valid, so just move any high rates to 1000 */
    if(rate > 255) {
        /* set odr range */
        uint8_t odr = 255;

        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ODR_W, odr), TAG, "write odr register failed" );

        /* enable hpower */
        ctrl2.bits.h_power_enabled = true;

        /* attempt control 2 register write */
        ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register failed" );
    } else {
        /* set odr range */
        uint8_t odr = (uint8_t)rate;

        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ODR_W, odr), TAG, "write odr register failed" );

        /* disable hpower */
        ctrl2.bits.h_power_enabled = false;

        /* attempt control 2 register write */
        ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register failed" );
    }

    device->config.data_rate = rate;

    return ESP_OK;
}

esp_err_t mmc56x3_set_measure_bandwidth(mmc56x3_handle_t handle, const mmc56x3_measurement_times_t bandwidth) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    mmc56x3_control1_register_t ctrl1;
    ctrl1.bits.bandwidth = bandwidth;

    /* attempt control 1 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control1_register(device, ctrl1), TAG, "unable to write control 1 register, set measure bandwidth failed" );

    device->config.measurement_bandwidth = bandwidth;

    return ESP_OK;
}

esp_err_t mmc56x3_enable_periodical_set(mmc56x3_handle_t handle, const mmc56x3_measurement_samples_t samples) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy registers */
    mmc56x3_control0_register_t ctrl0;
    mmc56x3_control2_register_t ctrl2;

    ctrl0.bits.auto_sr_enabled        = true;
    ctrl2.bits.periodical_set_enabled = true;
    ctrl2.bits.periodical_set_samples = samples;

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register, enable periodical set failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register, enable periodical set failed" );

    return ESP_OK;
}

esp_err_t mmc56x3_disable_periodical_set(mmc56x3_handle_t handle) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy registers */
    mmc56x3_control0_register_t ctrl0;
    mmc56x3_control2_register_t ctrl2;

    if(device->config.auto_sr_enabled == false) {
        ctrl0.bits.auto_sr_enabled = false;
    } else {
        ctrl0.bits.auto_sr_enabled = true;
    }
    ctrl2.bits.periodical_set_enabled = false;
    ctrl2.bits.periodical_set_samples = MMC56X3_MEAS_SAMPLE_1;

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register, enable periodical set failed" );

    /* attempt control 2 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control2_register(device, ctrl2), TAG, "write control 2 register, enable periodical set failed" );

    return ESP_OK;
}

esp_err_t mmc56x3_magnetic_set_reset(mmc56x3_handle_t handle) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set register */
    mmc56x3_control0_register_t ctrl0;
    ctrl0.bits.do_set   = true; // turn on set bit
    ctrl0.bits.do_reset = true; // turn on reset bit

    /* attempt control 0 register write */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control0_register(device, ctrl0), TAG, "write control 0 register failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MMC56X3_SETRESET_DELAY_MS));

    return ESP_OK;
}

esp_err_t mmc56x3_set_selftest_thresholds(mmc56x3_handle_t handle, const mmc56x3_selftest_axes_data_t axes_data) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transactions for each axis */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ST_X_TH_W, axes_data.x_axis), TAG, "write self-test x-axis threshold register failed" );
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ST_Y_TH_W, axes_data.y_axis), TAG, "write self-test y-axis threshold register failed" );
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ST_Z_TH_W, axes_data.z_axis), TAG, "write self-test z-axis threshold register failed" );

    return ESP_OK;
}

esp_err_t mmc56x3_get_selftest_set_values(mmc56x3_handle_t handle, mmc56x3_selftest_axes_data_t *const axes_data) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transactions for each axis */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_read_byte_from(device, MMC56X3_REG_ST_X_SV_RW, &axes_data->x_axis), TAG, "read self-test x-axis set value register failed" );
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_read_byte_from(device, MMC56X3_REG_ST_Y_SV_RW, &axes_data->y_axis), TAG, "read self-test y-axis set value register failed" );
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_read_byte_from(device, MMC56X3_REG_ST_Z_SV_RW, &axes_data->z_axis), TAG, "read self-test z-axis set value register failed" );
    
    return ESP_OK;
}

esp_err_t mmc56x3_set_selftest_set_values(mmc56x3_handle_t handle, const mmc56x3_selftest_axes_data_t axes_data) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transactions for each axis */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ST_X_SV_RW, axes_data.x_axis), TAG, "write self-test x-axis set value register failed" );
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ST_Y_SV_RW, axes_data.y_axis), TAG, "write self-test y-axis set value register failed" );
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_write_byte_to(device, MMC56X3_REG_ST_Z_SV_RW, axes_data.z_axis), TAG, "write self-test z-axis set value register failed" );

    return ESP_OK;
}

esp_err_t mmc56x3_reset(mmc56x3_handle_t handle) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt reset */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_reset(device), TAG, "reset device failed" );

    /* attempt setup */
    ESP_RETURN_ON_ERROR( mmc56x3_i2c_setup_registers(device), TAG, "setup device failed" );

    /* set register soft-reset to true */
    //mmc56x3_control1_register_t ctrl1 = { .bits.sw_reset = true };

    /* attempt control 1 register write */
    //ESP_RETURN_ON_ERROR( mmc56x3_i2c_set_control1_register(device, ctrl1), TAG, "write control 1 register failed" );

    /* delay task before next i2c transaction */
    //vTaskDelay(pdMS_TO_TICKS(MMC56X3_RESET_DELAY_MS));

    /* attempt magnet set reset */
    //ESP_RETURN_ON_ERROR( mmc56x3_magnetic_set_reset(handle), TAG, "magnetic set-reset failed" );

    /* attempt to set mode */
    //ESP_RETURN_ON_ERROR( mmc56x3_set_measure_mode(handle, false), TAG, "disable continuous mode set failed" );

    /* configure the device */
    //ESP_RETURN_ON_ERROR( mmc56x3_set_measure_bandwidth(handle, device->config.measurement_bandwidth), TAG, "unable to set measure bandwidth, init failed" );
    //ESP_RETURN_ON_ERROR( mmc56x3_set_data_rate(handle, device->config.data_rate), TAG, "unable to issue soft-reset, init failed" );
    //ESP_RETURN_ON_ERROR( mmc56x3_set_measure_mode(handle, device->config.continuous_mode_enabled), TAG, "unable to set measure mode, init failed" );

    return ESP_OK;
}

esp_err_t mmc56x3_remove(mmc56x3_handle_t handle) {
    mmc56x3_device_t* device = (mmc56x3_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(device->i2c_handle);
}

esp_err_t mmc56x3_delete(mmc56x3_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( mmc56x3_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

float mmc56x3_convert_to_heading(const mmc56x3_magnetic_axes_data_t axes_data) {
    float heading = 0.0f;

    /* honeywell application note AN-203 */
    if(axes_data.y_axis > 0.0f) heading = 90.0f - atanf(axes_data.x_axis/axes_data.y_axis) * 180.0f / M_PI;
    if(axes_data.y_axis < 0.0f) heading = 270.0f - atanf(axes_data.x_axis/axes_data.y_axis) * 180.0f / M_PI;
    if(axes_data.y_axis == 0.0f && axes_data.x_axis < 0.0f) heading = 180.0f;
    if(axes_data.y_axis == 0.0f && axes_data.x_axis > 0.0f) heading = 0.0f;

    /* convert magnetic axes to heading in degrees */
    //heading = (atan2f(axes_data.y_axis, axes_data.x_axis) * 180.0f / (float)M_PI);
    /* convert to heading to a 0..360 degree range */
    //if (heading < 0.0f) { heading += 360.0f; }

    return heading;
}

float mmc56x3_convert_to_true_heading(const float declination, const mmc56x3_magnetic_axes_data_t axes_data) {
    float heading = 0.0f;

    /* honeywell application note AN-203 */
    if(axes_data.y_axis > 0.0f) heading = 90.0f - atanf(axes_data.x_axis/axes_data.y_axis) * 180.0f / M_PI;
    if(axes_data.y_axis < 0.0f) heading = 270.0f - atanf(axes_data.x_axis/axes_data.y_axis) * 180.0f / M_PI;
    if(axes_data.y_axis == 0.0f && axes_data.x_axis < 0.0f) heading = 180.0f;
    if(axes_data.y_axis == 0.0f && axes_data.x_axis > 0.0f) heading = 0.0f;

    return heading + declination;
}

const char* mmc56x3_get_fw_version(void) {
    return (const char*)MMC56X3_FW_VERSION_STR;
}

int32_t mmc56x3_get_fw_version_number(void) {
    return (int32_t)MMC56X3_FW_VERSION_INT32;
}

