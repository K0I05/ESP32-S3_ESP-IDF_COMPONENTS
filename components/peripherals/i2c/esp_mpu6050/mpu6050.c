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
 * @file aht2x.c
 *
 * ESP-IDF driver for MPU6050 sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * MPU6050 definitions
*/

#define MPU6050_REG_SELF_TEST_X_RW          UINT8_C(0x0d)
#define MPU6050_REG_SELF_TEST_Y_RW          UINT8_C(0x0e)
#define MPU6050_REG_SELF_TEST_Z_RW          UINT8_C(0x0f)
#define MPU6050_REG_SELF_TEST_A_RW          UINT8_C(0x10)
#define MPU6050_REG_SMPLRT_DIV_RW           UINT8_C(0x19)
#define MPU6050_REG_CONFIG_RW               UINT8_C(0x1a)
#define MPU6050_REG_GYRO_CONFIG_RW          UINT8_C(0x1b)
#define MPU6050_REG_ACCEL_CONFIG_RW         UINT8_C(0x1c)
#define MPU6050_REG_FIFO_EN_RW              UINT8_C(0x23)

#define MPU6050_REG_INT_PIN_CFG_RW          UINT8_C(0x37)
#define MPU6050_REG_INT_ENABLE_RW           UINT8_C(0x38)
#define MPU6050_REG_INT_STATUS_R            UINT8_C(0x3a)

#define MPU6050_REG_ACCEL_XOUT_H_R          UINT8_C(0x3b)
#define MPU6050_REG_ACCEL_XOUT_L_R          UINT8_C(0x3c)
#define MPU6050_REG_ACCEL_YOUT_H_R          UINT8_C(0x3d)
#define MPU6050_REG_ACCEL_YOUT_L_R          UINT8_C(0x3e)
#define MPU6050_REG_ACCEL_ZOUT_H_R          UINT8_C(0x3f)
#define MPU6050_REG_ACCEL_ZOUT_L_R          UINT8_C(0x40)
#define MPU6050_REG_TEMP_OUT_H_R            UINT8_C(0x41)
#define MPU6050_REG_TEMP_OUT_L_R            UINT8_C(0x42)
#define MPU6050_REG_GYRO_XOUT_H_R           UINT8_C(0x43)
#define MPU6050_REG_GYRO_XOUT_L_R           UINT8_C(0x44)
#define MPU6050_REG_GYRO_YOUT_H_R           UINT8_C(0x45)
#define MPU6050_REG_GYRO_YOUT_L_R           UINT8_C(0x46)
#define MPU6050_REG_GYRO_ZOUT_H_R           UINT8_C(0x47)
#define MPU6050_REG_GYRO_ZOUT_L_R           UINT8_C(0x48)

#define MPU6050_REG_SIGNAL_PATH_RESET_RW    UINT8_C(0x68)
#define MPU6050_REG_USER_CTRL_RW            UINT8_C(0x6a)
#define MPU6050_REG_PWR_MGMT_1_RW           UINT8_C(0x6b)
#define MPU6050_REG_PWR_MGMT_2_RW           UINT8_C(0x6c)
#define MPU6050_REG_FIFO_COUNT_H_RW         UINT8_C(0x72)
#define MPU6050_REG_FIFO_COUNT_L_RW         UINT8_C(0x73)
#define MPU6050_REG_FIFO_R_W_RW             UINT8_C(0x74)
#define MPU6050_REG_WHO_AM_I_R              UINT8_C(0x75)


#define MPU6050_DATA_READY_DELAY_MS         UINT16_C(1)
#define MPU6050_DATA_POLL_TIMEOUT_MS        UINT16_C(500)
#define MPU6050_POWERUP_DELAY_MS            UINT16_C(100)
#define MPU6050_APPSTART_DELAY_MS           UINT16_C(20)
#define MPU6050_RESET_DELAY_MS              UINT16_C(50)
#define MPU6050_CMD_DELAY_MS                UINT16_C(5)
#define MPU6050_TX_RX_DELAY_MS              UINT16_C(10)
/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "mpu6050";


/**
 * @brief MPU6050 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle MPU6050 device handle.
 * @param reg_addr MPU6050 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mpu6050_i2c_read_from(mpu6050_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "mpu6050_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief MPU6050 I2C read byte from register address transaction.
 * 
 * @param handle MPU6050 device handle.
 * @param reg_addr MPU6050 register address to read from.
 * @param byte MPU6050 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mpu6050_i2c_read_byte_from(mpu6050_handle_t handle, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "mpu6050_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief MPU6050 I2C write byte to register address transaction.
 * 
 * @param handle MPU6050 device handle.
 * @param reg_addr MPU6050 register address to write to.
 * @param byte MPU6050 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mpu6050_i2c_write_byte_to(mpu6050_handle_t handle, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "mpu6050_i2c_write_byte_to, i2c write failed" );
                        
    return ESP_OK;
}

static inline esp_err_t mpu6050_get_accel_sensitivity(mpu6050_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* determine resolution */
    switch(handle->dev_config.accel_full_scale_range) {
        case MPU6050_ACCEL_FS_RANGE_2G:
            handle->accel_sensitivity = 16384.0;  // 16384 LSB/g
            break;
        case MPU6050_ACCEL_FS_RANGE_4G:
            handle->accel_sensitivity = 8192.0;  // 8192 LSB/g
            break;
        case MPU6050_ACCEL_FS_RANGE_8G:
            handle->accel_sensitivity = 4096.0;  // 4096 LSB/g
            break;
        case MPU6050_ACCEL_FS_RANGE_16G:
            handle->accel_sensitivity = 2048.0;  // 2048 LSB/g
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static inline esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* determine sensitivity */
    switch(handle->dev_config.gyro_full_scale_range) {
        case MPU6050_GYRO_FS_RANGE_250DPS:
            handle->gyro_sensitivity = 131.0;    // 131 LSB/°/s
            break;
        case MPU6050_GYRO_FS_RANGE_500DPS:
            handle->gyro_sensitivity = 65.5;    // 65.5 LSB/°/s
            break;
        case MPU6050_GYRO_FS_RANGE_1000DPS:
            handle->gyro_sensitivity = 32.8;    // 32.8 LSB/°/s
            break;
        case MPU6050_GYRO_FS_RANGE_2000DPS:
            handle->gyro_sensitivity = 16.4;    // 16.4 LSB/°/s
            break;
        default:
            return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}


esp_err_t mpu6050_get_sample_rate_divider_register(mpu6050_handle_t handle, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_SMPLRT_DIV_RW, reg), TAG, "read sample rate divider register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * sample rate = gyroscope output rate / (1 + sample rate divider)
 * 8khz or 1khz
 */
esp_err_t mpu6050_set_sample_rate_divider_register(mpu6050_handle_t handle, const uint8_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_SMPLRT_DIV_RW, reg), TAG, "write sample rate divider register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_config_register(mpu6050_handle_t handle, mpu6050_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_CONFIG_RW, &reg->reg), TAG, "read configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_config_register(mpu6050_handle_t handle, const mpu6050_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    mpu6050_config_register_t config = { .reg = reg.reg };

    config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_CONFIG_RW, config.reg), TAG, "write configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_gyro_config_register(mpu6050_handle_t handle, mpu6050_gyro_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_GYRO_CONFIG_RW, &reg->reg), TAG, "read gyroscope configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_gyro_config_register(mpu6050_handle_t handle, const mpu6050_gyro_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    mpu6050_gyro_config_register_t gyro_config = { .reg = reg.reg };

    gyro_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_GYRO_CONFIG_RW, gyro_config.reg), TAG, "write gyroscope configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    /* attempt to update gyroscope sensitivity */
    ESP_RETURN_ON_ERROR( mpu6050_get_gyro_sensitivity(handle), TAG, "gyroscope sensitivity update for write gyroscope configuration register failed" );

    return ESP_OK;
}

esp_err_t mpu6050_get_accel_config_register(mpu6050_handle_t handle, mpu6050_accel_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_ACCEL_CONFIG_RW, &reg->reg), TAG, "read accelerometer configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_accel_config_register(mpu6050_handle_t handle, const mpu6050_accel_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    mpu6050_accel_config_register_t accel_config = { .reg = reg.reg };

    accel_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_ACCEL_CONFIG_RW, accel_config.reg), TAG, "write accelerometer configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    /* attempt to update accelerometer sensitivity */
    ESP_RETURN_ON_ERROR( mpu6050_get_accel_sensitivity(handle), TAG, "accelerometer sensitivity update for write accelerometer configuration register failed" );

    return ESP_OK;
}

esp_err_t mpu6050_get_interrupt_enable_register(mpu6050_handle_t handle, mpu6050_interrupt_enable_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_INT_ENABLE_RW, &reg->reg), TAG, "read interrupt enable register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_interrupt_enable_register(mpu6050_handle_t handle, const mpu6050_interrupt_enable_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    mpu6050_interrupt_enable_register_t irq_enable = { .reg = reg.reg };
    irq_enable.bits.reserved1 = 0;
    irq_enable.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_INT_ENABLE_RW, irq_enable.reg), TAG, "write interrupt enable register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_interrupt_pin_config_register(mpu6050_handle_t handle, mpu6050_interrupt_pin_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_INT_PIN_CFG_RW, &reg->reg), TAG, "read interrupt pin configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_interrupt_pin_config_register(mpu6050_handle_t handle, const mpu6050_interrupt_pin_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    mpu6050_interrupt_pin_config_register_t irq_pin_config = { .reg = reg.reg };

    irq_pin_config.bits.reserved1 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_INT_PIN_CFG_RW, irq_pin_config.reg), TAG, "write interrupt pin configuration register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_interrupt_status_register(mpu6050_handle_t handle, mpu6050_interrupt_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_INT_STATUS_R, &reg->reg), TAG, "read interrupt status register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_power_management1_register(mpu6050_handle_t handle, mpu6050_power_management1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_PWR_MGMT_1_RW, &reg->reg), TAG, "read power management 1 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_power_management1_register(mpu6050_handle_t handle, const mpu6050_power_management1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    mpu6050_power_management1_register_t power_management1 = { .reg = reg.reg };

    power_management1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_PWR_MGMT_1_RW, power_management1.reg), TAG, "write power management 1 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_power_management2_register(mpu6050_handle_t handle, mpu6050_power_management2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_PWR_MGMT_2_RW, &reg->reg), TAG, "read power management 2 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_set_power_management2_register(mpu6050_handle_t handle, const mpu6050_power_management2_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_write_byte_to(handle, MPU6050_REG_PWR_MGMT_2_RW, reg.reg), TAG, "write power management 2 register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_get_who_am_i_register(mpu6050_handle_t handle, mpu6050_who_am_i_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_byte_from(handle, MPU6050_REG_WHO_AM_I_R, &reg->reg), TAG, "read who am i register failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t mpu6050_configure_interrupts(mpu6050_handle_t handle, const mpu6050_config_t *const config) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && config );

    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(config->irq_io_num), ESP_ERR_INVALID_ARG, TAG, "interrupt io number is invalid, configure interrupts failed");

    mpu6050_interrupt_pin_config_register_t irq_ping_config;

    ESP_RETURN_ON_ERROR( mpu6050_get_interrupt_pin_config_register(handle, &irq_ping_config), TAG, "unable to read interrupt pin configuration register, configure interrupts failed");

    if (MPU6050_IRQ_PIN_ACTIVE_LOW == config->irq_io_active_level) {
        irq_ping_config.bits.irq_active_level = MPU6050_IRQ_PIN_ACTIVE_LOW;
    }

    if (MPU6050_IRQ_PIN_OPEN_DRAIN == config->irq_io_mode) {
        irq_ping_config.bits.irq_level_mode = MPU6050_IRQ_PIN_OPEN_DRAIN;
    }

    if (MPU6050_IRQ_LATCH_UNTIL_CLEARED == config->irq_latch) {
        irq_ping_config.bits.irq_latch = MPU6050_IRQ_LATCH_UNTIL_CLEARED;
    }

    if (MPU6050_IRQ_CLEAR_ON_STATUS_READ == config->irq_clear_behavior) {
        irq_ping_config.bits.irq_read_clear = MPU6050_IRQ_CLEAR_ON_STATUS_READ;
    }

    ESP_RETURN_ON_ERROR( mpu6050_set_interrupt_pin_config_register(handle, irq_ping_config), TAG, "unable to write interrupt pin configuration register, configure interrupts failed");

    gpio_int_type_t gpio_intr_type;

    if (MPU6050_IRQ_PIN_ACTIVE_LOW == config->irq_io_active_level) {
        gpio_intr_type = GPIO_INTR_NEGEDGE;
    } else {
        gpio_intr_type = GPIO_INTR_POSEDGE;
    }

    gpio_config_t irq_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = gpio_intr_type,
        .pin_bit_mask = (BIT0 << config->irq_io_num)
    };

    ESP_RETURN_ON_ERROR( gpio_config(&irq_gpio_config), TAG, "unable to configure interrupt GPIO pin, configure interrupts failed");

    return ESP_OK;
}

static inline esp_err_t mpu6050_setup(mpu6050_handle_t handle) {
    uint8_t                                 sample_rate_divider_reg;
    mpu6050_config_register_t               config_reg;
    mpu6050_gyro_config_register_t          gyro_config_reg;
    mpu6050_accel_config_register_t         accel_config_reg;
    mpu6050_interrupt_enable_register_t     irq_enable_reg;
    mpu6050_power_management1_register_t    power_management1_reg;
    
    /* read configuration registers */

    /* attempt to read device sample rate divider register */
    ESP_RETURN_ON_ERROR(mpu6050_get_sample_rate_divider_register(handle, &sample_rate_divider_reg), TAG, "read sample rate divider register for init failed");

    /* attempt to read device configuration register */
    ESP_RETURN_ON_ERROR(mpu6050_get_config_register(handle, &config_reg), TAG, "write configuration register for init failed");

    /* attempt to read device gyroscope configuration register */
    ESP_RETURN_ON_ERROR(mpu6050_get_gyro_config_register(handle, &gyro_config_reg), TAG, "write gyroscope configuration register for init failed");

    /* attempt to read device accelerometer configuration register */
    ESP_RETURN_ON_ERROR(mpu6050_get_accel_config_register(handle, &accel_config_reg), TAG, "write accelerometer configuration register for init failed");

    /* attempt to read device interrupt enable register */
    ESP_RETURN_ON_ERROR(mpu6050_get_interrupt_enable_register(handle, &irq_enable_reg), TAG, "write interrupt enable register for init failed");

    /* attempt to read device power management 1 register */
    ESP_RETURN_ON_ERROR(mpu6050_get_power_management1_register(handle, &power_management1_reg), TAG, "write power management 1 register for init failed");


    /* write configuration registers */

    /* set sample divider register */
    sample_rate_divider_reg = 7; // low-pass filter is disabled - gyroscope output rate is 8KHz, set 1ms sampling rate

    /* attempt to write device sample rate divider register */
    ESP_RETURN_ON_ERROR(mpu6050_set_sample_rate_divider_register(handle, sample_rate_divider_reg), TAG, "write sample rate divider register for init failed");

    /* set configuration register */
    config_reg.bits.low_pass_filter  = handle->dev_config.low_pass_filter;
    config_reg.bits.ext_sync_setting = MPU6050_EXT_SYNC_SETTING_INPUT_DISABLED;

    /* attempt to write device configuration register */
    ESP_RETURN_ON_ERROR(mpu6050_set_config_register(handle, config_reg), TAG, "write configuration register for init failed");

    /* set gyroscope configuration register */
    gyro_config_reg.bits.full_scale_range = handle->dev_config.gyro_full_scale_range;
    gyro_config_reg.bits.x_axis_selftest_enabled = false;
    gyro_config_reg.bits.y_axis_selftest_enabled = false;
    gyro_config_reg.bits.z_axis_selftest_enabled = false;

    /* attempt to write device gyroscope configuration register */
    ESP_RETURN_ON_ERROR(mpu6050_set_gyro_config_register(handle, gyro_config_reg), TAG, "write gyroscope configuration register for init failed");

    /* set accelerometer configuration register */
    accel_config_reg.bits.full_scale_range = handle->dev_config.accel_full_scale_range;
    accel_config_reg.bits.x_axis_selftest_enabled = false;
    accel_config_reg.bits.y_axis_selftest_enabled = false;
    accel_config_reg.bits.z_axis_selftest_enabled = false;

    /* attempt to write device accelerometer configuration register */
    ESP_RETURN_ON_ERROR(mpu6050_set_accel_config_register(handle, accel_config_reg), TAG, "write accelerometer configuration register for init failed");

    /* set interrupt enable register  */
    irq_enable_reg.bits.data_ready_enabled    = true;
    irq_enable_reg.bits.i2c_master_enabled    = false;
    irq_enable_reg.bits.fifo_overflow_enabled = false;

    /* attempt to write device interrupt enable register */
    ESP_RETURN_ON_ERROR(mpu6050_set_interrupt_enable_register(handle, irq_enable_reg), TAG, "write interrupt enable register for init failed");

    /* set power management 1 register */
    power_management1_reg.bits.clock_source  = handle->dev_config.gyro_clock_source;
    power_management1_reg.bits.cycle_enabled = false;
    power_management1_reg.bits.reset_enabled = false;
    power_management1_reg.bits.temp_disabled = false;
    power_management1_reg.bits.sleep_enabled = false;

    /* attempt to write device power management 1 register */
    ESP_RETURN_ON_ERROR(mpu6050_set_power_management1_register(handle, power_management1_reg), TAG, "write power management 1 register for init failed");

    return ESP_OK;
}

esp_err_t mpu6050_init(i2c_master_bus_handle_t master_handle, const mpu6050_config_t *mpu6050_config, mpu6050_handle_t *mpu6050_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && mpu6050_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, mpu6050_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, mpu6050 device handle initialization failed", mpu6050_config->i2c_address);

    /* validate memory availability for handle */
    mpu6050_handle_t out_handle;
    out_handle = (mpu6050_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mpu6050 device for init failed");

    /* copy configuration */
    out_handle->dev_config = *mpu6050_config;

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

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_CMD_DELAY_MS));

    /* attempt soft-reset and initialize registers */
    ESP_GOTO_ON_ERROR(mpu6050_reset(out_handle), err_handle, TAG, "soft-reset failed");

    /* set device handle */
    *mpu6050_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t mpu6050_get_status(mpu6050_handle_t handle, bool *fifo_overflow, bool *i2c_master, bool *data_ready) {
    mpu6050_interrupt_status_register_t irq_sts;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_get_interrupt_status_register(handle, &irq_sts), TAG, "read interrupt status register for get status failed" );

    /* set interrupt status variables */
    *fifo_overflow  = irq_sts.bits.irq_fifo_overflow;
    *i2c_master     = irq_sts.bits.irq_i2c_master;
    *data_ready     = irq_sts.bits.irq_data_ready;

    return ESP_OK;
}

esp_err_t mpu6050_get_data_status(mpu6050_handle_t handle, bool *ready) {
    mpu6050_interrupt_status_register_t irq_sts;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_get_interrupt_status_register(handle, &irq_sts), TAG, "read interrupt status register for get data status failed" );

    /* set interrupt status variables */
    *ready = irq_sts.bits.irq_data_ready;

    return ESP_OK;
}

static inline esp_err_t mpu6050_get_raw_motion(mpu6050_handle_t handle, mpu6050_data_axes_t *gyro_data, mpu6050_data_axes_t *accel_data, int16_t *temperature) {
    esp_err_t   ret             = ESP_OK;
    uint64_t    start_time      = 0;
    bool        data_is_ready   = false;
    uint8_t     rx[14]          = { 0 };
    
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* set start time (us) for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( mpu6050_get_data_status(handle, &data_is_ready), err, TAG, "data ready read for get measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(MPU6050_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (MPU6050_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt i2c accelerometer, temperature, and gyroscope data read transaction */
    ESP_RETURN_ON_ERROR( mpu6050_i2c_read_from(handle, MPU6050_REG_ACCEL_XOUT_H_R, rx, sizeof(rx)), TAG, "read accelerometer, temperature, and gyroscope data registers failed" );
    
    /* set accelerometer raw data parameter */
    accel_data->x_axis = (int16_t)((rx[0] << 8) | (rx[1]));
    accel_data->y_axis = (int16_t)((rx[2] << 8) | (rx[3]));
    accel_data->z_axis = (int16_t)((rx[4] << 8) | (rx[5]));

    /* set temperature raw data parameter */
    *temperature = (int16_t)((rx[6] << 8) | (rx[7]));

    /* set gyroscope raw data parameter */
    gyro_data->x_axis = (int16_t)((rx[8] << 8) | (rx[9]));
    gyro_data->y_axis = (int16_t)((rx[10] << 8) | (rx[11]));
    gyro_data->z_axis = (int16_t)((rx[12] << 8) | (rx[13]));
    
    return ESP_OK;

    err:
        return ret;
}

esp_err_t mpu6050_get_motion(mpu6050_handle_t handle, mpu6050_gyro_data_axes_t *gyro_data, mpu6050_accel_data_axes_t *accel_data, float *temperature) {
    mpu6050_data_axes_t raw_accel_data_axes;
    mpu6050_data_axes_t raw_gyro_data_axes;
    int16_t                 raw_temperature;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read raw measurements */
    ESP_RETURN_ON_ERROR(mpu6050_get_raw_motion(handle, &raw_gyro_data_axes, &raw_accel_data_axes, &raw_temperature), TAG, "read raw movement for read movement failed");

    /* set corrected accelerometer data parameter */
    accel_data->x_axis = raw_accel_data_axes.x_axis / handle->accel_sensitivity;
    accel_data->y_axis = raw_accel_data_axes.y_axis / handle->accel_sensitivity;
    accel_data->z_axis = raw_accel_data_axes.z_axis / handle->accel_sensitivity;

    /* set corrected temperature data parameter */
    *temperature = raw_temperature / 340.00 + 36.53;

    /* set corrected gyroscope data parameter */
    gyro_data->x_axis = raw_gyro_data_axes.x_axis / handle->gyro_sensitivity;
    gyro_data->y_axis = raw_gyro_data_axes.y_axis / handle->gyro_sensitivity;
    gyro_data->z_axis = raw_gyro_data_axes.z_axis / handle->gyro_sensitivity;

    return ESP_OK;
}

esp_err_t mpu6050_register_isr(mpu6050_handle_t handle, const mpu6050_isr_t isr) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to register isr */
    ESP_RETURN_ON_ERROR( gpio_isr_handler_add(handle->dev_config.irq_io_num, ((gpio_isr_t) * (isr)), ((void *) handle) ), TAG, "register isr failed" );

    /* attempt to enable interrupt signal */
    ESP_RETURN_ON_ERROR( gpio_intr_enable(handle->dev_config.irq_io_num), TAG, "enable interrupt signal failed" );

    return ESP_OK;
}

esp_err_t mpu6050_reset(mpu6050_handle_t handle) {
    mpu6050_power_management1_register_t pwr_mgmnt;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read power management 1 register */
    ESP_RETURN_ON_ERROR( mpu6050_get_power_management1_register(handle, &pwr_mgmnt), TAG, "unable to get power management 1 register, reset failed" );

    /* set power management 1 register to reset device */
    pwr_mgmnt.bits.reset_enabled = true;

    /* attempt to write power management 1 register */
    ESP_RETURN_ON_ERROR( mpu6050_set_power_management1_register(handle, pwr_mgmnt), TAG, "unable to set power management 1 register, reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MPU6050_RESET_DELAY_MS));

    /* reconfigure sensor */
    ESP_RETURN_ON_ERROR( mpu6050_setup(handle), TAG, "unable to setup device, reset failed" );

    return ESP_OK;
}

esp_err_t mpu6050_remove(mpu6050_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t mpu6050_delete(mpu6050_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( mpu6050_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* mpu6050_get_fw_version(void) {
    return MPU6050_FW_VERSION_STR;
}

int32_t mpu6050_get_fw_version_number(void) {
    return MPU6050_FW_VERSION_INT32;
}

