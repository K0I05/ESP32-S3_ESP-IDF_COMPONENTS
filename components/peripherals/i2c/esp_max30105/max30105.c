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
 * @file max30105.c
 *
 * ESP-IDF driver for MAX30105 sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/max30105.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/*
 * MAX30105 definitions
 */

#define MAX30105_REG_INT_STS1_R              UINT8_C(0x00) /*!< max30105 */ 
#define MAX30105_REG_INT_STS2_R              UINT8_C(0x01) /*!< max30105 */ 
#define MAX30105_REG_INT_ENB1_RW             UINT8_C(0x02) /*!< max30105 */ 
#define MAX30105_REG_INT_ENB2_RW             UINT8_C(0x03) /*!< max30105 */ 

#define MAX30105_REG_FIFO_WR_PTR_RW          UINT8_C(0x04) /*!< max30105 */ 
#define MAX30105_REG_FIFO_OVF_CNT_RW         UINT8_C(0x05) /*!< max30105 */ 
#define MAX30105_REG_FIFO_RD_PTR_RW          UINT8_C(0x06) /*!< max30105 */ 
#define MAX30105_REG_FIFO_DATA_RW            UINT8_C(0x07) /*!< max30105 */ 

#define MAX30105_REG_FIFO_CONFIG_RW          UINT8_C(0x08) /*!< max30105 */
#define MAX30105_REG_MODE_CONFIG_RW          UINT8_C(0x09) /*!< max30105 */
#define MAX30105_REG_SPO2_CONFIG_RW          UINT8_C(0x0A) /*!< max30105 */
#define MAX30105_REG_LED1_PA_RW              UINT8_C(0x0C) /*!< max30105 */
#define MAX30105_REG_LED2_PA_RW              UINT8_C(0x0D) /*!< max30105 */
#define MAX30105_REG_LED3_PA_RW              UINT8_C(0x0E) /*!< max30105 */
#define MAX30105_REG_PILOT_PA_RW             UINT8_C(0x10) /*!< max30105 */
#define MAX30105_REG_MLED1_MC_RW             UINT8_C(0x11) /*!< max30105 */
#define MAX30105_REG_MLED2_MC_RW             UINT8_C(0x12) /*!< max30105 */

#define MAX30105_REG_DIETEMP_INT_R           UINT8_C(0x1F) /*!< max30105 */
#define MAX30105_REG_DIETEMP_FRAC_R          UINT8_C(0x20) /*!< max30105 */
#define MAX30105_REG_DIETEMP_CONFIG_R        UINT8_C(0x21) /*!< max30105 */

#define MAX30105_REG_PROX_INT_THLD_RW        UINT8_C(0x30) /*!< max30105 */
#define MAX30105_REG_REV_ID_R                UINT8_C(0xFE) /*!< max30105 */
#define MAX30105_REG_PART_ID_R               UINT8_C(0xFE) /*!< max30105 */



#define MAX30105_DATA_POLL_TIMEOUT_MS       UINT16_C(100)
#define MAX30105_DATA_READY_DELAY_MS        UINT16_C(2)
#define MAX30105_POWERUP_DELAY_MS           UINT16_C(120)
#define MAX30105_RESET_DELAY_MS             UINT16_C(25)
#define MAX30105_SETUP_DELAY_MS             UINT16_C(15)
#define MAX30105_APPSTART_DELAY_MS          UINT16_C(10)    /*!< max30105 delay after initialization before application start-up */
#define MAX30105_CMD_DELAY_MS               UINT16_C(5)     /*!< max30105 delay before attempting I2C transactions after a command is issued */
#define MAX30105_TX_RX_DELAY_MS             UINT16_C(10)    /*!< max30105 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "max30105";




/**
 * @brief MAX30105 I2C read byte from register address transaction.
 * 
 * @param handle MAX30105 device handle.
 * @param reg_addr MAX30105 register address to read from.
 * @param byte MAX30105 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_read_byte_from(max30105_handle_t handle, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read from failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(handle->i2c_handle, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C write byte to register address transaction.
 * 
 * @param handle MAX30105 device handle.
 * @param reg_addr MAX30105 register address to write to.
 * @param byte MAX30105 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_write_byte_to(max30105_handle_t handle, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write to failed" );
                        
    return ESP_OK;
}

esp_err_t max30105_get_interrupt_status1_register(max30105_handle_t handle, max30105_interrupt_status1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(handle, MAX30105_REG_INT_STS1_R, &reg->reg), TAG, "read interrupt status 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_get_interrupt_status2_register(max30105_handle_t handle, max30105_interrupt_status2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(handle, MAX30105_REG_INT_STS2_R, &reg->reg), TAG, "read interrupt status 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_get_interrupt_enable1_register(max30105_handle_t handle, max30105_interrupt_enable1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(handle, MAX30105_REG_INT_ENB1_RW, &reg->reg), TAG, "read interrupt enable 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_set_interrupt_enable1_register(max30105_handle_t handle, const max30105_interrupt_enable1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    max30105_interrupt_enable1_register_t irq_enable1 = { .reg = reg.reg };

    /* set register reserved settings */
    irq_enable1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(handle, MAX30105_REG_INT_ENB1_RW, irq_enable1.reg), TAG, "write interrupt enable 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_get_interrupt_enable2_register(max30105_handle_t handle, max30105_interrupt_enable2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(handle, MAX30105_REG_INT_ENB2_RW, &reg->reg), TAG, "read interrupt enable 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_set_interrupt_enable2_register(max30105_handle_t handle, const max30105_interrupt_enable2_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    max30105_interrupt_enable2_register_t irq_enable2 = { .reg = reg.reg };

    /* set register reserved settings */
    irq_enable2.bits.reserved1 = 0;
    irq_enable2.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(handle, MAX30105_REG_INT_ENB2_RW, irq_enable2.reg), TAG, "write interrupt enable 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_get_mode_configuration_register(max30105_handle_t handle, max30105_mode_configuration_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(handle, MAX30105_REG_MODE_CONFIG_RW, &reg->reg), TAG, "read mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t max30105_set_mode_configuration_register(max30105_handle_t handle, const max30105_mode_configuration_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    max30105_mode_configuration_register_t mode_config = { .reg = reg.reg };

    /* set register reserved settings */
    mode_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(handle, MAX30105_REG_MODE_CONFIG_RW, mode_config.reg), TAG, "write mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

