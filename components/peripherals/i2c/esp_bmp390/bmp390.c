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
 * @file bmp280.c
 *
 * ESP-IDF driver for BMP390 temperature and pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/bmp390.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * possible BMP390 registers
 */
#define BMP390_REG_TEMP_XLSB            UINT8_C(0x07)
#define BMP390_REG_TEMP_LSB             UINT8_C(0x08)
#define BMP390_REG_TEMP_MSB             UINT8_C(0x09)
#define BMP390_REG_TEMP                 (BMP390_REG_TEMP_XLSB)
#define BMP390_REG_PRESS_XLSB           UINT8_C(0x04) 
#define BMP390_REG_PRESS_LSB            UINT8_C(0x05)
#define BMP390_REG_PRESS_MSB            UINT8_C(0x06)
#define BMP390_REG_PRESSURE             (BMP390_REG_PRESS_XLSB)
#define BMP390_REG_SNRTIME_XLSB         UINT8_C(0x0C)
#define BMP390_REG_SNRTIME_LSB          UINT8_C(0x0D)
#define BMP390_REG_SNRTIME_MSB          UINT8_C(0x0E)
#define BMP390_REG_SNRTIME              (BMP390_REG_SNRTIME_XLSB)
#define BMP390_REG_EVENT                UINT8_C(0x10)
#define BMP390_REG_CONFIG               UINT8_C(0x1F)
#define BMP390_REG_PWRCTRL              UINT8_C(0x1B)
#define BMP390_REG_OSR                  UINT8_C(0x1C)
#define BMP390_REG_ODR                  UINT8_C(0x1D) 
#define BMP390_REG_STATUS               UINT8_C(0x03)
#define BMP390_REG_INT_STATUS           UINT8_C(0x11) 
#define BMP390_REG_INT_CNTRL            UINT8_C(0x19)
#define BMP390_REG_CHIP_ID              UINT8_C(0x00)
#define BMP390_REG_ERR                  UINT8_C(0x02)
#define BMP390_REG_CMD                  UINT8_C(0x7E)
#define BMP390_SFTRESET_CMD             UINT8_C(0xB6)

#define BMP390_CHIP_ID_DFLT             UINT8_C(0x60)  //!< BMP390 default

#define BMP390_DATA_POLL_TIMEOUT_MS     UINT16_C(1000) // ? see datasheet tables 13 and 14, standby-time could be 2-seconds (2000ms)
#define BMP390_DATA_READY_DELAY_MS      UINT16_C(1)
#define BMP390_POWERUP_DELAY_MS         UINT16_C(25)  // start-up time is 2-ms
#define BMP390_APPSTART_DELAY_MS        UINT16_C(25)
#define BMP390_RESET_DELAY_MS           UINT16_C(25)
#define BMP390_CMD_DELAY_MS             UINT16_C(5)
#define BMP390_TX_RX_DELAY_MS           UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "bmp390";


/**
 * @brief BMP390 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle BMP390 device handle.
 * @param reg_addr BMP390 register address to read from.
 * @param buffer BMP390 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_read_from(bmp390_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "bmp390_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C read halfword from register address transaction.
 * 
 * @param handle BMP390 device handle.
 * @param reg_addr BMP390 register address to read from.
 * @param halfword BMP390 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_read_word_from(bmp390_handle_t handle, const uint8_t reg_addr, uint16_t *const halfword) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bmp390_i2c_read_word_from failed" );

    /* set output parameter */
    *halfword = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief BMP390 I2C read byte from register address transaction.
 * 
 * @param handle BMP390 device handle.
 * @param reg_addr BMP390 register address to read from.
 * @param byte BMP390 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_read_byte_from(bmp390_handle_t handle, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bmp390_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief BMP390 I2C write byte to register address transaction.
 * 
 * @param handle BMP390 device handle.
 * @param reg_addr BMP390 register address to write to.
 * @param byte BMP390 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_write_byte_to(bmp390_handle_t handle, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Temperature compensation algorithm is taken from BMP390 datasheet.  See datasheet for details.
 *
 * @param[in] handle BMP390 device handle.
 * @param[in] adc_temperature Raw adc temperature.
 * @return Compensated temperature in degrees Celsius.
 */
static inline double bmp390_compensate_temperature(bmp390_handle_t handle, const uint32_t adc_temperature) {
    double var1 = (double)(adc_temperature - handle->dev_conv_cal_factors->PAR_T1);
    double var2 = (double)(var1 * handle->dev_conv_cal_factors->PAR_T2);
    //
    handle->dev_conv_cal_factors->t_lin = var2 + (var1 * var1) * handle->dev_conv_cal_factors->PAR_T3;

    return handle->dev_conv_cal_factors->t_lin;
}

/**
 * @brief Pressure compensation algorithm is taken from BMP390 datasheet.  See datasheet for details.
 *
 * @param[in] handle BMP390 device handle.
 * @param[in] adc_pressure Raw adc pressure.
 * @return Compensated pressure in pascal.
 */
static inline double bmp390_compensate_pressure(bmp390_handle_t handle, const uint32_t adc_pressure) {
    double dat1 = handle->dev_conv_cal_factors->PAR_P6 * handle->dev_conv_cal_factors->t_lin;
    double dat2 = handle->dev_conv_cal_factors->PAR_P7 * handle->dev_conv_cal_factors->t_lin * handle->dev_conv_cal_factors->t_lin;
    double dat3 = handle->dev_conv_cal_factors->PAR_P8 * handle->dev_conv_cal_factors->t_lin * handle->dev_conv_cal_factors->t_lin * handle->dev_conv_cal_factors->t_lin;
    double var1 = handle->dev_conv_cal_factors->PAR_P5 + dat1 + dat2 + dat3;
    //
    dat1 = handle->dev_conv_cal_factors->PAR_P2 * handle->dev_conv_cal_factors->t_lin;
    dat2 = handle->dev_conv_cal_factors->PAR_P3 * handle->dev_conv_cal_factors->t_lin * handle->dev_conv_cal_factors->t_lin;
    dat3 = handle->dev_conv_cal_factors->PAR_P4 * handle->dev_conv_cal_factors->t_lin * handle->dev_conv_cal_factors->t_lin * handle->dev_conv_cal_factors->t_lin;
    double var2 = (double)adc_pressure * (handle->dev_conv_cal_factors->PAR_P1 + dat1 + dat2 + dat3);
    //
    dat1 = (double)adc_pressure * (double)adc_pressure;
    dat2 = handle->dev_conv_cal_factors->PAR_P9 + handle->dev_conv_cal_factors->PAR_P10 * handle->dev_conv_cal_factors->t_lin;
    dat3 = dat1 * dat2;
    double dat4 = dat3 + (double)adc_pressure * (double)adc_pressure * (double)adc_pressure * handle->dev_conv_cal_factors->PAR_P11;
    //
    return var1 + var2 + dat4;
}

/**
 * @brief Reads calibration factors onboard the BMP390 and applies floating point correction factors.  See datasheet for details.
 *
 * @param[in] handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_get_cal_factors(bmp390_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* bmp280 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x31, &handle->dev_cal_factors->dig_T1) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x33, &handle->dev_cal_factors->dig_T2) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x35, (uint8_t *)&handle->dev_cal_factors->dig_T3) );
    /* bmp280 attempt to request P1-P10 calibration values from device */
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x36, (uint16_t *)&handle->dev_cal_factors->dig_P1) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x38, (uint16_t *)&handle->dev_cal_factors->dig_P2) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x3a, (uint8_t *)&handle->dev_cal_factors->dig_P3) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x3b, (uint8_t *)&handle->dev_cal_factors->dig_P4) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x3c, &handle->dev_cal_factors->dig_P5) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x3e, &handle->dev_cal_factors->dig_P6) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x40, (uint8_t *)&handle->dev_cal_factors->dig_P7) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x41, (uint8_t *)&handle->dev_cal_factors->dig_P8) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(handle, 0x42, (uint16_t *)&handle->dev_cal_factors->dig_P9) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x44, (uint8_t *)&handle->dev_cal_factors->dig_P10) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(handle, 0x45, (uint8_t *)&handle->dev_cal_factors->dig_P11) );

    /*
    ESP_LOGW(TAG, "Calibration data received:");
    ESP_LOGW(TAG, "dig_T1=%u", bmp390_handle->dev_cal_factors->dig_T1);
    ESP_LOGW(TAG, "dig_T2=%u", bmp390_handle->dev_cal_factors->dig_T2);
    ESP_LOGW(TAG, "dig_T3=%d", bmp390_handle->dev_cal_factors->dig_T3);
    ESP_LOGW(TAG, "dig_P1=%d", bmp390_handle->dev_cal_factors->dig_P1);
    ESP_LOGW(TAG, "dig_P2=%d", bmp390_handle->dev_cal_factors->dig_P2);
    ESP_LOGW(TAG, "dig_P3=%d", bmp390_handle->dev_cal_factors->dig_P3);
    ESP_LOGW(TAG, "dig_P4=%d", bmp390_handle->dev_cal_factors->dig_P4);
    ESP_LOGW(TAG, "dig_P5=%u", bmp390_handle->dev_cal_factors->dig_P5);
    ESP_LOGW(TAG, "dig_P6=%u", bmp390_handle->dev_cal_factors->dig_P6);
    ESP_LOGW(TAG, "dig_P7=%d", bmp390_handle->dev_cal_factors->dig_P7);
    ESP_LOGW(TAG, "dig_P8=%d", bmp390_handle->dev_cal_factors->dig_P8);
    ESP_LOGW(TAG, "dig_P9=%d", bmp390_handle->dev_cal_factors->dig_P9);
    ESP_LOGW(TAG, "dig_P10=%d", bmp390_handle->dev_cal_factors->dig_P10);
    ESP_LOGW(TAG, "dig_P11=%d", bmp390_handle->dev_cal_factors->dig_P11);
    */

    /* convert calibration factors to floating point numbers */
    handle->dev_conv_cal_factors->PAR_T1 = (float)handle->dev_cal_factors->dig_T1 / powf(2.0f, -8.0f);
    handle->dev_conv_cal_factors->PAR_T2 = (float)handle->dev_cal_factors->dig_T2 / powf(2.0f, 30.0f);
    handle->dev_conv_cal_factors->PAR_T3 = (float)handle->dev_cal_factors->dig_T3 / powf(2.0f, 48.0f);
    handle->dev_conv_cal_factors->PAR_P1 = ((float)handle->dev_cal_factors->dig_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    handle->dev_conv_cal_factors->PAR_P2 = ((float)handle->dev_cal_factors->dig_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    handle->dev_conv_cal_factors->PAR_P3 = (float)handle->dev_cal_factors->dig_P3 / powf(2.0f, 32.0f);
    handle->dev_conv_cal_factors->PAR_P4 = (float)handle->dev_cal_factors->dig_P4 / powf(2.0f, 37.0f);
    handle->dev_conv_cal_factors->PAR_P5 = (float)handle->dev_cal_factors->dig_P5 / powf(2.0f, -3.0f);
    handle->dev_conv_cal_factors->PAR_P6 = (float)handle->dev_cal_factors->dig_P6 / powf(2.0f, 6.0f);
    handle->dev_conv_cal_factors->PAR_P7 = (float)handle->dev_cal_factors->dig_P7 / powf(2.0f, 8.0f);
    handle->dev_conv_cal_factors->PAR_P8 = (float)handle->dev_cal_factors->dig_P8 / powf(2.0f, 15.0f);
    handle->dev_conv_cal_factors->PAR_P9 = (float)handle->dev_cal_factors->dig_P9 / powf(2.0f, 48.0f);
    handle->dev_conv_cal_factors->PAR_P10 = (float)handle->dev_cal_factors->dig_P10 / powf(2.0f, 48.0f);
    handle->dev_conv_cal_factors->PAR_P11 = (float)handle->dev_cal_factors->dig_P11 / powf(2.0f, 65.0f);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t bmp390_get_chip_id_register(bmp390_handle_t handle, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_CHIP_ID, reg), TAG, "read chip identifier register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_status_register(bmp390_handle_t handle, bmp390_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_STATUS, &reg->reg), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_interrupt_status_register(bmp390_handle_t handle, bmp390_interrupt_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_INT_STATUS, &reg->reg), TAG, "read interrupt status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_interrupt_control_register(bmp390_handle_t handle, bmp390_interrupt_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_INT_CNTRL, &reg->reg), TAG, "read interrupt control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_set_interrupt_control_register(bmp390_handle_t handle, const bmp390_interrupt_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    bmp390_interrupt_control_register_t interrupt_control = { .reg = reg.reg };

    /* set register reserved settings */
    interrupt_control.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(handle, BMP390_REG_INT_CNTRL, interrupt_control.reg), TAG, "write power control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_power_control_register(bmp390_handle_t handle, bmp390_power_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_PWRCTRL, &reg->reg), TAG, "read power control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_set_power_control_register(bmp390_handle_t handle, const bmp390_power_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    bmp390_power_control_register_t power_control = { .reg = reg.reg };

    /* set register reserved settings */
    power_control.bits.reserved1 = 0;
    power_control.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(handle, BMP390_REG_PWRCTRL, power_control.reg), TAG, "write power control register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_output_data_rate_register(bmp390_handle_t handle, bmp390_output_data_rate_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_ODR, &reg->reg), TAG, "read output data rate register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_set_output_data_rate_register(bmp390_handle_t handle, const bmp390_output_data_rate_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    bmp390_output_data_rate_register_t output_data_rate = { .reg = reg.reg };

    /* set register reserved settings */
    output_data_rate.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(handle, BMP390_REG_ODR, output_data_rate.reg), TAG, "write output data rate register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_oversampling_register(bmp390_handle_t handle, bmp390_oversampling_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_OSR, &reg->reg), TAG, "read oversampling register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_set_oversampling_register(bmp390_handle_t handle, const bmp390_oversampling_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    bmp390_oversampling_register_t oversampling = { .reg = reg.reg };

    /* set register reserved settings */
    oversampling.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(handle, BMP390_REG_OSR, oversampling.reg), TAG, "write oversampling register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_get_configuration_register(bmp390_handle_t handle, bmp390_configuration_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(handle, BMP390_REG_CONFIG, &reg->reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t bmp390_set_configuration_register(bmp390_handle_t handle, const bmp390_configuration_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register */
    bmp390_configuration_register_t config = { .reg = reg.reg };

    /* set register reserved settings */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(handle, BMP390_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t bmp390_setup(bmp390_handle_t handle) {
    /* configuration registers */
    bmp390_power_control_register_t     power_ctrl_reg;
    bmp390_configuration_register_t     config_reg;
    bmp390_oversampling_register_t      oversampling_reg;
    bmp390_output_data_rate_register_t  output_data_rate_reg;
    bmp390_interrupt_control_register_t interrupt_control_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR(bmp390_get_cal_factors(handle), TAG, "read calibration factors for get registers failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(bmp390_get_configuration_register(handle, &config_reg), TAG, "read configuration register for init failed");

    /* attempt to read oversampling register */
    ESP_RETURN_ON_ERROR(bmp390_get_oversampling_register(handle, &oversampling_reg), TAG, "read oversampling register for init failed");

    /* attempt to read to power control register */
    ESP_RETURN_ON_ERROR(bmp390_get_power_control_register(handle, &power_ctrl_reg), TAG, "read power control register for init failed");

    /* attempt to read to output data rate register */
    ESP_RETURN_ON_ERROR(bmp390_get_output_data_rate_register(handle, &output_data_rate_reg), TAG, "read output data rate register for init failed");

    /* attempt to read to interrupt control register */
    ESP_RETURN_ON_ERROR(bmp390_get_interrupt_control_register(handle, &interrupt_control_reg), TAG, "read interrupt control register for init failed");

    /* initialize configuration registers from configuration */
    output_data_rate_reg.bits.output_data_rate     = handle->dev_config.output_data_rate;
    config_reg.bits.iir_filter                     = handle->dev_config.iir_filter;
    power_ctrl_reg.bits.pressure_enabled           = true;
    power_ctrl_reg.bits.temperature_enabled        = true;
    power_ctrl_reg.bits.power_mode                 = handle->dev_config.power_mode;
    oversampling_reg.bits.temperature_oversampling = handle->dev_config.temperature_oversampling;
    oversampling_reg.bits.pressure_oversampling    = handle->dev_config.pressure_oversampling;
    interrupt_control_reg.bits.irq_data_ready_enabled = true;
    
    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR(bmp390_set_configuration_register(handle, config_reg), TAG, "write configuration register for init failed");

    /* attempt to write oversampling register */
    ESP_RETURN_ON_ERROR(bmp390_set_oversampling_register(handle, oversampling_reg), TAG, "write oversampling register for init failed");

    /* attempt to write to power control register */
    ESP_RETURN_ON_ERROR(bmp390_set_power_control_register(handle, power_ctrl_reg), TAG, "write power control register for init failed");

    /* attempt to write to output data rate register */
    ESP_RETURN_ON_ERROR(bmp390_set_output_data_rate_register(handle, output_data_rate_reg), TAG, "write output data rate register for init failed");

    /* attempt to write to interrupt control register */
    ESP_RETURN_ON_ERROR(bmp390_set_interrupt_control_register(handle, interrupt_control_reg), TAG, "write interrupt control register for init failed");

    return ESP_OK;
}

esp_err_t bmp390_init(i2c_master_bus_handle_t master_handle, const bmp390_config_t *bmp390_config, bmp390_handle_t *bmp390_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && bmp390_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, bmp390_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bmp280 device handle initialization failed", bmp390_config->i2c_address);

    /* validate memory availability for handle */
    bmp390_handle_t out_handle;
    out_handle = (bmp390_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device for init");

    /* validate memory availability for handle calibration factors */
    out_handle->dev_cal_factors = (bmp390_cal_factors_t*)calloc(1, sizeof(bmp390_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device calibration factors for init");

    /* validate memory availability for handle converted calibration factors */
    out_handle->dev_conv_cal_factors = (bmp390_conv_cal_factors_t*)calloc(1, sizeof(bmp390_conv_cal_factors_t));
    ESP_GOTO_ON_FALSE(out_handle->dev_conv_cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device converted calibration factors for init");

    /* copy configuration */
    out_handle->dev_config = *bmp390_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(bmp390_get_chip_id_register(out_handle, &out_handle->dev_type), err_handle, TAG, "read chip identifier for init failed");
    if(out_handle->dev_type != BMP390_CHIP_ID_DFLT) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", out_handle->dev_type);
    }

    /* attempt to reset the device and initialize handle registers */
    ESP_GOTO_ON_ERROR(bmp390_reset(out_handle), err_handle, TAG, "soft-reset and initialize registers for init failed");

    
    /* copy configuration */
    *bmp390_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t bmp390_get_measurements(bmp390_handle_t handle, float *const temperature, float *const pressure) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && temperature && pressure );

    /* initialize local variables */
    esp_err_t    ret                  = ESP_OK;
    uint64_t     start_time           = esp_timer_get_time(); /* set start time for timeout monitoring */
    bool         pressure_is_ready    = false;
    bool         temperature_is_ready = false;
    bit48_uint8_buffer_t rx                   = {};

    /* trigger measurement when in forced mode */
    if(handle->dev_config.power_mode == BMP390_POWER_MODE_FORCED) {
        bmp390_set_power_mode(handle, BMP390_POWER_MODE_FORCED);
    }

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( bmp390_get_data_status(handle, &temperature_is_ready, &pressure_is_ready), err, TAG, "data ready ready for get measurements failed." );
        
        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(BMP390_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, BMP390_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (pressure_is_ready == false && temperature_is_ready == false);

    // read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( bmp390_i2c_read_from(handle, BMP390_REG_PRESSURE, rx, BIT48_UINT8_BUFFER_SIZE), err, TAG, "read temperature and pressure data failed" );
    
    // concat pressure and temperature adc values
    uint32_t adc_press, adc_temp;
    uint32_t data_xlsb, data_lsb, data_msb;
    data_xlsb = (uint32_t)rx[0];
    data_lsb  = (uint32_t)rx[1] << 8;
    data_msb  = (uint32_t)rx[2] << 16;
    adc_press = data_msb | data_lsb | data_xlsb;
    data_xlsb = (uint32_t)rx[3];
    data_lsb  = (uint32_t)rx[4] << 8;
    data_msb  = (uint32_t)rx[5] << 16;
    adc_temp  = data_msb | data_lsb | data_xlsb;

    /* apply compensation and convert pressure and temperature values to engineering units of measure */
    *temperature = bmp390_compensate_temperature(handle, adc_temp);
    *pressure    = bmp390_compensate_pressure(handle, adc_press);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t bmp390_get_status(bmp390_handle_t handle, bool *const temperature_ready, bool *const pressure_ready, bool *const command_ready) {
    bmp390_status_register_t sts;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( bmp390_get_status_register(handle, &sts), TAG, "read status register (data ready state) failed" );

    /* set output parameters */
    *temperature_ready = sts.bits.temperature_data_ready;
    *pressure_ready    = sts.bits.pressure_data_ready;
    *command_ready     = sts.bits.command_ready;

    return ESP_OK;
}

esp_err_t bmp390_get_data_status(bmp390_handle_t handle, bool *const temperature_ready, bool *const pressure_ready) {
    bmp390_status_register_t sts;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( bmp390_get_status_register(handle, &sts), TAG, "read status register (data ready state) failed" );

    /* set output parameters */
    *temperature_ready = sts.bits.temperature_data_ready;
    *pressure_ready    = sts.bits.pressure_data_ready;

    return ESP_OK;
}

esp_err_t bmp390_get_power_mode(bmp390_handle_t handle, bmp390_power_modes_t *const power_mode) {
    bmp390_power_control_register_t pwrc;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read power control register */
    ESP_RETURN_ON_ERROR( bmp390_get_power_control_register(handle, &pwrc), TAG, "read power control register for get power mode failed" );

    /* set output parameter */
    *power_mode = pwrc.bits.power_mode;

    return ESP_OK;
}

esp_err_t bmp390_set_power_mode(bmp390_handle_t handle, const bmp390_power_modes_t power_mode) {
    bmp390_power_control_register_t pwrc;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read power control register */
    ESP_RETURN_ON_ERROR( bmp390_get_power_control_register(handle, &pwrc), TAG, "read power control register for get power mode failed" );

    /* set register setting */
    pwrc.bits.power_mode = power_mode;

    /* attempt to write power control register */
    ESP_RETURN_ON_ERROR( bmp390_set_power_control_register(handle, pwrc), TAG, "write power control register for set power mode failed" );

    /* set config parameter */
    handle->dev_config.power_mode = power_mode;

    return ESP_OK;
}

esp_err_t bmp390_get_pressure_oversampling(bmp390_handle_t handle, bmp390_pressure_oversampling_t *const oversampling) {
    bmp390_oversampling_register_t osmp;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bmp390_get_oversampling_register(handle, &osmp), TAG, "read oversampling register for get pressure oversampling failed" );

    /* set output parameter */
    *oversampling = osmp.bits.pressure_oversampling;

    return ESP_OK;
}

esp_err_t bmp390_set_pressure_oversampling(bmp390_handle_t handle, const bmp390_pressure_oversampling_t oversampling) {
    bmp390_oversampling_register_t osmp;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bmp390_get_oversampling_register(handle, &osmp), TAG, "read oversampling register for get pressure oversampling failed" );

    /* set register setting */
    osmp.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( bmp390_set_oversampling_register(handle, osmp), TAG, "write oversampling register for set pressure oversampling failed" );

    /* set config parameter */
    handle->dev_config.pressure_oversampling = oversampling;

    return ESP_OK;
}

esp_err_t bmp390_get_temperature_oversampling(bmp390_handle_t handle, bmp390_temperature_oversampling_t *const oversampling) {
    bmp390_oversampling_register_t osmp;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read oversampling register */
    ESP_RETURN_ON_ERROR( bmp390_get_oversampling_register(handle, &osmp), TAG, "read oversampling register for get temperature oversampling failed" );

    /* set output parameter */
    *oversampling = osmp.bits.temperature_oversampling;

    return ESP_OK;
}

esp_err_t bmp390_set_temperature_oversampling(bmp390_handle_t handle, const bmp390_temperature_oversampling_t oversampling) {
    bmp390_oversampling_register_t osmp;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read oversampling register */
    ESP_RETURN_ON_ERROR( bmp390_get_oversampling_register(handle, &osmp), TAG, "read oversampling register for get temperature oversampling failed" );

    /* set register setting */
    osmp.bits.temperature_oversampling = oversampling;

    /* attempt to write oversampling register */
    ESP_RETURN_ON_ERROR( bmp390_set_oversampling_register(handle, osmp), TAG, "write oversampling register for set temperature oversampling failed" );

    /* set config parameter */
    handle->dev_config.temperature_oversampling = oversampling;

    return ESP_OK;
}

esp_err_t bmp280_get_output_data_rate(bmp390_handle_t handle, bmp390_output_data_rates_t *const output_data_rate) {
    bmp390_output_data_rate_register_t odr;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bmp390_get_output_data_rate_register(handle, &odr), TAG, "read output data rate register for get standby time failed" );

    /* set output parameter */
    *output_data_rate = odr.bits.output_data_rate;

    return ESP_OK;
}

esp_err_t bmp280_set_output_data_rate(bmp390_handle_t handle, const bmp390_output_data_rates_t output_data_rate) {
    bmp390_output_data_rate_register_t odr;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bmp390_get_output_data_rate_register(handle, &odr), TAG, "read output data rate register for get standby time failed" );

    /* set register setting */
    odr.bits.output_data_rate  = output_data_rate;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( bmp390_set_output_data_rate_register(handle, odr), TAG, "write output data rate register for set stanby time failed" );

    /* set config parameter */
    handle->dev_config.output_data_rate = output_data_rate;

    return ESP_OK;
}

esp_err_t bmp390_get_iir_filter(bmp390_handle_t handle, bmp390_iir_filters_t *const iir_filter) {
    bmp390_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bmp390_get_configuration_register(handle, &config), TAG, "read configuration register for get IIR filter failed" );

    /* set output parameter */
    *iir_filter = config.bits.iir_filter;

    return ESP_OK;
}

esp_err_t bmp390_set_iir_filter(bmp390_handle_t handle, const bmp390_iir_filters_t iir_filter) {
    bmp390_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bmp390_get_configuration_register(handle, &config), TAG, "read configuration register for get IIR filter failed" );

    /* set register setting */
    config.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( bmp390_set_configuration_register(handle, config), TAG, "write configuration register for set IIR filter failed" );

    /* set config parameter */
    handle->dev_config.iir_filter = iir_filter;

    return ESP_OK;
}

esp_err_t bmp390_reset(bmp390_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(handle, BMP390_REG_CMD, BMP390_SFTRESET_CMD), TAG, "write reset register for reset failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(BMP390_RESET_DELAY_MS)); // check is busy in timeout loop...

    /* attempt to setup device  */
    ESP_RETURN_ON_ERROR( bmp390_setup(handle), TAG, "setup for reset failed" );

    return ESP_OK;
}

esp_err_t bmp390_remove(bmp390_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t bmp390_delete(bmp390_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( bmp390_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle->dev_cal_factors);
        free(handle->dev_conv_cal_factors);
        free(handle);
    }

    return ESP_OK;
}

const char* bmp390_get_fw_version(void) {
    return BMP390_FW_VERSION_STR;
}

int32_t bmp390_get_fw_version_number(void) {
    return BMP390_FW_VERSION_INT32;
}
