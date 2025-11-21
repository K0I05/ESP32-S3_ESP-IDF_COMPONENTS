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
 * @file bmp390.c
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

/**
 * @brief BMP390 status register (0x03) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp390_status_register_u {
    struct {
        uint8_t reserved1:4;    			/*!< bmp390 reserved (bit:0-3) */
        bool    command_ready:1;			/*!< bmp390 command decoder is ready to accept a new command when true                      (bit:4) */
		bool    pressure_data_ready:1;		/*!< bmp390 pressure data ready when true and it is reset when data register is read out    (bit:5) */
		bool    temperature_data_ready:1;	/*!< bmp390 temperature data ready when true and it is reset when data register is read out (bit:6) */
        uint8_t reserved2:1;    			/*!< bmp390 reserved (bit:7) */
    } bits;
    uint8_t reg;
} bmp390_status_register_t;

/**
 * @brief BMP390 interrupt status register (0x11) structure.  The reset state is ? for this register.
 */
typedef union __attribute__((packed)) bmp390_interrupt_status_register_u {
    struct {
        bool    fifo_watermark_irq:1;  /*!< bmp390 FIFO watermark interrupt, cleared after reading       (bit:0) */
		bool    fifo_full_irq:1;	   /*!< bmp390 FIFO full interrupt, cleared after reading            (bit:1) */
        uint8_t reserved1:1;    	   /*!< bmp390 reserved (bit:2) */
		bool    data_ready_irq:1;	   /*!< bmp390 data ready interrupt, cleared after reading           (bit:3) */
        uint8_t reserved2:4;    	   /*!< bmp390 reserved (bit:4-7) */
    } bits;
    uint8_t reg;
} bmp390_interrupt_status_register_t;

/**
 * @brief BMP390 interrupt control register (0x19) structure.  The reset state is 0x02 for this register.
 */
typedef union __attribute__((packed)) bmp390_interrupt_control_register_u {
    struct {
        bool    irq_output:1;       /*!< bmp390 open-drain (true) or push-pull (false)            (bit:0) */
		bool    irq_level:1;	     /*!< bmp390 active-high (true) or active-low (false)         (bit:1) */
        bool    irq_latch_enabled:1;         /*!< bmp390 latching of interrupt pin is enabled when true   (bit:2) */
        bool    irq_fifo_watermark_enabled:1; /*!< bmp390         (bit:3) */
        bool    irq_fifo_full_enabled:1;	/*!< bmp390     (bit:4) */
        bool    irq_ds:1;	            /*!< bmp390 high (true) or low (false)    (bit:5) */
        bool    irq_data_ready_enabled:1;	/*!<     (bit:6) */
        uint8_t reserved:1;    	   /*!< bmp390 reserved (bit:7) */
    } bits;
    uint8_t reg;
} bmp390_interrupt_control_register_t;

/**
 * @brief BMP390 power control register (0x1b) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp390_power_control_register_u {
    struct {
		bool						pressure_enabled:1;		/*!< bmp390 pressure sensor enabled when true     (bit:0) */
		bool						temperature_enabled:1;	/*!< bmp390 temperature sensor enabled when true  (bit:1) */
		uint8_t						reserved1:2;			/*!< bmp380 reserved                              (bit:2-3) */
        bmp390_power_modes_t	    power_mode:2;           /*!< bmp390 power mode of the device              (bit:4-5)  */
        uint8_t						reserved2:2;    		/*!< bmp390 reserved                              (bit:6-7) */
    } bits;
    uint8_t reg;
} bmp390_power_control_register_t;

/**
 * @brief BMP390 OSR register (0x1c) structure.  The reset state is 0x02 for this register.
 */
typedef union __attribute__((packed)) bmp390_oversampling_register_u {
    struct {
        bmp390_pressure_oversampling_t      pressure_oversampling:3;    /*!< bmp390 oversampling of pressure data       (bit:0-2) */
        bmp390_temperature_oversampling_t   temperature_oversampling:3; /*!< bmp390 oversampling of temperature data    (bit:3-5) */
        uint8_t						        reserved:2;    		        /*!< bmp390 reserved                            (bit:6-7) */
    } bits;
    uint8_t reg;
} bmp390_oversampling_register_t;

/**
 * @brief BMP390 ODR register (0x1d) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp390_output_data_rate_register_u {
    struct {
        bmp390_output_data_rates_t output_data_rate:5; /*!< bmp390 output data rate       (bit:0-4) */
        uint8_t					   reserved:3;    	   /*!< bmp390 reserved               (bit:5-7) */
    } bits;
    uint8_t reg;
} bmp390_output_data_rate_register_t;

/**
 * @brief BMP390 configuration register (0x1f) structure.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp390_config_register_u {
    struct {
        uint8_t                 reserved1:1;    /*!< bmp390 reserved                                (bit:0) */
        bmp390_iir_filters_t    iir_filter:3;   /*!< bmp390 time constant of the IIR filter         (bit:1-3) */
        uint8_t                 reserved2:4;    /*!< bmp390 reserved                                (bit:4-7) */
    } bits;
    uint8_t reg;
} bmp390_config_register_t;


/**
 * @brief BMP390 temperature and pressure calibration factors structure.
 */
typedef struct bmp390_cal_factors_s {
    /* temperature and pressure compensation */
    uint16_t                dig_T1;
    uint16_t                dig_T2;
    int8_t                  dig_T3;
    int16_t                 dig_P1;
    int16_t                 dig_P2;
    int8_t                  dig_P3;
    int8_t                  dig_P4;
    uint16_t                dig_P5;
    uint16_t                dig_P6;
    int8_t                  dig_P7;
    int8_t                  dig_P8;
    int16_t                 dig_P9;
	int8_t                  dig_P10;
	int8_t                  dig_P11;
} bmp390_cal_factors_t;

/**
 * @brief BMP390 temperature and pressure converted calibration factors structure.
 */
typedef struct bmp390_conv_cal_factors_s {
    /* temperature and pressure compensation */
    double                PAR_T1;
    double                PAR_T2;
    double                PAR_T3;
    double                PAR_P1;
    double                PAR_P2;
    double                PAR_P3;
    double                PAR_P4;
    double                PAR_P5;
    double                PAR_P6;
    double                PAR_P7;
    double                PAR_P8;
    double                PAR_P9;
	double                PAR_P10;
	double                PAR_P11;
    double                t_lin;
} bmp390_conv_cal_factors_t;

/**
 * @brief BMP390 device structure definition.
 */
typedef struct bmp390_device_s {
    bmp390_config_t                         config;                 /*!< bmp390 device configuration */
    i2c_master_dev_handle_t                 i2c_handle;             /*!< bmp380 i2c device handle */
    bmp390_conv_cal_factors_t              *cal_factors;            /*!< bmp390 device calibration factors converted to floating point numbers (section 8.4)*/
    uint8_t                                 type;                   /*!< device type, should be bmp390 */
} bmp390_device_t;

/*
* static constant declarations
*/
static const char *TAG = "bmp390";


/**
 * @brief BMP390 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device BMP390 device descriptor.
 * @param reg_addr BMP390 register address to read from.
 * @param buffer BMP390 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_read_from(bmp390_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "bmp390_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read word from register address transaction.
 * 
 * @param device BMP390 device descriptor.
 * @param reg_addr BMP390 register address to read from.
 * @param word BMP390 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_read_word_from(bmp390_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bmp390_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read byte from register address transaction.
 * 
 * @param device BMP390 device descriptor.
 * @param reg_addr BMP390 register address to read from.
 * @param byte BMP390 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_read_byte_from(bmp390_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bmp390_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write byte to register address transaction.
 * 
 * @param device BMP390 device descriptor.
 * @param reg_addr BMP390 register address to write to.
 * @param byte BMP390 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_write_byte_to(bmp390_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Temperature compensation algorithm is taken from BMP390 datasheet.  See datasheet for details.
 *
 * @param[in] device BMP390 device descriptor.
 * @param[in] adc_temperature Raw adc temperature.
 * @return Compensated temperature in degrees Celsius.
 */
static inline double bmp390_compensate_temperature(bmp390_device_t *const device, const uint32_t adc_temperature) {
    const double var1 = (double)(adc_temperature - device->cal_factors->PAR_T1);
    const double var2 = (double)(var1 * device->cal_factors->PAR_T2);
    //
    device->cal_factors->t_lin = var2 + (var1 * var1) * device->cal_factors->PAR_T3;

    return device->cal_factors->t_lin;
}

/**
 * @brief Pressure compensation algorithm is taken from BMP390 datasheet.  See datasheet for details.
 *
 * @param[in] device BMP390 device descriptor.
 * @param[in] adc_pressure Raw adc pressure.
 * @return Compensated pressure in pascal.
 */
static inline double bmp390_compensate_pressure(bmp390_device_t *const device, const uint32_t adc_pressure) {
    double dat1 = device->cal_factors->PAR_P6 * device->cal_factors->t_lin;
    double dat2 = device->cal_factors->PAR_P7 * device->cal_factors->t_lin * device->cal_factors->t_lin;
    double dat3 = device->cal_factors->PAR_P8 * device->cal_factors->t_lin * device->cal_factors->t_lin * device->cal_factors->t_lin;
    const double var1 = device->cal_factors->PAR_P5 + dat1 + dat2 + dat3;
    //
    dat1 = device->cal_factors->PAR_P2 * device->cal_factors->t_lin;
    dat2 = device->cal_factors->PAR_P3 * device->cal_factors->t_lin * device->cal_factors->t_lin;
    dat3 = device->cal_factors->PAR_P4 * device->cal_factors->t_lin * device->cal_factors->t_lin * device->cal_factors->t_lin;
    const double var2 = (double)adc_pressure * (device->cal_factors->PAR_P1 + dat1 + dat2 + dat3);
    //
    dat1 = (double)adc_pressure * (double)adc_pressure;
    dat2 = device->cal_factors->PAR_P9 + device->cal_factors->PAR_P10 * device->cal_factors->t_lin;
    dat3 = dat1 * dat2;
    const double dat4 = dat3 + (double)adc_pressure * (double)adc_pressure * (double)adc_pressure * device->cal_factors->PAR_P11;
    //
    return var1 + var2 + dat4;
}

/**
 * @brief BMP390 I2C HAL read calibration factors and apply floating point correction factors.  See datasheet for details.
 *
 * @param[in] device BMP390 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_cal_factors(bmp390_device_t *const device) {
    bmp390_cal_factors_t cal_factors = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* bmp280 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x31, &cal_factors.dig_T1) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x33, &cal_factors.dig_T2) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x35, (uint8_t *)&cal_factors.dig_T3) );
    /* bmp280 attempt to request P1-P10 calibration values from device */
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x36, (uint16_t *)&cal_factors.dig_P1) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x38, (uint16_t *)&cal_factors.dig_P2) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x3a, (uint8_t *)&cal_factors.dig_P3) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x3b, (uint8_t *)&cal_factors.dig_P4) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x3c, &cal_factors.dig_P5) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x3e, &cal_factors.dig_P6) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x40, (uint8_t *)&cal_factors.dig_P7) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x41, (uint8_t *)&cal_factors.dig_P8) );
    ESP_ERROR_CHECK( bmp390_i2c_read_word_from(device, 0x42, (uint16_t *)&cal_factors.dig_P9) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x44, (uint8_t *)&cal_factors.dig_P10) );
    ESP_ERROR_CHECK( bmp390_i2c_read_byte_from(device, 0x45, (uint8_t *)&cal_factors.dig_P11) );

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
    device->cal_factors->PAR_T1 = (float)cal_factors.dig_T1 / powf(2.0f, -8.0f);
    device->cal_factors->PAR_T2 = (float)cal_factors.dig_T2 / powf(2.0f, 30.0f);
    device->cal_factors->PAR_T3 = (float)cal_factors.dig_T3 / powf(2.0f, 48.0f);
    device->cal_factors->PAR_P1 = ((float)cal_factors.dig_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
    device->cal_factors->PAR_P2 = ((float)cal_factors.dig_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
    device->cal_factors->PAR_P3 = (float)cal_factors.dig_P3 / powf(2.0f, 32.0f);
    device->cal_factors->PAR_P4 = (float)cal_factors.dig_P4 / powf(2.0f, 37.0f);
    device->cal_factors->PAR_P5 = (float)cal_factors.dig_P5 / powf(2.0f, -3.0f);
    device->cal_factors->PAR_P6 = (float)cal_factors.dig_P6 / powf(2.0f, 6.0f);
    device->cal_factors->PAR_P7 = (float)cal_factors.dig_P7 / powf(2.0f, 8.0f);
    device->cal_factors->PAR_P8 = (float)cal_factors.dig_P8 / powf(2.0f, 15.0f);
    device->cal_factors->PAR_P9 = (float)cal_factors.dig_P9 / powf(2.0f, 48.0f);
    device->cal_factors->PAR_P10 = (float)cal_factors.dig_P10 / powf(2.0f, 48.0f);
    device->cal_factors->PAR_P11 = (float)cal_factors.dig_P11 / powf(2.0f, 65.0f);

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read chip identification register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 chip identification register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_chip_id_register(bmp390_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_CHIP_ID, reg), TAG, "read chip identifier register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read status register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_status_register(bmp390_device_t *const device, bmp390_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_STATUS, &reg->reg), TAG, "read status register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read interrupt control register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 interrupt status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_interrupt_status_register(bmp390_device_t *const device, bmp390_interrupt_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_INT_STATUS, &reg->reg), TAG, "read interrupt status register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read interrupt control register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 interrupt control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_interrupt_control_register(bmp390_device_t *const device, bmp390_interrupt_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_INT_CNTRL, &reg->reg), TAG, "read interrupt control register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write interrupt control register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[in] reg BMP390 interrupt control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_set_interrupt_control_register(bmp390_device_t *const device, const bmp390_interrupt_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    bmp390_interrupt_control_register_t interrupt_control = { .reg = reg.reg };

    /* set register reserved settings */
    interrupt_control.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(device, BMP390_REG_INT_CNTRL, interrupt_control.reg), TAG, "write power control register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read power control register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 power control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_power_control_register(bmp390_device_t *const device, bmp390_power_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_PWRCTRL, &reg->reg), TAG, "read power control register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write power control register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[in] reg BMP390 power control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_set_power_control_register(bmp390_device_t *const device, const bmp390_power_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    bmp390_power_control_register_t power_control = { .reg = reg.reg };

    /* set register reserved settings */
    power_control.bits.reserved1 = 0;
    power_control.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(device, BMP390_REG_PWRCTRL, power_control.reg), TAG, "write power control register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read output data rate register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 output data rate register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_output_data_rate_register(bmp390_device_t *const device, bmp390_output_data_rate_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_ODR, &reg->reg), TAG, "read output data rate register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write output data rate register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[in] reg BMP390 output data rate register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_set_output_data_rate_register(bmp390_device_t *const device, const bmp390_output_data_rate_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    bmp390_output_data_rate_register_t output_data_rate = { .reg = reg.reg };

    /* set register reserved settings */
    output_data_rate.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(device, BMP390_REG_ODR, output_data_rate.reg), TAG, "write output data rate register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read oversampling register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 oversampling register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_oversampling_register(bmp390_device_t *const device, bmp390_oversampling_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_OSR, &reg->reg), TAG, "read oversampling register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write oversampling register. 
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[in] reg BMP390 oversampling register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_set_oversampling_register(bmp390_device_t *const device, const bmp390_oversampling_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    bmp390_oversampling_register_t oversampling = { .reg = reg.reg };

    /* set register reserved settings */
    oversampling.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(device, BMP390_REG_OSR, oversampling.reg), TAG, "write oversampling register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL read configuration register.
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[out] reg BMP390 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_config_register(bmp390_device_t *const device, bmp390_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_read_byte_from(device, BMP390_REG_CONFIG, &reg->reg), TAG, "read configuration register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write configuration register. 
 * 
 * @param[in] device BMP390 device descriptor.
 * @param[in] reg BMP390 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_set_config_register(bmp390_device_t *const device, const bmp390_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    bmp390_config_register_t config = { .reg = reg.reg };

    /* set register reserved settings */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(device, BMP390_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL write reset register.
 * 
 * @param device BMP390 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_set_reset_register(bmp390_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_write_byte_to(device, BMP390_REG_CMD, BMP390_SFTRESET_CMD), TAG, "write reset register failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(BMP390_RESET_DELAY_MS)); // check is busy in timeout loop...

    return ESP_OK;
}

/**
 * @brief BMP390 I2C HAL to get raw adc temperature and pressure signals.  This function will trigger a measurement if in forced mode and then poll the status register.
 * 
 * @param device BMP390 device descriptor.
 * @param temperature Raw adc temperature signal.
 * @param pressure Raw adc pressure signal.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_get_adc_signals(bmp390_device_t *const device, uint32_t *const temperature, uint32_t *const pressure) {
    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && pressure );

    /* initialize local variables */
    esp_err_t      ret                  = ESP_OK;
    const uint64_t start_time           = esp_timer_get_time(); /* set start time for timeout monitoring */
    bool           pressure_is_ready    = false;
    bool           temperature_is_ready = false;
    bit48_uint8_buffer_t rx           = {};

    /* trigger measurement when in forced mode */
    if(device->config.power_mode == BMP390_POWER_MODE_FORCED) {
        bmp390_power_control_register_t pwrc = { 0 };

        /* attempt to read power control register */
        ESP_RETURN_ON_ERROR( bmp390_i2c_get_power_control_register(device, &pwrc), TAG, "read power control register for get adc signals failed" );

        /* set register setting */
        pwrc.bits.power_mode = device->config.power_mode;

        /* attempt to write power control register */
        ESP_RETURN_ON_ERROR( bmp390_i2c_set_power_control_register(device, pwrc), TAG, "write power control register for set adc signals failed" );
    }

    /* attempt to poll until data is available or timeout */
    do {
        bmp390_status_register_t sts = { 0 };

        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( bmp390_i2c_get_status_register(device, &sts), err, TAG, "data ready ready for get adc signals failed." );

        pressure_is_ready    = sts.bits.pressure_data_ready;
        temperature_is_ready = sts.bits.temperature_data_ready;
        
        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(BMP390_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, BMP390_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (pressure_is_ready == false && temperature_is_ready == false);

    // read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( bmp390_i2c_read_from(device, BMP390_REG_PRESSURE, rx, BIT48_UINT8_BUFFER_SIZE), err, TAG, "read temperature and pressure data failed" );
    
    // concat pressure and temperature adc values
    const uint32_t pdata_xlsb = (uint32_t)rx[0];
    const uint32_t pdata_lsb  = (uint32_t)rx[1] << 8;
    const uint32_t pdata_msb  = (uint32_t)rx[2] << 16;
    const uint32_t adc_press = pdata_msb | pdata_lsb | pdata_xlsb;
    const uint32_t tdata_xlsb = (uint32_t)rx[3];
    const uint32_t tdata_lsb  = (uint32_t)rx[4] << 8;
    const uint32_t tdata_msb  = (uint32_t)rx[5] << 16;
    const uint32_t adc_temp  = tdata_msb | tdata_lsb | tdata_xlsb;

    /* set output parameters */
    *temperature = adc_temp;
    *pressure    = adc_press;

    return ESP_OK;

    err:
        return ret;
}

/**
 * @brief BMP390 I2C HAL setup and configuration of registers.
 * 
 * @param device BMP390 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bmp390_i2c_setup_registers(bmp390_device_t *const device) {
    /* configuration registers */
    bmp390_power_control_register_t     power_ctrl_reg = { 0 };
    bmp390_config_register_t            config_reg = { 0 };
    bmp390_oversampling_register_t      oversampling_reg = { 0 };
    bmp390_output_data_rate_register_t  output_data_rate_reg = { 0 };
    bmp390_interrupt_control_register_t interrupt_control_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR(bmp390_i2c_get_cal_factors(device), TAG, "read calibration factors for get registers failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_get_config_register(device, &config_reg), TAG, "read configuration register for init failed");

    /* attempt to read oversampling register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_get_oversampling_register(device, &oversampling_reg), TAG, "read oversampling register for init failed");

    /* attempt to read to power control register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_get_power_control_register(device, &power_ctrl_reg), TAG, "read power control register for init failed");

    /* attempt to read to output data rate register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_get_output_data_rate_register(device, &output_data_rate_reg), TAG, "read output data rate register for init failed");

    /* attempt to read to interrupt control register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_get_interrupt_control_register(device, &interrupt_control_reg), TAG, "read interrupt control register for init failed");

    /* initialize configuration registers from configuration */
    output_data_rate_reg.bits.output_data_rate        = device->config.output_data_rate;
    config_reg.bits.iir_filter                        = device->config.iir_filter;
    power_ctrl_reg.bits.pressure_enabled              = true;
    power_ctrl_reg.bits.temperature_enabled           = true;
    power_ctrl_reg.bits.power_mode                    = device->config.power_mode;
    oversampling_reg.bits.temperature_oversampling    = device->config.temperature_oversampling;
    oversampling_reg.bits.pressure_oversampling       = device->config.pressure_oversampling;
    interrupt_control_reg.bits.irq_data_ready_enabled = true;
    
    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_set_config_register(device, config_reg), TAG, "write configuration register for init failed");

    /* attempt to write oversampling register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_set_oversampling_register(device, oversampling_reg), TAG, "write oversampling register for init failed");

    /* attempt to write to power control register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_set_power_control_register(device, power_ctrl_reg), TAG, "write power control register for init failed");

    /* attempt to write to output data rate register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_set_output_data_rate_register(device, output_data_rate_reg), TAG, "write output data rate register for init failed");

    /* attempt to write to interrupt control register */
    ESP_RETURN_ON_ERROR(bmp390_i2c_set_interrupt_control_register(device, interrupt_control_reg), TAG, "write interrupt control register for init failed");

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
    bmp390_device_t* device = (bmp390_device_t*)calloc(1, sizeof(bmp390_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bmp280 device for init");

    /* validate memory availability for handle converted calibration factors */
    device->cal_factors = (bmp390_conv_cal_factors_t*)calloc(1, sizeof(bmp390_conv_cal_factors_t));
    ESP_GOTO_ON_FALSE(device->cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bmp280 device converted calibration factors for init");

    /* copy configuration */
    device->config = *bmp390_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(bmp390_i2c_get_chip_id_register(device, &device->type), err_handle, TAG, "read chip identifier for init failed");
    if(device->type != BMP390_CHIP_ID_DFLT) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", device->type);
    }

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(bmp390_i2c_set_reset_register(device), err_handle, TAG, "soft-reset register for init failed");

    /* attempt to setup the device */
    ESP_GOTO_ON_ERROR(bmp390_i2c_setup_registers(device), err_handle, TAG, "setup for init failed");

    /* copy configuration */
    *bmp390_handle = (bmp390_handle_t)device;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP390_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t bmp390_get_measurements(bmp390_handle_t handle, float *const temperature, float *const pressure) {
    uint32_t adc_press, adc_temp;
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && pressure );

    // read in one sequence to ensure they match.
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_adc_signals(device, &adc_temp, &adc_press), TAG, "read temperature and pressure adc signals failed" );

    /* apply compensation and convert pressure and temperature values to engineering units of measure */
    *temperature = bmp390_compensate_temperature(device, adc_temp);
    *pressure    = bmp390_compensate_pressure(device, adc_press);

    return ESP_OK;
}

esp_err_t bmp390_get_status(bmp390_handle_t handle, bool *const temperature_ready, bool *const pressure_ready, bool *const command_ready) {
    bmp390_status_register_t sts = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_status_register(device, &sts), TAG, "read status register (data ready state) failed" );

    /* set output parameters */
    *temperature_ready = sts.bits.temperature_data_ready;
    *pressure_ready    = sts.bits.pressure_data_ready;
    *command_ready     = sts.bits.command_ready;

    return ESP_OK;
}

esp_err_t bmp390_get_data_status(bmp390_handle_t handle, bool *const temperature_ready, bool *const pressure_ready) {
    bmp390_status_register_t sts = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_status_register(device, &sts), TAG, "read status register (data ready state) failed" );

    /* set output parameters */
    *temperature_ready = sts.bits.temperature_data_ready;
    *pressure_ready    = sts.bits.pressure_data_ready;

    return ESP_OK;
}

esp_err_t bmp390_get_power_mode(bmp390_handle_t handle, bmp390_power_modes_t *const power_mode) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *power_mode = device->config.power_mode;

    return ESP_OK;
}

esp_err_t bmp390_set_power_mode(bmp390_handle_t handle, const bmp390_power_modes_t power_mode) {
    bmp390_power_control_register_t pwrc = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read power control register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_power_control_register(device, &pwrc), TAG, "read power control register for get power mode failed" );

    /* set register setting */
    pwrc.bits.power_mode = power_mode;

    /* attempt to write power control register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_set_power_control_register(device, pwrc), TAG, "write power control register for set power mode failed" );

    /* set config parameter */
    device->config.power_mode = power_mode;

    return ESP_OK;
}

esp_err_t bmp390_get_pressure_oversampling(bmp390_handle_t handle, bmp390_pressure_oversampling_t *const oversampling) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *oversampling = device->config.pressure_oversampling;

    return ESP_OK;
}

esp_err_t bmp390_set_pressure_oversampling(bmp390_handle_t handle, const bmp390_pressure_oversampling_t oversampling) {
    bmp390_oversampling_register_t osmp = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_oversampling_register(device, &osmp), TAG, "read oversampling register for get pressure oversampling failed" );

    /* set register setting */
    osmp.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_set_oversampling_register(device, osmp), TAG, "write oversampling register for set pressure oversampling failed" );

    /* set config parameter */
    device->config.pressure_oversampling = oversampling;

    return ESP_OK;
}

esp_err_t bmp390_get_temperature_oversampling(bmp390_handle_t handle, bmp390_temperature_oversampling_t *const oversampling) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *oversampling = device->config.temperature_oversampling;

    return ESP_OK;
}

esp_err_t bmp390_set_temperature_oversampling(bmp390_handle_t handle, const bmp390_temperature_oversampling_t oversampling) {
    bmp390_oversampling_register_t osmp = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read oversampling register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_oversampling_register(device, &osmp), TAG, "read oversampling register for get temperature oversampling failed" );

    /* set register setting */
    osmp.bits.temperature_oversampling = oversampling;

    /* attempt to write oversampling register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_set_oversampling_register(device, osmp), TAG, "write oversampling register for set temperature oversampling failed" );

    /* set config parameter */
    device->config.temperature_oversampling = oversampling;

    return ESP_OK;
}

esp_err_t bmp280_get_output_data_rate(bmp390_handle_t handle, bmp390_output_data_rates_t *const output_data_rate) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *output_data_rate = device->config.output_data_rate;

    return ESP_OK;
}

esp_err_t bmp280_set_output_data_rate(bmp390_handle_t handle, const bmp390_output_data_rates_t output_data_rate) {
    bmp390_output_data_rate_register_t odr = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_output_data_rate_register(device, &odr), TAG, "read output data rate register for get standby time failed" );

    /* set register setting */
    odr.bits.output_data_rate  = output_data_rate;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_set_output_data_rate_register(device, odr), TAG, "write output data rate register for set stanby time failed" );

    /* set config parameter */
    device->config.output_data_rate = output_data_rate;

    return ESP_OK;
}

esp_err_t bmp390_get_iir_filter(bmp390_handle_t handle, bmp390_iir_filters_t *const iir_filter) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *iir_filter = device->config.iir_filter;

    return ESP_OK;
}

esp_err_t bmp390_set_iir_filter(bmp390_handle_t handle, const bmp390_iir_filters_t iir_filter) {
    bmp390_config_register_t config = { 0 };
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_get_config_register(device, &config), TAG, "read configuration register for get IIR filter failed" );

    /* set register setting */
    config.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( bmp390_i2c_set_config_register(device, config), TAG, "write configuration register for set IIR filter failed" );

    /* set config parameter */
    device->config.iir_filter = iir_filter;

    return ESP_OK;
}

esp_err_t bmp390_reset(bmp390_handle_t handle) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( bmp390_i2c_set_reset_register(device), TAG, "write reset register for reset failed" );

    /* attempt to setup device  */
    ESP_RETURN_ON_ERROR( bmp390_i2c_setup_registers(device), TAG, "setup for reset failed" );

    return ESP_OK;
}

esp_err_t bmp390_remove(bmp390_handle_t handle) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate handle instance */
    if(device->i2c_handle) {
        /* remove device from i2c master bus */
        esp_err_t ret = i2c_master_bus_rm_device(device->i2c_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_rm_device failed");
            return ret;
        }
        device->i2c_handle = NULL;
    }

    return ESP_OK;
}

esp_err_t bmp390_delete(bmp390_handle_t handle) {
    bmp390_device_t* device = (bmp390_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from master bus */
    esp_err_t ret = bmp390_remove(handle);

    /* validate handle instance and free handles */
    if(device->cal_factors) {
        free(device->cal_factors);
    }
    free(handle);

    return ret;
}

const char* bmp390_get_fw_version(void) {
    return (const char*)BMP390_FW_VERSION_STR;
}

int32_t bmp390_get_fw_version_number(void) {
    return (int32_t)BMP390_FW_VERSION_INT32;
}
