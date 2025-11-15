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
 * @file bme680.c
 *
 * ESP-IDF driver for BME680 temperature, humidity, pressure, and gas sensor
 *
 * Ported from esp-open-rtos
 * 
 * https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c
 * 
 * iaq: https://github.com/3KUdelta/heltec_wifi_kit_32_BME680/blob/master/Wifi_Kit_32_BME680.ino
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/bme680.h"
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
 * possible BME680 registers
 */

#define BME680_REG_STATUS0          UINT8_C(0x1D)
#define BME680_REG_RESET            UINT8_C(0xE0) /*!< reset value: 0xB6 */
#define BME680_REG_ID               UINT8_C(0xD0)
#define BME680_REG_CONFIG           UINT8_C(0x75)
#define BME680_REG_CTRL_MEAS        UINT8_C(0x74)
#define BME680_REG_CTRL_HUMI        UINT8_C(0x72)
#define BME680_REG_CTRL_GAS1        UINT8_C(0x71)
#define BME680_REG_CTRL_GAS0        UINT8_C(0x70)
#define BME680_REG_GAS_WAIT         UINT8_C(0x64) /*!< gas_wait_x:  0x64 to 0x6D */
#define BME680_REG_RES_HEAT         UINT8_C(0x5A) /*!< res_heat_x:  0x5A to 0x63 */
#define BME680_REG_IDAC_HEAT        UINT8_C(0x50) /*!< idac_heat_x: 0x50 to 0x59 */
#define BME680_REG_GAS_R_LSB        UINT8_C(0x2B)
#define BME680_REG_GAS_R_MSB        UINT8_C(0x2A)
#define BME680_REG_GAS_R            UINT8_C(BME680_REG_GAS_R_MSB)
#define BME680_REG_HUMI_LSB         UINT8_C(0x26)
#define BME680_REG_HUMI_MSB         UINT8_C(0x25)
#define BME680_REG_HUMI             UINT8_C(BME680_REG_HUMI_MSB)
#define BME680_REG_TEMP_XLSB        UINT8_C(0x24)
#define BME680_REG_TEMP_LSB         UINT8_C(0x23)
#define BME680_REG_TEMP_MSB         UINT8_C(0x22)
#define BME680_REG_TEMP             UINT8_C(BME680_REG_TEMP_MSB)
#define BME680_REG_PRESS_XLSB       UINT8_C(0x21)
#define BME680_REG_PRESS_LSB        UINT8_C(0x20)
#define BME680_REG_PRESS_MSB        UINT8_C(0x1F)
#define BME680_REG_PRESS            UINT8_C(BME680_REG_PRESS_MSB)
#define BME680_RESET_VALUE          UINT8_C(0xB6)
#define BME680_REG_SHD_HEATR_DUR    UINT8_C(0x6E)  /* Shared heating duration address */

#define BME680_REG_VARIANT_ID       UINT8_C(0xF0)

#define BME680_VARIANT_GAS_LOW      UINT8_C(0x00)  /* Low Gas variant */
#define BME680_VARIANT_GAS_HIGH     UINT8_C(0x01)  /* High Gas variant */

#define BME680_ENABLE_GAS_MEAS_L    UINT8_C(0x01) /* Enable gas measurement low */
#define BME680_ENABLE_GAS_MEAS_H    UINT8_C(0x02) /* Enable gas measurement high */

// field data 1 registers (not documented, used in SEQUENTIAL_MODE)
#define BME680_REG_MEAS_STATUS_1    UINT8_C(0x2e)
#define BME680_REG_MEAS_INDEX_1     UINT8_C(0x2f)

// field data 2 registers (not documented, used in SEQUENTIAL_MODE)
#define BME680_REG_MEAS_STATUS_2    UINT8_C(0x3f)
#define BME680_REG_MEAS_INDEX_2     UINT8_C(0x40)

#define BME680_CHIP_ID              UINT8_C(0x61)

#define BME680_AQI_TEMP_CORR               (-3) // Calibration offset - calibrate yourself the temp reading --> humidity will 
                                                // be automatically adjusted using the August-Roche-Magnus approximation
                                                // http://bmcnoldy.rsmas.miami.edu/Humidity.html


#define BME680_DATA_POLL_TIMEOUT_MS UINT16_C(1500) // ? see datasheet tables 13 and 14, standby-time could be 2-seconds (2000ms)
#define BME680_DATA_READY_DELAY_MS  UINT16_C(1)
#define BME680_POWERUP_DELAY_MS     UINT16_C(25)
#define BME680_APPSTART_DELAY_MS    UINT16_C(25)
#define BME680_RESET_DELAY_MS       UINT16_C(25)
#define BME680_CMD_DELAY_MS         UINT16_C(5)
#define BME680_TX_RX_DELAY_MS       UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief BME680 device descriptor structure definition.
 */
typedef struct bme680_device_s {
    bme680_config_t                         config;             /*!< bme680 device configuration */
    i2c_master_dev_handle_t                 i2c_handle;         /*!< bme680 I2C device handle */
    bme680_cal_factors_t                   *cal_factors;        /*!< bme680 device calibration factors */
    uint8_t                                 chip_id;            /*!< bme680 chip identification register */
    uint16_t                                ambient_temperature;
    uint8_t                                 variant_id;
} bme680_device_t;

/*
* static constant declarations
*/
static const char *TAG = "bme680";


/**
 * @brief BME680 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device BME680 device descriptor.
 * @param reg_addr BME680 register address to read from.
 * @param buffer BME680 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_read_from(bme680_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "bme680_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read halfword from register address transaction.
 * 
 * @param device BME680 device descriptor.
 * @param reg_addr BME680 register address to read from.
 * @param word BME680 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_read_word_from(bme680_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bme680_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read byte from register address transaction.
 * 
 * @param device BME680 device descriptor.
 * @param reg_addr BME680 register address to read from.
 * @param byte BME680 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_read_byte_from(bme680_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bme680_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write byte to register address transaction.
 * 
 * @param device BME680 device descriptor.
 * @param reg_addr BME680 register address to write to.
 * @param byte BME680 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_write_byte_to(bme680_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bme680_i2c_write_byte_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Calculates dew-point temperature from air temperature and relative humidity.
 *
 * @param[in] temperature Air temperature in degrees Celsius (valid range: -40 to 80°C).
 * @param[in] humidity Relative humidity in percent (valid range: 0 to 100%).
 * @param[out] dewpoint Calculated dew-point temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if parameters are out of range.
 */
static inline esp_err_t bme680_calculate_dewpoint(const float temperature, const float humidity, float *dewpoint) {
    /* validate parameters */
    ESP_ARG_CHECK( dewpoint );
    
    if(temperature > 80.0f || temperature < -40.0f) {
        ESP_LOGE(TAG, "Temperature %.2f°C out of valid range (-40 to 80°C)", temperature);
        return ESP_ERR_INVALID_ARG;
    }
    
    if(humidity > 100.0f || humidity < 0.0f) {
        ESP_LOGE(TAG, "Humidity %.2f%% out of valid range (0 to 100%%)", humidity);
        return ESP_ERR_INVALID_ARG;
    }
    
    /* calculate dew-point temperature using Magnus formula */
    float H = (log10f(humidity) - 2.0f) / 0.4343f + (17.62f * temperature) / (243.12f + temperature);
    *dewpoint = 243.12f * H / (17.62f - H);
    
    return ESP_OK;
}

/**
 * @brief Temperature compensation algorithm is taken from datasheet.  See datasheet for details.
 *
 * @param[in] device BME680 device descriptor.
 * @param[in] adc_temperature raw adc temperature.
 * @return float Temperature in degrees Celsius.
 */
static inline float bme680_compensate_temperature(bme680_device_t *const device, const uint32_t adc_temperature) {
    /* calculate var1 data */
    float var1 = ((((float)adc_temperature / 16384.0f) - ((float)device->cal_factors->par_T1 / 1024.0f)) * ((float)device->cal_factors->par_T2));

    /* calculate var2 data */
    float var2 =
        (((((float)adc_temperature / 131072.0f) - ((float)device->cal_factors->par_T1 / 8192.0f)) *
          (((float)adc_temperature / 131072.0f) - ((float)device->cal_factors->par_T1 / 8192.0f))) * ((float)device->cal_factors->par_T3 * 16.0f));

    /* t_fine value*/
    device->cal_factors->temperature_fine = (var1 + var2);

    /* compensated temperature data*/
    return ((device->cal_factors->temperature_fine) / 5120.0f);
}

/**
 * @brief Humidity compensation algorithm is taken from datasheet.  See datasheet for details.
 * 
 * @param device BME680 device descriptor.
 * @param adc_humidity Raw ADC humidity.
 * @return float Humidity in percentage.
 */
static inline float bme680_compensate_humidity(bme680_device_t *const device, const uint16_t adc_humidity) {
    /* compensated temperature data*/
    float temp_comp = ((device->cal_factors->temperature_fine) / 5120.0f);
    float var1 = (float)((float)adc_humidity) -
           (((float)device->cal_factors->par_H1 * 16.0f) + (((float)device->cal_factors->par_H3 / 2.0f) * temp_comp));
    float var2 = var1 *
           ((float)(((float)device->cal_factors->par_H2 / 262144.0f) *
                    (1.0f + (((float)device->cal_factors->par_H4 / 16384.0f) * temp_comp) +
                     (((float)device->cal_factors->par_H5 / 1048576.0f) * temp_comp * temp_comp))));
    float var3 = (float)device->cal_factors->par_H6 / 16384.0f;
    float var4 = (float)device->cal_factors->par_H7 / 2097152.0f;
    float calc_hum = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);
    if (calc_hum > 100.0f) {
        calc_hum = 100.0f;
    } else if (calc_hum < 0.0f) {
        calc_hum = 0.0f;
    }

    return calc_hum;
}

/**
 * @brief Pressure compensation algorithm is taken from datasheet.  see datasheet for details.
 *
 * @param[in] device BME680 device descriptor.
 * @param[in] adc_pressure raw adc pressure.
 * @return float Pressure in Pascal.  Divide by 100 for Hecto-Pascals.
 */
static inline float bme680_compensate_pressure(bme680_device_t *const device, const uint32_t adc_pressure) {
    float var1 = (((float)device->cal_factors->temperature_fine / 2.0f) - 64000.0f);
    float var2 = var1 * var1 * (((float)device->cal_factors->par_P6) / (131072.0f));
    var2 = var2 + (var1 * ((float)device->cal_factors->par_P5) * 2.0f);
    var2 = (var2 / 4.0f) + (((float)device->cal_factors->par_P4) * 65536.0f);
    var1 = (((((float)device->cal_factors->par_P3 * var1 * var1) / 16384.0f) + ((float)device->cal_factors->par_P2 * var1)) / 524288.0f);
    var1 = ((1.0f + (var1 / 32768.0f)) * ((float)device->cal_factors->par_P1));
    float calc_pres = (1048576.0f - ((float)adc_pressure));

    /* Avoid exception caused by division by zero */
    if ((int)var1 != 0) {
        calc_pres = (((calc_pres - (var2 / 4096.0f)) * 6250.0f) / var1);
        var1 = (((float)device->cal_factors->par_P9) * calc_pres * calc_pres) / 2147483648.0f;
        var2 = calc_pres * (((float)device->cal_factors->par_P8) / 32768.0f);
        float var3 = ((calc_pres / 256.0f) * (calc_pres / 256.0f) * (calc_pres / 256.0f) * (device->cal_factors->par_P10 / 131072.0f));
        calc_pres = (calc_pres + (var1 + var2 + var3 + ((float)device->cal_factors->par_P7 * 128.0f)) / 16.0f);
    } else {
        calc_pres = 0;
    }

    return calc_pres;
}

static inline float bme680_compensate_gas_resistance(bme680_device_t *const device, uint16_t adc_gas_res, uint8_t gas_range) {
    const float lookup_k1_range[16] = {
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 0.99f, 1.0f, 0.992f, 1.0f, 1.0f, 0.998f, 0.995f, 1.0f, 0.99f, 1.0f, 1.0f
    };
    const float lookup_k2_range[16] = {
        8000000.0f, 4000000.0f, 2000000.0f, 1000000.0f, 499500.4995f, 248262.1648f, 125000.0f, 63004.03226f, 
        31281.28128f, 15625.0f, 7812.5f, 3906.25f, 1953.125f, 976.5625f, 488.28125f, 244.140625f
    };
    float var1 = (1340.0f + 5.0f * device->cal_factors->range_switching_error) * lookup_k1_range[gas_range];
    return var1 * lookup_k2_range[gas_range] / (adc_gas_res - 512.0f + var1);
}

/**
 * @brief Calculates the heater resistance setting for a given temperature.
 * 
 * @param device BME680 device descriptor.
 * @param temperature Desired heater temperature in degrees Celsius.
 * @return uint8_t Heater resistance setting.
 */
static inline uint8_t bme680_compensate_heater_resistance(bme680_device_t *const device, uint16_t temperature) {
    /* Cap temperature */
    if (temperature > 400) {
        temperature = 400;
    }

    float var1 = (((float)device->cal_factors->par_G1 / (16.0f)) + 49.0f);
    float var2 = ((((float)device->cal_factors->par_G2 / (32768.0f)) * (0.0005f)) + 0.00235f);
    float var3 = ((float)device->cal_factors->par_G3 / (1024.0f));
    float var4 = (var1 * (1.0f + (var2 * (float)temperature)));
    float var5 = (var4 + (var3 * (float)device->ambient_temperature));
    uint8_t res_heat =
        (uint8_t)(3.4f *
                  ((var5 * (4 / (4 + (float)device->cal_factors->res_heat_range)) *
                    (1 / (1 + ((float)device->cal_factors->res_heat_val * 0.002f)))) -
                   25.0f));

    return res_heat;
}

/**
 * @brief Calculates the gas wait time.
 * 
 * @param duration 
 * @return uint8_t Gas wait time duration.
 */
static inline uint8_t bme680_compute_gas_wait(const uint16_t duration) {
    uint8_t factor = 0;
    uint8_t durval;
    uint16_t dura = duration;

    if (dura >= 0xfc0) {
        durval = 0xff; /* Max duration*/
    } else {
        while (dura > 0x3F) {
            dura = dura / 4;
            factor += 1;
        }
        durval = (uint8_t)(dura + (factor * 64));
    }

    return durval;
}

static inline uint8_t bme680_compute_heater_shared_duration(const uint16_t duration) {
    uint8_t factor = 0;
    uint8_t heatdurval;
    uint16_t dura = duration;

    if (dura >= 0x783) {
        heatdurval = 0xff; /* Max duration */
    } else {
        /* Step size of 0.477ms */
        dura = (uint16_t)(((uint32_t)dura * 1000) / 477);
        while (dura > 0x3F) {
            dura = dura >> 2;
            factor += 1;
        }

        heatdurval = (uint8_t)(dura + (factor * 64));
    }

    return heatdurval;
}

/**
 * @brief Gets an estimated measurement duration in milliseconds.
 * 
 * @param device BME680 device descriptor.
 * @return uint32_t Estimated measurement duration in milliseconds.
 */
static inline uint32_t bme680_get_measurement_duration(bme680_device_t *const device) {
    const uint8_t os_to_meas_cycles[6] = { 0, 1, 2, 4, 8, 16 };
    uint32_t meas_dur = 0; /* Calculate in us */
    uint32_t meas_cycles;

    /* validate arguments */
    if((device) != NULL) {
        meas_cycles = os_to_meas_cycles[device->config.temperature_oversampling];
        meas_cycles += os_to_meas_cycles[device->config.pressure_oversampling];
        meas_cycles += os_to_meas_cycles[device->config.humidity_oversampling];

        /* TPH measurement duration */
        meas_dur = meas_cycles * (uint32_t)(1963);
        meas_dur += (uint32_t)(477 * 4); /* TPH switching duration */
        meas_dur += (uint32_t)(477 * 5); /* Gas measurement duration */

        if (device->config.power_mode != BME680_POWER_MODE_PARALLEL) {
            meas_dur += (uint32_t)(1000); /* Wake up duration of 1ms */
        }
    }

    return meas_dur;
}

/**
 * @brief BME680 I2C HAL read the calibration factors.  See datasheet for details.
 *
 * @param[in] device BME680 device descriptor..
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_cal_factors(bme680_device_t *const device) {
    bit16_uint8_buffer_t rx;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* bme680 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0xe9, &device->cal_factors->par_T1) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x8a, (uint16_t *)&device->cal_factors->par_T2) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x8c, (uint8_t *)&device->cal_factors->par_T3) );
    /* bme680 attempt to request H1-H7 calibration values from device */
    ESP_ERROR_CHECK( bme680_i2c_read_from(device, 0xe2, rx, BIT16_UINT8_BUFFER_SIZE) );
    device->cal_factors->par_H1 = (uint16_t)(((uint16_t)rx[1] << 4) | (rx[0] & 0x0F));
    ESP_ERROR_CHECK( bme680_i2c_read_from(device, 0xe1, rx, BIT16_UINT8_BUFFER_SIZE) );
    device->cal_factors->par_H2 = (uint16_t)(((uint16_t)rx[0] << 4) | (rx[1] >> 4));
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xe4, (uint8_t *)&device->cal_factors->par_H3) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xe5, (uint8_t *)&device->cal_factors->par_H4) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xe6, (uint8_t *)&device->cal_factors->par_H5) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xe7, &device->cal_factors->par_H6) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xe8, (uint8_t *)&device->cal_factors->par_H7) );
    /* bme680 attempt to request P1-P10 calibration values from device */
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x8e, &device->cal_factors->par_P1) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x90, (uint16_t *)&device->cal_factors->par_P2) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x92, (uint8_t *)&device->cal_factors->par_P3) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x94, (uint16_t *)&device->cal_factors->par_P4) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x96, (uint16_t *)&device->cal_factors->par_P5) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x99, (uint8_t *)&device->cal_factors->par_P6) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x98, (uint8_t *)&device->cal_factors->par_P7) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x9c, (uint16_t *)&device->cal_factors->par_P8) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0x9e, (uint16_t *)&device->cal_factors->par_P9) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xa0, &device->cal_factors->par_P10) );
    /* bme680 attempt to request G1-G3 calibration values from device */
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xed, (uint8_t *)&device->cal_factors->par_G1) );
    ESP_ERROR_CHECK( bme680_i2c_read_word_from(device, 0xeb, (uint16_t *)&device->cal_factors->par_G2) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0xee, (uint8_t *)&device->cal_factors->par_G3) );
    /* bme680 attempt to request gas range and switching error values from device */
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x02, &device->cal_factors->res_heat_range) );
    device->cal_factors->res_heat_range = (device->cal_factors->res_heat_range & 0x30) / 16;
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x00, (uint8_t *)&device->cal_factors->res_heat_val) );
    ESP_ERROR_CHECK( bme680_i2c_read_byte_from(device, 0x04, (uint8_t *)&device->cal_factors->range_switching_error) );
    device->cal_factors->range_switching_error = (device->cal_factors->range_switching_error & 0xf0) / 16;

    /*
    ESP_LOGD(TAG, "Calibration data received:");
    //
    ESP_LOGD(TAG, "par_T1=%u", handle->dev_cal_factors->par_T1);
    ESP_LOGD(TAG, "par_T2=%d", handle->dev_cal_factors->par_T2);
    ESP_LOGD(TAG, "par_T3=%d", handle->dev_cal_factors->par_T3);
    //
    ESP_LOGD(TAG, "par_P1=%u", handle->dev_cal_factors->par_P1);
    ESP_LOGD(TAG, "par_P2=%d", handle->dev_cal_factors->par_P2);
    ESP_LOGD(TAG, "par_P3=%d", handle->dev_cal_factors->par_P3);
    ESP_LOGD(TAG, "par_P4=%d", handle->dev_cal_factors->par_P4);
    ESP_LOGD(TAG, "par_P5=%d", handle->dev_cal_factors->par_P5);
    ESP_LOGD(TAG, "par_P6=%d", handle->dev_cal_factors->par_P6);
    ESP_LOGD(TAG, "par_P7=%d", handle->dev_cal_factors->par_P7);
    ESP_LOGD(TAG, "par_P8=%d", handle->dev_cal_factors->par_P8);
    ESP_LOGD(TAG, "par_P9=%d", handle->dev_cal_factors->par_P9);
    ESP_LOGD(TAG, "par_P10=%d", handle->dev_cal_factors->par_P10);
    */
    
    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL setup and configure gas and heater profile registers.
 * 
 * @param device BME680 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_setup_heater_profiles(bme680_device_t *const device, bme680_heater_setpoints_t *const heater_setpoint) {
    uint8_t i;
    uint8_t shared_dur;
    uint8_t write_len = 0;
    uint8_t rh_reg_addr[BME680_HEATER_PROFILE_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t rh_reg_data[BME680_HEATER_PROFILE_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t gw_reg_addr[BME680_HEATER_PROFILE_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint8_t gw_reg_data[BME680_HEATER_PROFILE_SIZE] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    switch(device->config.power_mode) {
        case BME680_POWER_MODE_FORCED:
            //rh_reg_addr[0] = BME680_REG_RES_HEAT;
            //rh_reg_data[0] = bme680_compensate_heater_resistance(handle, handle->dev_config.heater_temperature);
            //gw_reg_addr[0] = BME680_REG_GAS_WAIT;
            //gw_reg_data[0] = bme680_compute_gas_wait(handle->dev_config.heater_duration);
            //(*heater_setpoint) = handle->dev_config.heater_profile_size - 1;
            //write_len = handle->dev_config.heater_profile_size;
            if ((device->config.heater_profile_size == 0) || (device->config.heater_profile_size > 10)) {
                ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "heater duration or temperature profile are empty and cannot be larger than 10, setup heater setpoints failed");
            }

            if(device->config.heater_profile_size == 1) {
                rh_reg_addr[0] = BME680_REG_RES_HEAT;
                rh_reg_data[0] = bme680_compensate_heater_resistance(device, device->config.heater_temperature);
                gw_reg_addr[0] = BME680_REG_GAS_WAIT;
                gw_reg_data[0] = bme680_compute_gas_wait(device->config.heater_duration);
            } else {
                for (i = 0; i < device->config.heater_profile_size; i++) {
                    rh_reg_addr[i] = BME680_REG_RES_HEAT + i;
                    rh_reg_data[i] = bme680_compensate_heater_resistance(device, device->config.heater_temperature_profile[i]);
                    gw_reg_addr[i] = BME680_REG_GAS_WAIT + i;
                    gw_reg_data[i] = bme680_compute_gas_wait(device->config.heater_duration_profile[i]);
                }
            }

            (*heater_setpoint) = device->config.heater_profile_size - 1;
            write_len = device->config.heater_profile_size;
            break;
        case BME680_POWER_MODE_SEQUENTIAL:
            if ((device->config.heater_profile_size == 0) || (device->config.heater_profile_size > 10)) {
                ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "heater duration or temperature profile are empty and cannot be larger than 10, setup heater setpoints failed");
            }

            for (i = 0; i < device->config.heater_profile_size; i++) {
                rh_reg_addr[i] = BME680_REG_RES_HEAT + i;
                rh_reg_data[i] = bme680_compensate_heater_resistance(device, device->config.heater_temperature_profile[i]);
                gw_reg_addr[i] = BME680_REG_GAS_WAIT + i;
                gw_reg_data[i] = bme680_compute_gas_wait(device->config.heater_duration_profile[i]);
            }

            (*heater_setpoint) = device->config.heater_profile_size - 1;
            write_len = device->config.heater_profile_size;

            break;
        case BME680_POWER_MODE_PARALLEL:
            if ((device->config.heater_profile_size == 0) || (device->config.heater_profile_size > 10)) {
                ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "heater duration or temperature profile are empty and cannot be larger than 10, setup heater setpoints failed");
            }

            if(device->config.heater_shared_duration == 0) {
                ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "heater shared duration must be greater than 0, setup heater setpoints failed");
            }

            for (i = 0; i < device->config.heater_profile_size; i++) {
                rh_reg_addr[i] = BME680_REG_RES_HEAT + i;
                rh_reg_data[i] = bme680_compensate_heater_resistance(device, device->config.heater_temperature_profile[i]);
                gw_reg_addr[i] = BME680_REG_GAS_WAIT + i;
                gw_reg_data[i] = (uint8_t)device->config.heater_duration_profile[i];
            }

            (*heater_setpoint) = device->config.heater_profile_size - 1;
            write_len = device->config.heater_profile_size;
            shared_dur = bme680_compute_heater_shared_duration(device->config.heater_shared_duration);
            ESP_RETURN_ON_ERROR(bme680_i2c_write_byte_to(device, BME680_REG_SHD_HEATR_DUR, shared_dur), TAG, "unable to write shared heater duration, setup heater setpoints failed");
            break;
        case BME680_POWER_MODE_SLEEP:
            return ESP_ERR_INVALID_ARG;
    };

    /* attempt to write resistance heater and gas wait profile */
    for (i = 0; i < write_len; i++) {
        ESP_RETURN_ON_ERROR(bme680_i2c_write_byte_to(device, rh_reg_addr[i], rh_reg_data[i]), TAG, "unable to write resistance heater profile, setup heater setpoints failed");
        ESP_RETURN_ON_ERROR(bme680_i2c_write_byte_to(device, gw_reg_addr[i], gw_reg_data[i]), TAG, "unable to write gas wait profile, setup heater setpoints failed");

        ESP_LOGI(TAG, "bme680_i2c_setup_heater_profiles: rh_reg_data %d | gw_reg_data %d", rh_reg_data[i], gw_reg_data[i]);
    }

    return ESP_OK;
}

/**
 * @brief IAQ calculations following Dr. Julie Riggs, The IAQ Rating Index, www.iaquk.org.uk.
 * 
 * - Weighting: Temperature, Humidity each one tenth of the rating ==> 6.5 points max each 
 * giving gas resistance readings 8 tenths of the rating ==> 52 points max
 * 
 * @param handle BME680 device handle.
 * @param data BME680 data structure, air temperature, dew-point temperature, and 
 * relative humidity parameters must be initialized as a minimum.
 * @return esp_err_t ESP_OK on success.
 */
static inline void bme680_compute_iaq(bme680_data_t *const data) {
    float adjusted_temp = data->air_temperature + BME680_AQI_TEMP_CORR;

    // August-Roche-Magnus approximation (http://bmcnoldy.rsmas.miami.edu/Humidity.html)
    float adjusted_humi = 100 * (exp((17.625 * data->dewpoint_temperature) / (243.04 + data->dewpoint_temperature)) / exp((17.625 * adjusted_temp) / (243.04 + adjusted_temp)));

    uint32_t gas = data->gas_resistance;

    // Calculate humidity score
    if (adjusted_humi >= 40 && adjusted_humi <= 60) data->humidity_score = 6.5;  //ideal condition, full points
    else if ((adjusted_humi >= 30 && adjusted_humi < 40) || (adjusted_humi > 60 && adjusted_humi <= 70)) data->humidity_score = 5.2;  //20% less
    else if ((adjusted_humi >= 20 && adjusted_humi < 30) || (adjusted_humi > 70 && adjusted_humi <= 80)) data->humidity_score = 3.9;  //40% less
    else if ((adjusted_humi >= 10 && adjusted_humi < 20) || (adjusted_humi > 80 && adjusted_humi <= 90)) data->humidity_score = 2.6;  //60% less
    else if (adjusted_humi < 10 && adjusted_humi > 90) data->humidity_score = 1.3;  //80% less

    // Calculate temperature score
    int compare_temp = adjusted_temp;     // round to full degree
    if (compare_temp >= 18 && compare_temp <= 22) data->temperature_score = 6.5;  //ideal condition, full points
    else if (compare_temp == 17 || compare_temp == 23) data->temperature_score = 5.2;  //20% less
    else if (compare_temp == 16 || compare_temp == 24) data->temperature_score = 3.9;  //40% less
    else if (compare_temp == 15 || compare_temp == 25) data->temperature_score = 2.6;  //60% less
    else if (compare_temp == 14 || compare_temp == 26) data->temperature_score = 1.3;  //80% less
    else if (compare_temp  < 14 || compare_temp  > 26) data->temperature_score = 0;    //100% less

    // Calculate gas score on resistance values - best practices
    if (gas >= 430000) data->gas_score = 52;  //ideal condition, full points
    else if (gas >= 210000 && gas < 430000) data->gas_score = 43;
    else if (gas >= 100000 && gas < 210000) data->gas_score = 35;
    else if (gas >=  55000 && gas < 100000) data->gas_score = 26;
    else if (gas >=  27000 && gas <  55000) data->gas_score = 18;
    else if (gas >=  13500 && gas >   9000) data->gas_score = 9;
    else if (gas < 9000) data->gas_score = 0;

    // calculate iaq score
    data->iaq_score = data->humidity_score + data->temperature_score + data->gas_score;
}

/**
 * @brief BME680 I2C HAL read chip identification register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg BME680 chip identifier.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_chip_id_register(bme680_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_ID, reg), TAG, "read chip identifier register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read variant identification register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg BME680 variant identifier.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_variant_id_register(bme680_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_VARIANT_ID, reg), TAG, "read variant identifier register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read status register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Status 0 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_status0_register(bme680_device_t *const device, bme680_status0_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_STATUS0, &reg->reg), TAG, "read status register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read gas resistance LSB register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Gas LSB register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_gas_lsb_register(bme680_device_t *const device, bme680_gas_lsb_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_GAS_R_LSB, &reg->reg), TAG, "read gas lsb register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write gas resistance LSB register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] reg Gas LSB register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_set_gas_lsb_register(bme680_device_t *const device, const bme680_gas_lsb_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_GAS_R_LSB, reg.reg), TAG, "write control measurement register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read control measurement register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_control_measurement_register(bme680_device_t *const device, bme680_control_measurement_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_CTRL_MEAS, &reg->reg), TAG, "read control measurement register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write control measurement register. 
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] reg Control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_set_control_measurement_register(bme680_device_t *const device, const bme680_control_measurement_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_CTRL_MEAS, reg.reg), TAG, "write control measurement register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read control humidity register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Control humidity register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_control_humidity_register(bme680_device_t *const device, bme680_control_humidity_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_CTRL_HUMI, &reg->reg), TAG, "read control humidity register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write control humidity register. 
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] reg Control humidity register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_set_control_humidity_register(bme680_device_t *const device, const bme680_control_humidity_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    bme680_control_humidity_register_t ctrl_hum = { .reg = reg.reg };
    ctrl_hum.bits.reserved1 = 0;
    ctrl_hum.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_CTRL_HUMI, ctrl_hum.reg), TAG, "write control humidity register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read control gas 0 register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Control gas 0 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_control_gas0_register(bme680_device_t *const device, bme680_control_gas0_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_CTRL_GAS0, &reg->reg), TAG, "read control gas 0 register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write control gas 0 register. 
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] reg Control gas 0 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_set_control_gas0_register(bme680_device_t *const device, const bme680_control_gas0_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    bme680_control_gas0_register_t gas0 = { .reg = reg.reg };
    gas0.bits.reserved1 = 0;
    gas0.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_CTRL_GAS0, gas0.reg), TAG, "write control gas 0 register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read control gas 1 register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Control gas 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_control_gas1_register(bme680_device_t *const device, bme680_control_gas1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_CTRL_GAS1, &reg->reg), TAG, "read control gas 1 register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write control gas 1 register. 
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] reg Control gas 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_set_control_gas1_register(bme680_device_t *const device, const bme680_control_gas1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    bme680_control_gas1_register_t gas1 = { .reg = reg.reg };
    gas1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_CTRL_GAS1, gas1.reg), TAG, "write control gas 1 register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL read configuration register.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_get_configuration_register(bme680_device_t *const device, bme680_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_read_byte_from(device, BME680_REG_CONFIG, &reg->reg), TAG, "read configuration register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL write configuration register. 
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_set_configuration_register(bme680_device_t *const device, const bme680_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    bme680_config_register_t config = { .reg = reg.reg };

    /* set reserved to 0 */
    config.bits.reserved1 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL setup and configure gas and heater registers.
 * 
 * @param device BME680 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_setup_heater(bme680_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    bme680_heater_setpoints_t      heater_setpoint;
    bme680_control_gas0_register_t ctrl_gas0_reg;
    bme680_control_gas1_register_t ctrl_gas1_reg;

    ESP_RETURN_ON_ERROR(bme680_i2c_setup_heater_profiles(device, &heater_setpoint), TAG, "setup heater profiles for setup heater failed");

    /* attempt to read control gas 0 register */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_control_gas0_register(device, &ctrl_gas0_reg), TAG, "read control gas 0 register for setup heater failed");

    /* attempt to read control gas 1 register */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_control_gas1_register(device, &ctrl_gas1_reg), TAG, "read control gas 1 register for setup heater failed");

    /* initialize control gas 0 and 1 from configuration params */
    if(device->config.gas_enabled) {
        ctrl_gas0_reg.bits.heater_disabled        = false;
        ctrl_gas1_reg.bits.gas_conversion_enabled = true;
        ctrl_gas1_reg.bits.heater_setpoint        = heater_setpoint;
    } else {
        ctrl_gas1_reg.bits.gas_conversion_enabled = false;
        ctrl_gas0_reg.bits.heater_disabled        = true;
    }

    /* attempt to write control gas 0 register */
    ESP_RETURN_ON_ERROR(bme680_i2c_set_control_gas0_register(device, ctrl_gas0_reg), TAG, "write control gas 0 register for setup heater failed");

    /* attempt to write control gas 1 register */
    ESP_RETURN_ON_ERROR(bme680_i2c_set_control_gas1_register(device, ctrl_gas1_reg), TAG, "write control gas 1 register for setup heater failed");

    return ESP_OK;
}

/**
 * @brief BME680 I2C HAL setup and configure context and registers.
 * 
 * @param device BME680 device descriptor..
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bme680_i2c_setup(bme680_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_control_humidity_register_t    ctrl_humi_reg;
    bme680_config_register_t              config_reg;
    bme680_control_gas1_register_t        gas1_reg;

    /* set ambient temperature of sensor to default value (25 degree C) */
    device->ambient_temperature = 25;

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_cal_factors(device), TAG, "read calibration factors for setup failed" );

    /* attempt to read variant identifier register from device */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_variant_id_register(device, &device->variant_id), TAG, "read variant identifier register for setup failed" );

    /* attempt to read control humidity register */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_control_humidity_register(device, &ctrl_humi_reg), TAG, "read control humidity register for setup failed");

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for setup failed");

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_configuration_register(device, &config_reg), TAG, "read configuration register for setup failed");

    /* attempt to read gas 1 register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_gas1_register(device, &gas1_reg), TAG, "read control gas 1 register for setup failed" );

    /* initialize configuration register from configuration params */
    config_reg.bits.iir_filter                  = device->config.iir_filter;
    config_reg.bits.spi_enabled                 = false;

    /* set standby time */
    gas1_reg.bits.standby_period_msb            = device->config.standby_time>>3 & 1;
    config_reg.bits.standby_period_lsb          = device->config.standby_time & 0b111;

    /* initialize control measurement register from configuration params */
    ctrl_meas_reg.bits.power_mode               = device->config.power_mode;
    ctrl_meas_reg.bits.temperature_oversampling = device->config.temperature_oversampling;
    ctrl_meas_reg.bits.pressure_oversampling    = device->config.pressure_oversampling;

    /* initialize control humidity register from configuration params */
    ctrl_humi_reg.bits.humidity_oversampling    = device->config.humidity_oversampling;
    ctrl_humi_reg.bits.spi_irq_enabled          = false;

    /* attempt to write control humidity register */
    ESP_RETURN_ON_ERROR(bme680_i2c_set_control_humidity_register(device, ctrl_humi_reg), TAG, "write control humidity register for setup failed");

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR(bme680_i2c_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for setup failed");

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR(bme680_i2c_set_configuration_register(device, config_reg), TAG, "write configuration register for setup failed");

    /* attempt to write gas 1 register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_control_gas1_register(device, gas1_reg), TAG, "write control gas 1 register for setup failed" );

    return ESP_OK;
}

esp_err_t bme680_init(i2c_master_bus_handle_t master_handle, const bme680_config_t *bme680_config, bme680_handle_t *bme680_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && bme680_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BME680_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, bme680_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bme680 device handle initialization failed", bme680_config->i2c_address);

    /* validate memory availability for handle */
    bme680_device_t* device = (bme680_device_t*)calloc(1, sizeof(bme680_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 bme680 device for init");

    /* validate memory availability for handle calibration factors */
    device->cal_factors = (bme680_cal_factors_t*)calloc(1, sizeof(bme680_cal_factors_t));
    ESP_GOTO_ON_FALSE(device->cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for i2c bme680 device calibration factors for init");

    /* copy configuration */
    device->config = *bme680_config;

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
    vTaskDelay(pdMS_TO_TICKS(BME680_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR(bme680_i2c_get_chip_id_register(device, &device->chip_id), err_handle, TAG, "read chip identifier for init failed");
    if(device->chip_id != BME680_CHIP_ID) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", device->chip_id);
    }

    /* attempt to reset the device and initialize registers */
    ESP_GOTO_ON_ERROR(bme680_reset((bme680_handle_t)device), err_handle, TAG, "soft-reset and initialize registers for init failed");

    /* copy configuration */
    *bme680_handle = (bme680_handle_t)device;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BME680_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

char *bme680_air_quality_to_string(float iaq_score) {
    if      (iaq_score < 25)                     return "Hazardous";
    else if (iaq_score >= 25 && iaq_score <= 38) return "Unhealthy";
    else if (iaq_score  > 38 && iaq_score <= 51) return "Moderate";
    else if (iaq_score  > 51 && iaq_score <= 60) return "Good";
    else if (iaq_score  > 60 && iaq_score <= 65) return "Excellent";
    else                                         return "Unknown";
}

esp_err_t bme680_get_adc_signals(bme680_handle_t handle, bme680_adc_data_t *const data) {
    esp_err_t       ret             = ESP_OK;
    bool            data_is_ready   = false;
    uint8_t         gas_index       = 0;
    uint16_t        adc_gas_r;
    uint32_t        adc_press;
    uint16_t        adc_humi;
    uint32_t        adc_temp;
    bit104_uint8_buffer_t rx;
    bme680_status0_register_t status0_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && data );

    /* trigger measurement when in forced mode */
    if(device->config.power_mode == BME680_POWER_MODE_FORCED) {
        bme680_set_power_mode(handle, device->config.power_mode);
    }

    /* set start time for timeout monitoring */
    const uint64_t start_time = esp_timer_get_time();

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( bme680_i2c_get_status0_register(device, &status0_reg), err, TAG, "status 0 register for get adc signals failed." );

        data_is_ready = status0_reg.bits.new_data;
        gas_index = status0_reg.bits.gas_measurement_index;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(BME680_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, BME680_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // need to read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( bme680_i2c_read_from(device, BME680_REG_PRESS, rx, BIT104_UINT8_BUFFER_SIZE), err, TAG, "read adc data failed" );

    /* instantiate gas lsb register */
    bme680_gas_lsb_register_t gas_lsb_reg = { .reg = rx[12] };

    /* concat parameters */
    adc_press = ((uint32_t)rx[0] << 12) | ((uint32_t)rx[1] << 4) | ((uint32_t)rx[2] >> 4);
    adc_temp  = ((uint32_t)rx[3] << 12) | ((uint32_t)rx[4] << 4) | ((uint32_t)rx[5] >> 4);
    adc_humi  = ((uint16_t)rx[6] << 8) | (uint16_t)rx[7];
    adc_gas_r = ((uint16_t)rx[11] << 2) | ((uint16_t)rx[12] >> 6);

    ESP_LOGD(TAG, "ADC humidity:    %u", adc_humi);
    ESP_LOGD(TAG, "ADC temperature: %lu", adc_temp);
    ESP_LOGD(TAG, "ADC pressure:    %lu", adc_press);
    ESP_LOGD(TAG, "ADC gas:         %u", adc_gas_r);
    ESP_LOGD(TAG, "ADC gas index:  %u", gas_index);

    /* initialize data structure */
    data->temperature    = adc_temp;
    data->humidity       = adc_humi;
    data->pressure       = adc_press;
    data->gas            = adc_gas_r;
    data->gas_index      = gas_index;
    data->gas_range      = gas_lsb_reg.bits.gas_range;
    data->heater_stable  = gas_lsb_reg.bits.heater_stable;
    data->gas_valid      = gas_lsb_reg.bits.gas_valid;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t bme680_get_adc_signals_by_heater_profile(bme680_handle_t handle, uint8_t profile_index, bme680_adc_data_t *const data) {
    esp_err_t       ret             = ESP_OK;
    bool            data_is_ready   = false;
    uint8_t         gas_index       = 0;
    uint16_t        adc_gas_r;
    uint32_t        adc_press;
    uint16_t        adc_humi;
    uint32_t        adc_temp;
    bit104_uint8_buffer_t rx;
    bme680_status0_register_t status0_reg;
    bme680_control_gas1_register_t ctrl_gas1_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && data );

    if ((device->config.heater_profile_size == 0) || (device->config.heater_profile_size > 10)) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "heater duration or temperature profile are empty and cannot be larger than 10, get adc signals by heater profile failed");
    }

    /* trigger measurement when in forced mode */
    if(device->config.power_mode == BME680_POWER_MODE_FORCED) {
        bme680_set_power_mode(handle, device->config.power_mode);
    }

    /* attempt to read control gas 1 register */
    ESP_RETURN_ON_ERROR(bme680_i2c_get_control_gas1_register(device, &ctrl_gas1_reg), TAG, "read control gas 1 register for setup heater failed");

    ctrl_gas1_reg.bits.heater_setpoint = (bme680_heater_setpoints_t)profile_index;

    /* attempt to write control gas 1 register */
    ESP_RETURN_ON_ERROR(bme680_i2c_set_control_gas1_register(device, ctrl_gas1_reg), TAG, "write control gas 1 register for setup heater failed");

    /* set start time for timeout monitoring */
    const uint64_t start_time = esp_timer_get_time();

    /* attempt to poll until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        //ESP_GOTO_ON_ERROR( bme680_get_data_status(handle, &data_is_ready), err, TAG, "data ready for get adc signals failed." );
        ESP_GOTO_ON_ERROR( bme680_i2c_get_status0_register(device, &status0_reg), err, TAG, "status 0 register for get adc signals failed." );

        data_is_ready = status0_reg.bits.new_data;
        gas_index = status0_reg.bits.gas_measurement_index;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(BME680_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, BME680_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // need to read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( bme680_i2c_read_from(device, BME680_REG_PRESS, rx, BIT104_UINT8_BUFFER_SIZE), err, TAG, "read adc data failed" );

    /* instantiate gas lsb register */
    bme680_gas_lsb_register_t gas_lsb_reg = { .reg = rx[12] };

    /* concat parameters */
    adc_press = ((uint32_t)rx[0] << 12) | ((uint32_t)rx[1] << 4) | ((uint32_t)rx[2] >> 4);
    adc_temp  = ((uint32_t)rx[3] << 12) | ((uint32_t)rx[4] << 4) | ((uint32_t)rx[5] >> 4);
    adc_humi  = ((uint16_t)rx[6] << 8) | (uint16_t)rx[7];
    adc_gas_r = ((uint16_t)rx[11] << 2) | ((uint16_t)rx[12] >> 6);

    ESP_LOGD(TAG, "ADC humidity:    %u", adc_humi);
    ESP_LOGD(TAG, "ADC temperature: %lu", adc_temp);
    ESP_LOGD(TAG, "ADC pressure:    %lu", adc_press);
    ESP_LOGD(TAG, "ADC gas:         %u", adc_gas_r);
    ESP_LOGD(TAG, "ADC gas index:   %u", gas_index);

    /* initialize data structure */
    data->temperature    = adc_temp;
    data->humidity       = adc_humi;
    data->pressure       = adc_press;
    data->gas            = adc_gas_r;
    data->gas_index      = gas_index;
    data->gas_range      = gas_lsb_reg.bits.gas_range;
    data->heater_stable  = gas_lsb_reg.bits.heater_stable;
    data->gas_valid      = gas_lsb_reg.bits.gas_valid;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t bme680_get_data(bme680_handle_t handle, bme680_data_t *const data) {
    bme680_adc_data_t adc_data;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && data );

    /* attempt to read adc signals */
    ESP_RETURN_ON_ERROR( bme680_get_adc_signals(handle, &adc_data), TAG, "read adc signals failed" );

    /* initialize data structure */
    data->air_temperature       = bme680_compensate_temperature(device, adc_data.temperature);
    data->relative_humidity     = bme680_compensate_humidity(device, adc_data.humidity);
    bme680_calculate_dewpoint(data->air_temperature, data->relative_humidity, &data->dewpoint_temperature);
    data->barometric_pressure   = bme680_compensate_pressure(device, adc_data.pressure);
    data->gas_resistance        = bme680_compensate_gas_resistance(device, adc_data.gas, adc_data.gas_range);
    data->gas_range             = adc_data.gas_range;
    data->heater_stable         = adc_data.heater_stable;
    data->gas_valid             = adc_data.gas_valid;
    data->gas_index             = adc_data.gas_index;

    /* compute scores */
    bme680_compute_iaq(data);

    return ESP_OK;
}

esp_err_t bme680_get_data_by_heater_profile(bme680_handle_t handle, const uint8_t profile_index, bme680_data_t *const data) {
    bme680_adc_data_t adc_data;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && data );

    if ((device->config.heater_profile_size == 0) || (device->config.heater_profile_size > 10)) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "heater duration or temperature profile are empty and cannot be larger than 10, get data by heater profile failed");
    }

    /* attempt to read adc signals */
    ESP_RETURN_ON_ERROR( bme680_get_adc_signals_by_heater_profile(handle, profile_index, &adc_data), TAG, "read adc signals failed" );

    /* initialize data structure */
    data->air_temperature       = bme680_compensate_temperature(device, adc_data.temperature);
    data->relative_humidity     = bme680_compensate_humidity(device, adc_data.humidity);
    bme680_calculate_dewpoint(data->air_temperature, data->relative_humidity, &data->dewpoint_temperature);
    data->barometric_pressure   = bme680_compensate_pressure(device, adc_data.pressure);
    data->gas_resistance        = bme680_compensate_gas_resistance(device, adc_data.gas, adc_data.gas_range);
    data->gas_range             = adc_data.gas_range;
    data->heater_stable         = adc_data.heater_stable;
    data->gas_valid             = adc_data.gas_valid;
    data->gas_index             = adc_data.gas_index;

    /* compute scores */
    bme680_compute_iaq(data);

    return ESP_OK;
}

esp_err_t bme680_get_data_status(bme680_handle_t handle, bool *const ready) {
    bme680_status0_register_t   status0_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_status0_register(device, &status0_reg), TAG, "read status register (data ready state) failed" );

    /* set ready state */
    *ready = status0_reg.bits.new_data;
    
    return ESP_OK;
}

esp_err_t bme680_get_gas_measurement_index(bme680_handle_t handle, uint8_t *const index) {
    bme680_status0_register_t   status0_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_status0_register(device, &status0_reg), TAG, "read status register (gas measurement index) failed" );

    /* set gas measurement index */
    *index = status0_reg.bits.gas_measurement_index;

    return ESP_OK;
}

esp_err_t bme680_get_power_mode(bme680_handle_t handle, bme680_power_modes_t *const power_mode) {
    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get power mode failed" );

    /* set power mode */
    *power_mode = ctrl_meas_reg.bits.power_mode;

    return ESP_OK;
}

esp_err_t bme680_set_power_mode(bme680_handle_t handle, const bme680_power_modes_t power_mode) {
    bme680_control_measurement_register_t   ctrl_meas_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get power mode failed" );

    /* initialize control measurement register */
    ctrl_meas_reg.bits.power_mode = power_mode;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for set power mode failed" );

    return ESP_OK;
}

esp_err_t bme680_get_pressure_oversampling(bme680_handle_t handle, bme680_pressure_oversampling_t *const oversampling) {
    bme680_control_measurement_register_t   ctrl_meas_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get pressure oversampling failed" );

    /* set oversampling */
    *oversampling = ctrl_meas_reg.bits.pressure_oversampling;

    return ESP_OK;
}

esp_err_t bme680_set_pressure_oversampling(bme680_handle_t handle, const bme680_pressure_oversampling_t oversampling) {
    bme680_control_measurement_register_t   ctrl_meas_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get pressure oversampling failed" );

    /* initialize control measurement register */
    ctrl_meas_reg.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for set pressure oversampling failed" );

    return ESP_OK;
}

esp_err_t bme680_get_temperature_oversampling(bme680_handle_t handle, bme680_temperature_oversampling_t *const oversampling) {
    bme680_control_measurement_register_t   ctrl_meas_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get temperature oversampling failed" );

    /* set oversampling */
    *oversampling = ctrl_meas_reg.bits.temperature_oversampling;

    return ESP_OK;
}

esp_err_t bme680_set_temperature_oversampling(bme680_handle_t handle, const bme680_temperature_oversampling_t oversampling) {
    bme680_control_measurement_register_t   ctrl_meas_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get temperature oversampling failed" );

    /* initialize control measurement register */
    ctrl_meas_reg.bits.temperature_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for set temperature oversampling failed" );

    return ESP_OK;
}

esp_err_t bme680_get_humidity_oversampling(bme680_handle_t handle, bme680_humidity_oversampling_t *const oversampling) {
    bme680_control_humidity_register_t   ctrl_humi_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control humidity register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_humidity_register(device, &ctrl_humi_reg), TAG, "read control humidity register for get humidity oversampling failed" );

    /* set oversampling */
    *oversampling = ctrl_humi_reg.bits.humidity_oversampling;

    return ESP_OK;
}

esp_err_t bme680_set_humidity_oversampling(bme680_handle_t handle, const bme680_humidity_oversampling_t oversampling) {
    bme680_control_humidity_register_t   ctrl_humi_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control humidity register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_humidity_register(device, &ctrl_humi_reg), TAG, "read control humidity register for get humidity oversampling failed" );

    /* set oversampling */
    ctrl_humi_reg.bits.humidity_oversampling = oversampling;

    /* attempt to write control humidity register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_control_humidity_register(device, ctrl_humi_reg), TAG, "write control humidity register for get humidity oversampling failed" );

    return ESP_OK;
}

esp_err_t bme680_get_iir_filter(bme680_handle_t handle, bme680_iir_filters_t *const iir_filter) {
    bme680_config_register_t   config_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_configuration_register(device, &config_reg), TAG, "read configuration register for get IIR filter failed" );

    /* set standby time */
    *iir_filter = config_reg.bits.iir_filter;

    return ESP_OK;
}

esp_err_t bme680_set_iir_filter(bme680_handle_t handle, const bme680_iir_filters_t iir_filter) {
    bme680_config_register_t   config_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_configuration_register(device, &config_reg), TAG, "read configuration register for get IIR filter failed" );

    /* initialize configuration register */
    config_reg.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_configuration_register(device, config_reg), TAG, "write configuration register for set IIR filter failed" );

    return ESP_OK;
}

esp_err_t bme680_get_standby_time(bme680_handle_t handle, bme680_standby_times_t *const standby_time) {
    bme680_config_register_t   config_reg;
    bme680_control_gas1_register_t gas1_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_configuration_register(device, &config_reg), TAG, "read configuration register for get stand-by time failed" );

    /* attempt to read gas 1 register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_gas1_register(device, &gas1_reg), TAG, "read control gas 1 register for get stand-by time failed" );

    /* set standby time */
    *standby_time = (bme680_standby_times_t)(gas1_reg.bits.standby_period_msb<<3 | config_reg.bits.standby_period_lsb);

    return ESP_OK;
}

esp_err_t bme680_set_standby_time(bme680_handle_t handle, const bme680_standby_times_t standby_time) {
    bme680_config_register_t   config_reg;
    bme680_control_gas1_register_t gas1_reg;
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_configuration_register(device, &config_reg), TAG, "read configuration register for set stand-by time failed" );

    /* attempt to read gas 1 register */
    ESP_RETURN_ON_ERROR( bme680_i2c_get_control_gas1_register(device, &gas1_reg), TAG, "read control gas 1 register for set stand-by time failed" );

    /* set standby time */
    gas1_reg.bits.standby_period_msb = standby_time>>3 & 1;
    config_reg.bits.standby_period_lsb = standby_time & 0b111;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_configuration_register(device, config_reg), TAG, "write configuration register for set stand-by time failed" );

    /* attempt to write gas 1 register */
    ESP_RETURN_ON_ERROR( bme680_i2c_set_control_gas1_register(device, gas1_reg), TAG, "write control gas 1 register for set stand-by time failed" );

    return ESP_OK;
}

esp_err_t bme680_reset(bme680_handle_t handle) {
    bme680_device_t* device = (bme680_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c transaction */
    ESP_RETURN_ON_ERROR( bme680_i2c_write_byte_to(device, BME680_REG_RESET, BME680_RESET_VALUE), TAG, "write reset register for reset failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(BME680_RESET_DELAY_MS)); // check is busy in timeout loop...

    /* attempt to setup device */
    ESP_RETURN_ON_ERROR( bme680_i2c_setup(device), TAG, "setup device for reset failed" );

    /* attempt to setup device heaters */
    ESP_RETURN_ON_ERROR( bme680_i2c_setup_heater(device), TAG, "setup device heaters for reset failed" );

    return ESP_OK;
}

esp_err_t bme680_remove(bme680_handle_t handle) {
    bme680_device_t* device = (bme680_device_t*)handle;

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

esp_err_t bme680_delete(bme680_handle_t handle){
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    esp_err_t ret = bme680_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}

const char* bme680_get_fw_version(void) {
    return (const char*)BME680_FW_VERSION_STR;
}

int32_t bme680_get_fw_version_number(void) {
    return (int32_t)BME680_FW_VERSION_INT32;
}
