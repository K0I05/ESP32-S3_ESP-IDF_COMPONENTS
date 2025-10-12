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
 * @file sgp4x.h
 * @defgroup drivers sgp4x
 * @{
 *
 * ESP-IDF driver for sgp4x sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SGP4X_H__
#define __SGP4X_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "sgp4x_version.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * SGP4X definitions
 */
#define I2C_SGP4X_DEV_CLK_SPD           UINT32_C(100000) //!< sgp4x I2C default clock frequency (100KHz)

#define I2C_SGP4X_DEV_ADDR              UINT8_C(0x59) //!< sgp4x I2C address

/*
 * SGP4X macro definitions
 */
#define SGP41_CONFIG_DEFAULT {                   \
    .i2c_address        = I2C_SGP4X_DEV_ADDR,        \
    .i2c_clock_speed    = I2C_SGP4X_DEV_CLK_SPD,     \
    .dev_version        = SGP4X_VERSION_SGP41 }


/**
 * @brief SGP4X versions enumerator.
 */
typedef enum sgp4x_versions_e {
    SGP4X_VERSION_SGP40,  /*!< not implemented */
    SGP4X_VERSION_SGP41
} sgp4x_versions_t;

/**
 * @brief SGP4X self-test result structure.
 */
typedef union __attribute__((packed)) {
    struct {
        bool voc_pixel_failed:1;    /*!< one or more tests have failed when true     (bit:0)  */
        bool nox_pixel_failed:1;    /*!< one or more tests have failed when true     (bit:1) */
        uint8_t reserved:6;         /*!< reserved      (bit:2-7) */
    } pixels;
    uint8_t integrity;
} sgp4x_self_test_result_t;

/**
 * @brief SGP4X device configuration structure definition.
 */
typedef struct {
    uint16_t                i2c_address;        /*!< sgp4x i2c device address */
    uint32_t                i2c_clock_speed;    /*!< sgp4x i2c device scl clock speed  */
    sgp4x_versions_t        dev_version;        /*!< sgp4x version */
} sgp4x_config_t;


/**
 * @brief SGP4X opaque handle structure definition.
 */
typedef void* sgp4x_handle_t;


/**
 * @brief Initializes an SGP4X device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] sgp4x_config SGP4X device configuration.
 * @param[out] sgp4x_handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_init(i2c_master_bus_handle_t master_handle, const sgp4x_config_t *sgp4x_config, sgp4x_handle_t *sgp4x_handle);

/**
 * @brief Starts the conditioning with temperature and humidity compensation, i.e., the VOC pixel will be operated at 
 * the same temperature as it is by calling the sgp41_measure_raw_signals function while the NOx pixel will be operated 
 * at a different temperature for conditioning.  The conditioning should be ran when the device is initially powered for
 * 10-seconds but do not run the conditioning longer than 10-seconds, otherwise damage may occur to the SGP4X.
 * 
 * @param[in] handle SGP4X device handle.
 * @param[in] temperature Temperature compensation in degree celsius.
 * @param[in] humidity Humidity compensation in percentage.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_execute_compensated_conditioning(sgp4x_handle_t handle, const float temperature, const float humidity, uint16_t *sraw_voc);

/**
 * @brief Starts the conditioning, i.e., the VOC pixel will be operated at the same temperature as it is by calling the
 * sgp41_measure_raw_signals function while the NOx pixel will be operated at a different temperature for conditioning.
 * The conditioning should be ran when the device is initially powered for 10-seconds but do not run the conditioning 
 * longer than 10-seconds, otherwise damage may occur to the SGP4X.
 * 
 * @param[in] handle SGP4X device handle.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_execute_conditioning(sgp4x_handle_t handle, uint16_t *sraw_voc);

/**
 * @brief Starts and/or continues the VOC and NOX measurement mode with temperature and humidity compensation.
 * 
 * @param[in] handle SGP4X device handle.
 * @param[in] temperature Temperature compensation in degree Celsius.
 * @param[in] humidity Humidity compensation in percentage.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @param[out] sraw_nox Raw signal of NOX in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_measure_compensated_signals(sgp4x_handle_t handle, const float temperature, const float humidity, uint16_t *sraw_voc, uint16_t *sraw_nox);

/**
 * @brief Starts and/or continues the VOC and NOX measurement mode using default temperature and humidity compensation.
 * 
 * @param[in] handle SGP4X device handle.
 * @param[out] sraw_voc Raw signal of VOC in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @param[out] sraw_nox Raw signal of NOX in ticks which is proportional to the logarithm of the resistance of the sensing element.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_measure_signals(sgp4x_handle_t handle, uint16_t *sraw_voc, uint16_t *sraw_nox);

/**
 * @brief Performs the built-in self-test that checks for integrity of 
 * both hotplate and MOX material.
 * 
 * @param[in] handle SGP4X device handle.
 * @param[out] result Results of the self-tests.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_execute_self_test(sgp4x_handle_t handle, sgp4x_self_test_result_t *const result);

/**
 * @brief Turns the hotplate off, stops the measurement, and SGP4X enters idle mode.
 * 
 * @param[in] handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_disable_heater(sgp4x_handle_t handle);

/**
 * @brief Issues soft-reset and initializes SGP4X.  See datasheet for details.
 *
 * @param[in] handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_reset(sgp4x_handle_t handle);

/**
 * @brief Removes an SGP4X device from master bus.
 *
 * @param[in] handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_remove(sgp4x_handle_t handle);

/**
 * @brief Removes an SGP4X device from master bus and frees handle.
 * 
 * @param[in] handle SGP4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sgp4x_delete(sgp4x_handle_t handle);

/**
 * @brief Converts SGP4X firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* SGP4X firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* sgp4x_get_fw_version(void);

/**
 * @brief Converts SGP4X firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t SGP4X firmware version number.
 */
int32_t sgp4x_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __SGP4X_H__
