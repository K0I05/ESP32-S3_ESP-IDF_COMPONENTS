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
 * @file s12sd.h
 * @defgroup drivers guva-s12sd uv sensor
 * @{
 *
 * ESP-IDF driver for guva-s12sd uv sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __S12SD_H__
#define __S12SD_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_check.h>
#include <esp_err.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "s12sd_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define ADC_S12SD_UNIT_DEFAULT        ADC_UNIT_1
#define ADC_S12SD_CHANNEL_DEFAULT     ADC_CHANNEL_0
#define ADC_S12SD_SAMPLE_SIZE         (1000)
#define ADC_S12SD_ATTEN               ADC_ATTEN_DB_12
#define ADC_S12SD_DIGI_BIT_WIDTH      (12)   //!< adc bit width at 12-bits

/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `adc_s12sd_config_t` to default configuration settings.
 */
#define S12SD_CONFIG_DEFAULT {                  \
    .adc_unit    = ADC_S12SD_UNIT_DEFAULT,          \
    .adc_channel = ADC_S12SD_CHANNEL_DEFAULT,  } 

/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief S12SD configuration structure definition.
 */
typedef struct s12sd_config_s {
    uint8_t     adc_unit;           /*!< s12sd adc unit */
    uint8_t     adc_channel;        /*!< s12sd adc channel */
} s12sd_config_t;

/**
 * @brief S12SD opaque handle structure definition.
 */
typedef void* s12sd_handle_t;

/**
 * public function and subroutine declarations
 */

/**
 * @brief Initializes an adc S12SD device.
 *
 * @param[in] s12sd_config S12SD device configuration.
 * @param[out] s12sd_handle S12SD device handle.
 * @return ESP_OK: init success.
 */
esp_err_t s12sd_init(const s12sd_config_t *s12sd_config, s12sd_handle_t *s12sd_handle);

/**
 * @brief Measure S12SD device.
 *
 * @param[in] handle S12SD device handle.
 * @param[out] uv_index uv index (1 to 11).
 * @return ESP_OK: init success.
 */
esp_err_t s12sd_measure(s12sd_handle_t handle, uint8_t *uv_index);

/**
 * @brief Deinitialize S12SD device.
 *
 * @param[in] handle S12SD device handle
 * @return ESP_OK: init success.
 */
esp_err_t s12sd_delete(s12sd_handle_t handle);

/**
 * @brief Converts S12SD firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* S12SD firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* s12sd_get_fw_version(void);

/**
 * @brief Converts S12SD firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t S12SD firmware version number.
 */
int32_t s12sd_get_fw_version_number(void);





#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __S12SD_H__
