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
 * @file pressure_tendency.h
 *
 * Air pressure tendency libary
 * 
 * A air pressure tendency appears after three (3) or six (6) hours of operation. The tendency 
 * codes and change are based on the 3-hr or 6-hr variance from the previous 3-hr or 6-hr history.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __PRESSURE_TENDENCY_H__
#define __PRESSURE_TENDENCY_H__

#include <stdio.h>
#include <esp_check.h>
#include "pressure_tendency_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Pressure tendency codes enumerator.
 */
typedef enum pressure_tendency_codes_tag {
    PRESSURE_TENDENCY_CODE_UNKNOWN   = 1,  /*!< pressure tendency is unknown */
    PRESSURE_TENDENCY_CODE_RISING    = 2,  /*!< pressure tendency is rising  */
    PRESSURE_TENDENCY_CODE_STEADY    = 3,  /*!< pressure tendency is steady  */
    PRESSURE_TENDENCY_CODE_FALLING   = 4,  /*!< pressure tendency is falling  */
    PRESSURE_TENDENCY_CODE_TRAINING  = 5   /*!< pressure tendency is training  */
} pressure_tendency_codes_t;

/**
 * @brief Pressure tendency processing periods enumerator.
 */
typedef enum pressure_tendency_periods_e {
    PRESSURE_TENDENCY_3HR_PERIOD = 1,   /*!< 3-hr period */
    PRESSURE_TENDENCY_6HR_PERIOD = 2,   /*!< 6-hr period */
} pressure_tendency_periods_t;

/**
 * @brief Pressure tendency opaque handle structure definition.
 */
typedef void* pressure_tendency_handle_t;

/**
 * @brief Converts `pressure_tendency_codes_t` enumerator pressure tendency code to string.
 * 
 * @param code Pressure tendency code to convert to a string.
 * @return const char* String representation of the pressure tendency code.
 */
const char* pressure_tendency_code_to_string(const pressure_tendency_codes_t code);

/**
 * @brief Initializes a pressure tendency handle by sampling interval of the 3-hr or 6-hr samples 
 * to analyze.  The size of the samples is calculated from the sampling interval in seconds.  
 * As an example, if the sampling interval is once every minute, the 
 * size of the samples buffer should be 180 e.g., three (3) hours.
 * 
 * @param sampling_interval Pressure tendency sampling interval in seconds. 
 * @param sampling_period Pressure tendency sampling period.
 * @param pressure_tendency_handle Pressure tendency handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pressure_tendency_init(const uint16_t sampling_interval, 
                            pressure_tendency_periods_t sampling_period,
                            pressure_tendency_handle_t *pressure_tendency_handle);
/**
 * @brief Analyzes historical samples and pressure tendency appears after three 
 * (3) or six (6) hours of operation. The pressure tendency are based the 3-hr or 6-hr variance of 
 * the previous 3-hr or 6-hr history.
 * 
 * @param pressure_tendency_handle Pressure tendency handle.
 * @param sample Air pressure sample, in millibars or hecto-pascal, to push onto the samples stack.
 * @param code Pressure tendency code of three (3) or six (6) hour analysis.  Pressure
 * tendency code `PRESSURE_TENDENCY_UNKNOWN` is reported when the pressure tendency is unknown and 
 * `PRESSURE_TENDENCY_TRAINING` is reported when there is an insufficient number of samples to analyze.
 * @param change Pressure tendency variance over the past three (3) or six (6) hours.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pressure_tendency_analysis(pressure_tendency_handle_t pressure_tendency_handle, 
                                const float sample, 
                                pressure_tendency_codes_t *const code,
                                float *const change);

/**
 * @brief Purges pressure tendency samples array and resets samples counter.
 * 
 * @param pressure_tendency_handle Pressure tendency handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pressure_tendency_reset(pressure_tendency_handle_t pressure_tendency_handle);

/**
 * @brief Frees pressure tendency handle.
 * 
 * @param pressure_tendency_handle Pressure tendency handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pressure_tendency_delete(pressure_tendency_handle_t pressure_tendency_handle);

/**
 * @brief Converts `pressure_tendency` firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* `pressure_tendency` firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* pressure_tendency_get_fw_version(void);

/**
 * @brief Converts `pressure_tendency` firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t `pressure_tendency` firmware version number.
 */
int32_t pressure_tendency_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

#endif // __PRESSURE_TENDENCY_H__