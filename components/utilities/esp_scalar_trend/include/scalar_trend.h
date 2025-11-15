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
 * @file scalar_trend.h
 *
 * Scalar trend libary
 * 
 * A scalar Trend appears after one (1) hour of operation. The trend codes are a 
 * forecast of the 3-hr change based on the previous 1-hour history.
 * 
 * Original source: https://gist.github.com/Paraphraser/c5609f85cc7ee6ecd03ce179fb7f7edb
 * 
 * Original source code was modified to support the esp-idf framework and includes the 
 * left-tailed inverse of the studen's t-distribution calculation on handle initialization.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SCALAR_TREND_H__
#define __SCALAR_TREND_H__

#include <stdio.h>
#include <esp_check.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Scalar trend codes enumerator.
 */
typedef enum scalar_trend_codes_tag {        /*!< 1-hr Change */
    SCALAR_TREND_CODE_UNKNOWN   = 1, /*!< unknown */
    SCALAR_TREND_CODE_RISING    = 2, /*!< rising  */
    SCALAR_TREND_CODE_STEADY    = 3, /*!< steady  */
    SCALAR_TREND_CODE_FALLING   = 4  /*!< falling  */
} scalar_trend_codes_t;

/**
 * @brief Scalar trend opaque handle structure definition.
 */
typedef void* scalar_trend_handle_t;

/**
 * @brief Converts `scalar_trend_codes_t` enumerator trend code to string.
 * 
 * @param code Scalar trend code to convert to a string.
 * @return const char* String representation of the scalar trend code.
 */
const char* scalar_trend_code_to_string(const scalar_trend_codes_t code);

/**
 * @brief Initializes a scalar trend handle by size of the 1-hr samples 
 * to analyze.  The size of the samples is calculated from the sampling rate.  
 * As an example, if the sampling rate is once every minute, the 
 * size of the samples buffer should be 60 e.g., one (1) hour.
 * 
 * @param samples_size Scalar trend samples buffer size. 
 * @param scalar_trend_handle Scalar trend handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t scalar_trend_init(const uint16_t samples_size, scalar_trend_handle_t *scalar_trend_handle);
/**
 * @brief Analyzes historical samples and scalar Trend appears after one (1) 
 * hour of operation. The trend codes are a forecast of the 3-hr change based 
 * on the previous 1-hour history.
 * 
 * @param scalar_trend_handle Scalar trend handle.
 * @param sample Scalar sample to push onto the samples stack.
 * @param code Scalar trend code of one (1) hour analysis.  Scalar trend code 
 * `SCALAR_TREND_UNKNOWN` is reported when there is an insufficient number of 
 * samples to analyze.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t scalar_trend_analysis(scalar_trend_handle_t scalar_trend_handle, 
                                const float sample, 
                                scalar_trend_codes_t *const code);

/**
 * @brief Purges scalar trend samples array and resets samples counter.
 * 
 * @param scalar_trend_handle Scalar trend handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t scalar_trend_reset(scalar_trend_handle_t scalar_trend_handle);

/**
 * @brief Frees scalar trend handle.
 * 
 * @param scalar_trend_handle Scalar trend handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t scalar_trend_delete(scalar_trend_handle_t scalar_trend_handle);


#ifdef __cplusplus
}
#endif

#endif // __SCALAR_TREND_H__