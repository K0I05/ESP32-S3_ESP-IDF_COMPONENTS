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
 * @file quaternion.h
 * @defgroup math 3d_math
 * @{
 *
 * ESP-IDF 3d math library
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Quaternion definitions.
 * 
 */
typedef struct quaternion_t quaternion_t;
/**
 * @brief Quaternion handle structure.
 * 
 */
typedef struct quaternion_t *quaternion_handle_t;
/**
 * @brief Quaternion structure.
 * 
 */
struct quaternion_t {
    float w;
    float x;
    float y;
    float z;
};

esp_err_t quaternion_init(quaternion_handle_t *quaternion_handle);
esp_err_t quaternion_init_data(quaternion_handle_t quaternion_handle, float nw, float nx, float ny, float nz);
esp_err_t quaternion_get_product(quaternion_handle_t quaternion_handle, quaternion_handle_t *product_handle);
esp_err_t quaternion_get_conjugate(quaternion_handle_t quaternion_handle, quaternion_handle_t *conjugate_handle);
esp_err_t quaternion_get_magnitude(quaternion_handle_t quaternion_handle, float *magnitude);
esp_err_t quaternion_normalize(quaternion_handle_t quaternion_handle);
esp_err_t quaternion_get_normalized(quaternion_handle_t quaternion_handle, quaternion_handle_t *normalized_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __QUATERNION_H__
