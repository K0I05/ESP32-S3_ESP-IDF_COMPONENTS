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
 * @file vector_int16.h
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
#ifndef __VECTOR_INT16_H__
#define __VECTOR_INT16_H__

#include "quaternion.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Vector_int16 definitions.
 * 
 */
typedef struct vector_int16_t vector_int16_t;
/**
 * @brief Vector_int16 handle structure.
 * 
 */
typedef struct vector_int16_t *vector_int16_handle_t;
/**
 * @brief Vector_int16 structure.
 * 
 */
struct vector_int16_t {
    int16_t x;
    int16_t y;
    int16_t z;
};

esp_err_t vector_int16_init(vector_int16_handle_t *vector_int16_handle);
esp_err_t vector_int16_init_data(vector_int16_handle_t vector_int16_handle, int16_t nx, int16_t ny, int16_t nz);
esp_err_t vector_int16_get_magnitude(vector_int16_handle_t vector_int16_handle, float *magnitude);
esp_err_t vector_int16_normalize(vector_int16_handle_t vector_int16_handle);
esp_err_t vector_int16_get_normalized(vector_int16_handle_t vector_int16_handle, vector_int16_handle_t *normalized_handle);
esp_err_t vector_int16_rotate(vector_int16_handle_t vector_int16_handle, quaternion_handle_t quaternion_handle);
esp_err_t vector_int16_get_rotated(vector_int16_handle_t vector_int16_handle, quaternion_handle_t quaternion_handle, vector_int16_handle_t *rotated_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __VECTOR_INT16_H__
