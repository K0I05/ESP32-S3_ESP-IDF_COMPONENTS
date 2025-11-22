/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
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
 * @file cla_common.h
 * @defgroup 
 * @{
 *
 * ESP-IDF compact linear algebra (cla) library
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CLA_COMMON_H__
#define __CLA_COMMON_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>



#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */


/**
 * public macro definitions
 */


/**
 * public enumerator, union, and structure definitions
 */




/**
 * public function & subroutine prototype definitions
 */

/**
 * @brief Swaps two values.
 * 
 * @param val1 The first value to swap with the second value.
 * @param val2 The second value to swap with the first value.
 */
void cla_swap_values(double *const val1, double *const val2);

/**
 * @brief Copies the contents of one buffer to another.
 * 
 * @param pa Pointer to the source buffer.
 * @param pb Pointer to the destination buffer.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_copy_buffer(const void *pa, const void *pb);

/**
 * @brief Checks if two double-precision floating-point values are approximately equal within a given tolerance.
 * 
 * @param val1 The first value.
 * @param val2 The second value.
 * @param tolerance The tolerance for comparison. It's used for both absolute and relative error checks.
 * @return bool True when values are equal to each other, otherwise, false.
 */
bool cla_is_value_equal(const double val1, const double val2, const double tolerance);

/**
 * @brief Checks if a given unsigned integer value is a power of two.
 * 
 * @param val The unsigned integer to check.
 * @return bool True if the number is a power of two, otherwise false.
 */
bool cla_is_value_power_of_two(const uint16_t val);

/**
 * @brief Calculates heading in degrees from magnetic x and y components.
 * 
 * @param x_axis X axis magnetic component.
 * @param y_axis Y axis magnetic component.
 * @return float Heading in degrees.
 */
float cla_get_heading(const float x_axis, const float y_axis);

/**
 * @brief Calculates heading from true north in degrees from magnetic x and y components and magnetic declination.
 * 
 * @param x_axis X axis magnetic component.
 * @param y_axis Y axis magnetic component.
 * @param declination Magnetic declination in degrees (+east / -west).
 * @return float Heading from true north in degrees.
 */
float cla_get_true_heading(const float x_axis, const float y_axis, const float declination);

/**
 * @brief Calculates the magnitude of the earth's magnetic field from its x, y, and z components.
 * 
 * @param x_axis Magnetic field x component.
 * @param y_axis Magnetic field y component.
 * @param z_axis Magnetic field z component.
 * @return float Magnitude of the earth's magnetic field.
 */
float cla_get_earth_field(const float x_axis, const float y_axis, const float z_axis);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CLA_COMMON_H__
