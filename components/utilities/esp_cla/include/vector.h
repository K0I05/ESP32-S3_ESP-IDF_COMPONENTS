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
 * @file vector.h
 * @defgroup 
 * @{
 *
 * ESP-IDF compact linear algebra (cla) vector library
 * 
 * examples
 * https://github.com/goldsborough/vector
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CLA_VECTOR_H__
#define __CLA_VECTOR_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_check.h>




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
 * @brief Vector structure definition.
 */
typedef struct cla_vector_s {
    uint16_t num_cmps;   /*!< number of components in the vector */
    double *data;        /*!< 1 dimensional array for vector components */
    bool is_2d;          /*!< true if vector is 2D, false otherwise */
    bool is_3d;          /*!< true if vector is 3D, false otherwise */
} cla_vector_t;

/**
 * @brief Pointer to a vector structure.
 */
typedef cla_vector_t* cla_vector_ptr_t;

/**
 * @brief Vector iterator structure definition.
 */
typedef struct cla_vector_iterator_s {
    uint16_t cmp_size;
    double* data_ptr;
} cla_vector_iterator_t;


/**
 * public function prototypes
 */

// iterator example in C: https://github.com/goldsborough/vector
/*
cla_vector_iterator_t cla_vector_begin(const cla_vector_ptr_t v);
cla_vector_iterator_t cla_vector_end(const cla_vector_ptr_t v);
cla_vector_iterator_t cla_vector_iterator(const cla_vector_ptr_t v, const uint16_t cmp_idx);

double* cla_vector_iterator_get(const cla_vector_iterator_t it);

esp_err_t cla_vector_iterator_delete(cla_vector_ptr_t *const v, cla_vector_iterator_t *const it);

void cla_vector_iterator_increment(cla_vector_iterator_t *const it);
void cla_vector_iterator_decrement(cla_vector_iterator_t *const it);

double* cla_vector_next(cla_vector_iterator_t *const it);
double* cla_vector_previous(cla_vector_iterator_t *const it);

bool cla_vector_iterator_equals(const cla_vector_iterator_t it1, const cla_vector_iterator_t it2);
bool cla_vector_iterator_is_before(const cla_vector_iterator_t it1, const cla_vector_iterator_t it2);
bool cla_vector_iterator_is_after(const cla_vector_iterator_t it1, const cla_vector_iterator_t it2);

uint16_t cla_vector_iterator_index(const cla_vector_ptr_t v, const cla_vector_iterator_t it);
*/



/**
 * @brief Instantiates a vector by the number of components.
 * 
 * @param num_cmps Number of components for the vector and the maximum allowed is 128 components.
 * @param v An instantiated vector structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_create(const uint16_t num_cmps, cla_vector_ptr_t *const v);

/**
 * @brief Deletes a vector instance to free memory.
 * 
 * @param v Vector instance to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_delete(cla_vector_ptr_t v);

/**
 * @brief Prints the vector components to the console.
 * 
 * @param v Vector to print to the console.
 * @return esp_err_t ESP_OK on success. 
 */
esp_err_t cla_vector_print(cla_vector_ptr_t v);

/**
 * @brief Adds two vectors component-wise.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param v_sum Sum of vector 1 and vector 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_add(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_sum);

/**
 * @brief Subtracts vector 2 from vector 1 component-wise.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param v_difference Vector difference of vector 1 minus vector 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_subtract(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_difference);

/**
 * @brief Multiplies two vectors component-wise.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param v_product Product of vector 1 and vector 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_multiply(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_product);

/**
 * @brief Divides vector 1 by vector 2 component-wise.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param v_quotient Quotient of vector 1 divided by vector 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_divide(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_quotient);

/**
 * @brief Calculates the DOT product of two multi-dimensional vectors.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param v_dot DOT product result vector.
 * @return esp_err_t 
 */
esp_err_t cla_vector_get_dot_product(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_dot);

/**
 * @brief Calculates the CROSS product of two 3D vectors.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param v_cross Cross product result vector.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_get_cross_product(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_cross);

/**
 * @brief Copies a source vector to a destination vector.
 * 
 * @param v_src Source vector to copy from.
 * @param v_dst Destination vector to copy to.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_copy(const cla_vector_ptr_t v_src, cla_vector_ptr_t *const v_dst);

/**
 * @brief Scales vector components by a scalar value.
 * 
 * @param v Vector to scale.
 * @param scalar Scaling factor which must be greater than 0.
 * @param v_scaled Scaled vector result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_scale(const cla_vector_ptr_t v, const double scalar, cla_vector_ptr_t *const v_scaled);

/**
 * @brief Checks if two vectors have the same dimension.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param equal True if vectors have the same dimension, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_is_dimension_equal(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, bool *const is_equal);

/**
 * @brief Checks if two vectors are equal within a specified tolerance.
 * 
 * @param v1 Vector 1.
 * @param v2 Vector 2.
 * @param tolerance Tolerance value for component comparison.
 * @param equal True if vectors are equal, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_is_equal(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, const double tolerance, bool *const is_equal);

/**
 * @brief Checks if a vector is empty (has zero components).
 * 
 * @param v Vector to check.
 * @param empty True if vector is empty, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_is_empty(const cla_vector_ptr_t v, bool *const is_empty);

/**
 * @brief Checks if a vector is 2-dimensional.
 * 
 * @param v Vector to check.
 * @param is_2d True if vector is 2D, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_is_2d(const cla_vector_ptr_t v, bool *const is_2d);

/**
 * @brief Checks if a vector is 3-dimensional.
 * 
 * @param v Vector to check.
 * @param is_3d True if vector is 3D, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_is_3d(const cla_vector_ptr_t v, bool *const is_3d);

/**
 * @brief Gets the value of a vector component by index.
 * 
 * @param cmp_idx Vector component index to get value from.
 * @param v Vector to get component value from.
 * @param cmp_value Vector component value retrieved from the vector.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_get_value(const uint16_t cmp_idx, const cla_vector_ptr_t v, double *const cmp_value);

/**
 * @brief Sets the value of a vector component by index.
 * 
 * @param cmp_idx Vector component index to set value for.
 * @param cmp_value Vector component value to set in the vector.
 * @param v Vector to set component value in.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_set_value(const uint16_t cmp_idx, const double cmp_value, cla_vector_ptr_t *const v);

/**
 * @brief Sets all components of the vector to a specified value.
 * 
 * @param cmp_value Value to set all vector components to.
 * @param v Vector to set component values in.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_set_values(const double cmp_value, cla_vector_ptr_t *const v);

/**
 * @brief Sets all components of the vector to 0.
 * 
 * @param v Vector to set all component values to 0.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_zero_values(cla_vector_ptr_t *const v);

/**
 * @brief
 * 
 * @param v 
 * @return esp_err_t 
 */
esp_err_t cla_vector_add_component(cla_vector_ptr_t *const v);

/**
 * @brief 
 * 
 * @param cmp_idx 
 * @param v 
 * @return esp_err_t 
 */
esp_err_t cla_vector_delete_component(const uint16_t cmp_idx, cla_vector_ptr_t *const v);

/**
 * @brief 
 * 
 * @param v 
 * @param v_normalized 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_normalize(const cla_vector_ptr_t v, cla_vector_ptr_t *const v_normalized);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CLA_VECTOR_H__
