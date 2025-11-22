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
 * @file vector.c
 *
 * ESP-IDF compact linear algebra (cla) vector library
 * 
 * 
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <math.h>
#include <float.h>
#include <stdint.h>
#include "include/cla_vector.h"

#define CLA_VECTOR_CMPS_SIZE_MAX    UINT16_C(INT16_MAX)

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "cla_vector";

/**
 * @brief Get offset pointer for vector component index.
 * 
 * @param v Vector pointer.
 * @param cmp_idx Component index.
 * @return double* Vector data offset pointer.
 */
static inline double* get_offset(const cla_vector_ptr_t v, const uint16_t cmp_idx) {
	return v->data + (cmp_idx * v->num_cmps);
}

esp_err_t cla_vector_create(const uint16_t num_cmps, cla_vector_ptr_t *const v) {
    ESP_RETURN_ON_FALSE( (num_cmps > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid vector dimensions, number of components must be greater than 0" );
    ESP_RETURN_ON_FALSE( (num_cmps <= CLA_VECTOR_CMPS_SIZE_MAX), ESP_ERR_INVALID_ARG, TAG, "Invalid vector dimensions, number of components exceeds maximum allowed" );
    cla_vector_ptr_t vector = (cla_vector_ptr_t)calloc(1, sizeof(cla_vector_t));
    ESP_RETURN_ON_FALSE( (vector != NULL), ESP_ERR_NO_MEM, TAG, "Unable to allocate memory for vector structure" );
    vector->num_cmps = num_cmps;
    vector->is_2d    = (num_cmps == 2) ? true : false;
    vector->is_3d    = (num_cmps == 3) ? true : false;
    vector->data     = (double*)calloc(num_cmps, sizeof(double));
    ESP_RETURN_ON_FALSE( (vector->data != NULL), ESP_ERR_NO_MEM, TAG, "Unable to allocate memory for vector data array" );
    *v = vector;
    return ESP_OK;
}

esp_err_t cla_vector_delete(cla_vector_ptr_t v) {
    ESP_ARG_CHECK(v);
    free(v->data);
    free(v);
    return ESP_OK;
}

esp_err_t cla_vector_print(cla_vector_ptr_t v) {
    ESP_ARG_CHECK(v);
    printf("\n");
    for(uint16_t i = 0; i < v->num_cmps; i++) {
        printf("%6.6lf\t", v->data[i]);
    }
    printf("\n");
    return ESP_OK;
}

esp_err_t cla_vector_add(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_sum) {
    ESP_ARG_CHECK(v1 && v2);
    ESP_RETURN_ON_FALSE( (v1->num_cmps == v2->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Vectors must have the same dimension to perform addition" );
    ESP_RETURN_ON_ERROR( cla_vector_create(v1->num_cmps, v_sum), TAG, "Unable to create vector instance, addition failed" );
    for(uint16_t i = 0; i < (*v_sum)->num_cmps; i++) {
        (*v_sum)->data[i] = v1->data[i] + v2->data[i];
    }
    return ESP_OK;
}

esp_err_t cla_vector_subtract(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_difference) {
    ESP_ARG_CHECK(v1 && v2);
    ESP_RETURN_ON_FALSE( (v1->num_cmps == v2->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Vectors must have the same dimension to perform subtraction" );
    ESP_RETURN_ON_ERROR( cla_vector_create(v1->num_cmps, v_difference), TAG, "Unable to create vector instance, subtraction failed" );
    for(uint16_t i = 0; i < (*v_difference)->num_cmps; i++) {
        (*v_difference)->data[i] = v1->data[i] - v2->data[i];
    }
    return ESP_OK;
}

esp_err_t cla_vector_multiply(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_product) {
    ESP_ARG_CHECK(v1 && v2);
    ESP_RETURN_ON_FALSE( (v1->num_cmps == v2->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Vectors must have the same dimension to perform multiplication" );
    ESP_RETURN_ON_ERROR( cla_vector_create(v1->num_cmps, v_product), TAG, "Unable to create vector instance, multiplication failed" );
    for(uint16_t i = 0; i < (*v_product)->num_cmps; i++) {
        (*v_product)->data[i] = v1->data[i] * v2->data[i];
    }
    return ESP_OK;
}

esp_err_t cla_vector_get_dot_product(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_dot) {
    ESP_ARG_CHECK(v1 && v2);
    ESP_RETURN_ON_FALSE( (v1->num_cmps == v2->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Vectors must have the same dimension to calculate dot product" );
    ESP_RETURN_ON_ERROR( cla_vector_create(1, v_dot), TAG, "Unable to create vector instance, dot product calculation failed" );
    (*v_dot)->data[0] = 0.0;
    for(uint16_t i = 0; i < (*v_dot)->num_cmps; i++) {
        (*v_dot)->data[0] += v1->data[i] * v2->data[i];
    }
    return ESP_OK;
}

esp_err_t cla_vector_get_cross_product(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *const v_cross) {
    ESP_ARG_CHECK(v1 && v2);
    ESP_RETURN_ON_FALSE( (v1->num_cmps == v2->num_cmps && v1->num_cmps == 3), ESP_ERR_INVALID_ARG, TAG, "Vectors must be 3-dimensional to calculate cross product" );
    ESP_RETURN_ON_ERROR( cla_vector_create(3, v_cross), TAG, "Unable to create vector instance, cross product calculation failed" );
    (*v_cross)->data[0] = v1->data[1] * v2->data[2] - v1->data[2] * v2->data[1];
    (*v_cross)->data[1] = v1->data[2] * v2->data[0] - v1->data[0] * v2->data[2];
    (*v_cross)->data[2] = v1->data[0] * v2->data[1] - v1->data[1] * v2->data[0];
    return ESP_OK;
}

esp_err_t cla_vector_copy(const cla_vector_ptr_t v_src, cla_vector_ptr_t *const v_dst) {
    ESP_ARG_CHECK(v_src);
    ESP_RETURN_ON_ERROR( cla_vector_create(v_src->num_cmps, v_dst), TAG, "Unable to create vector instance, copy vector failed" );
    for(uint16_t i = 0; i < (*v_dst)->num_cmps; i++) {
        (*v_dst)->data[i] = v_src->data[i];
    }
    return ESP_OK;
}

esp_err_t cla_vector_scale(const cla_vector_ptr_t v, const double scalar, cla_vector_ptr_t *const v_scaled) {
    ESP_ARG_CHECK(v);
    ESP_RETURN_ON_FALSE( (v_scaled != 0), ESP_ERR_INVALID_ARG, TAG, "Invalid destination vector, cannot be NULL" );
    ESP_RETURN_ON_ERROR( cla_vector_create(v->num_cmps, v_scaled), TAG, "Unable to create vector instance, scale vector failed" );
    for(uint16_t i = 0; i < (*v_scaled)->num_cmps; i++) {
        (*v_scaled)->data[i] = v->data[i] * scalar;
    }
    return ESP_OK;
}

esp_err_t cla_vector_is_dimension_equal(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, bool *const is_equal) {
    ESP_ARG_CHECK(v1 && v2);
    *is_equal = (v1->num_cmps == v2->num_cmps) ? true : false;
    return ESP_OK;
}

esp_err_t cla_vector_is_equal(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, const double tolerance, bool *const is_equal) {
    ESP_ARG_CHECK(v1 && v2);
    ESP_RETURN_ON_FALSE( (v1->num_cmps == v2->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Vectors must have the same dimension to compare equality" );
    for(uint16_t i = 0; i < v1->num_cmps; i++) {
        const double diff = fabs(v1->data[i] - v2->data[i]);
        if(diff > tolerance || diff > fmax(fabs(v1->data[i]), fabs(v2->data[i])) * tolerance) {
            *is_equal = false;
            return ESP_OK;
        }
    }
    *is_equal = true;
    return ESP_OK;
}

esp_err_t cla_vector_is_empty(const cla_vector_ptr_t v, bool *const is_empty) {
    ESP_ARG_CHECK(v);
    *is_empty = (v->num_cmps == 0) ? true : false;
    return ESP_OK;
}

esp_err_t cla_vector_is_2d(const cla_vector_ptr_t v, bool *const is_2d) {
    ESP_ARG_CHECK(v);
    *is_2d = (v->num_cmps == 2) ? true : false;
    return ESP_OK;
}

esp_err_t cla_vector_is_3d(const cla_vector_ptr_t v, bool *const is_3d) {
    ESP_ARG_CHECK(v);
    *is_3d = (v->num_cmps == 3) ? true : false;
    return ESP_OK;
}

esp_err_t cla_vector_get_value(const uint16_t cmp_idx, const cla_vector_ptr_t v, double *const cmp_value) {
    ESP_ARG_CHECK(v);
    ESP_RETURN_ON_FALSE( (cmp_idx < v->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Component index out of bounds" );
    *cmp_value = v->data[cmp_idx];
    return ESP_OK;
}

esp_err_t cla_vector_set_value(const uint16_t cmp_idx, const double cmp_value, cla_vector_ptr_t *const v) {
    ESP_ARG_CHECK(v);
    ESP_RETURN_ON_FALSE( (cmp_idx < (*v)->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Component index out of bounds" );
    (*v)->data[cmp_idx] = cmp_value;
    return ESP_OK;
}

esp_err_t cla_vector_set_values(const double cmp_value, cla_vector_ptr_t *const v) {
    ESP_ARG_CHECK(v);
    for(uint16_t i = 0; i < (*v)->num_cmps; i++) {
        (*v)->data[i] = cmp_value;
    }
    return ESP_OK;
}

esp_err_t cla_vector_zero_values(cla_vector_ptr_t *const v) {
    ESP_ARG_CHECK(v);
    for(uint16_t i = 0; i < (*v)->num_cmps; i++) {
        (*v)->data[i] = 0.0;
    }
    return ESP_OK;
}

esp_err_t cla_vector_add_component(cla_vector_ptr_t *const v) {
    ESP_ARG_CHECK(v);
    cla_vector_ptr_t v_a = NULL;
    ESP_RETURN_ON_ERROR( cla_vector_create((*v)->num_cmps + 1, &v_a), TAG, "Unable to create vector instance, add component failed" );
    for(uint16_t i = 0; i < (*v)->num_cmps; i++) {
        v_a->data[i] = (*v)->data[i];
    }
    cla_vector_delete(*v);
    *v = v_a;
    return ESP_OK;
}

esp_err_t cla_vector_delete_component(const uint16_t cmp_idx, cla_vector_ptr_t *const v) {
    ESP_ARG_CHECK(v);
    ESP_RETURN_ON_FALSE( (cmp_idx < (*v)->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Component index out of bounds" );
    cla_vector_ptr_t v_d = NULL;
    ESP_RETURN_ON_ERROR( cla_vector_create((*v)->num_cmps - 1, &v_d), TAG, "Unable to create vector instance, delete component failed" );
    for(uint16_t i = 0, j = 0; i < (*v)->num_cmps; i++) {
        if (i != cmp_idx) {
            v_d->data[j++] = (*v)->data[i];
        }
    }
    cla_vector_delete(*v);
    *v = v_d;
    return ESP_OK;
}






/*

cla_vector_iterator_t cla_vector_begin(const cla_vector_ptr_t v) {
    return cla_vector_iterator(v, 0);
}

cla_vector_iterator_t cla_vector_end(const cla_vector_ptr_t v) {
    return cla_vector_iterator(v, v->num_cmps);
}

cla_vector_iterator_t cla_vector_iterator(const cla_vector_ptr_t v, const uint16_t cmp_idx) {
    cla_vector_iterator_t it = { .data_ptr = NULL, .cmp_size = 0 };
    if(v == NULL) return it;
    if(cmp_idx >= v->num_cmps) return it;
    if(v->num_cmps == 0) return it;
    it.data_ptr = cla_vector_get_offset(v, cmp_idx);
    it.cmp_size = v->num_cmps;
    return it;
}

double* cla_vector_iterator_get(const cla_vector_iterator_t it) {
    return it.data_ptr;
}

esp_err_t cla_vector_iterator_delete(cla_vector_ptr_t *const v, cla_vector_iterator_t *const it) {
    ESP_ARG_CHECK(v && it);
    uint16_t cmp_idx = cla_vector_iterator_index(*v, *it);
    ESP_RETURN_ON_ERROR( cla_vector_delete_component(cmp_idx, v), TAG, "Unable to delete component from vector" );
    *it = cla_vector_iterator(*v, cmp_idx);
    return ESP_OK;
}

void cla_vector_iterator_increment(cla_vector_iterator_t *const it) {
    *it->data_ptr += it->cmp_size;
}

void cla_vector_iterator_decrement(cla_vector_iterator_t *const it) {
    *it->data_ptr -= it->cmp_size;
}

double* cla_vector_next(cla_vector_iterator_t *const it) {
    double* current = it->data_ptr;
    cla_vector_iterator_increment(it);
    return current;
}

double* cla_vector_previous(cla_vector_iterator_t *const it) {
    double* current = it->data_ptr;
    cla_vector_iterator_decrement(it);
    return current;
}

bool cla_vector_iterator_equals(const cla_vector_iterator_t it1, const cla_vector_iterator_t it2) {
    return (it1.cmp_size == it2.cmp_size) ? true : false;
}

bool cla_vector_iterator_is_before(const cla_vector_iterator_t it1, const cla_vector_iterator_t it2) {
    return (it1.data_ptr < it2.data_ptr) ? true : false;
}

bool cla_vector_iterator_is_after(const cla_vector_iterator_t it1, const cla_vector_iterator_t it2) {
    return (it1.data_ptr > it2.data_ptr) ? true : false;
}

uint16_t cla_vector_iterator_index(const cla_vector_ptr_t v, const cla_vector_iterator_t it) {
    return (it.data_ptr - v->data) / v->num_cmps;
}

*/
