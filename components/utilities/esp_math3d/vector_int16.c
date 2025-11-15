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
 * @file vector_int16.c
 *
 * ESP-IDF math 3d library
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "include/vector_int16.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declerations
*/
static const char *TAG = "quaternion";

esp_err_t vector_int16_init(vector_int16_handle_t *vector_int16_handle) {
    esp_err_t               ret = ESP_OK;
    vector_int16_handle_t   out_handle;

    /* validate memory availability for handle */
    out_handle = (vector_int16_handle_t)calloc(1, sizeof(vector_int16_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for vector int16 handle, init failed");

    /* set handle parameters */
    out_handle->x = 0.0f;
    out_handle->y = 0.0f;
    out_handle->z = 0.0f;

    /* set handle */
    *vector_int16_handle = out_handle;

    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}

esp_err_t vector_int16_init_data(vector_int16_handle_t vector_int16_handle, int16_t nx, int16_t ny, int16_t nz) {
    /* validate arguments */
    ESP_ARG_CHECK( vector_int16_handle );

    /* set handle parameters */
    vector_int16_handle->x = nx;
    vector_int16_handle->y = ny;
    vector_int16_handle->z = nz;

    return ESP_OK;
}

esp_err_t vector_int16_get_magnitude(vector_int16_handle_t vector_int16_handle, float *magnitude) {
    /* validate arguments */
    ESP_ARG_CHECK( vector_int16_handle );

    /* compute vector int16 magnitude */
    *magnitude = sqrt(vector_int16_handle->x*vector_int16_handle->x 
                + vector_int16_handle->y*vector_int16_handle->y 
                + vector_int16_handle->z*vector_int16_handle->z);

    return ESP_OK;
}

esp_err_t vector_int16_normalize(vector_int16_handle_t vector_int16_handle) {
    float m;

    /* validate arguments */
    ESP_ARG_CHECK( vector_int16_handle );
    
    /* attempt vector int16 magnitude */
    ESP_RETURN_ON_ERROR(vector_int16_get_magnitude(vector_int16_handle, &m), TAG, "Vector int16 get magnitude for normalize failed");

    /* set handle parameters */
    vector_int16_handle->x /= m;
    vector_int16_handle->y /= m;
    vector_int16_handle->z /= m;

    return ESP_OK;
}

esp_err_t vector_int16_get_normalized(vector_int16_handle_t vector_int16_handle, vector_int16_handle_t *normalized_handle) {
    esp_err_t               ret = ESP_OK;
    vector_int16_handle_t   out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( vector_int16_handle );

    /* attempt vector int16 init */
    ESP_GOTO_ON_ERROR(vector_int16_init(&out_handle), err, TAG, "Vector int16 init for get normalized failed");

    /* attempt vector int16 init data */
    ESP_GOTO_ON_ERROR(vector_int16_init_data(out_handle, vector_int16_handle->x, vector_int16_handle->y, vector_int16_handle->z), 
        err, TAG, "Vector int16 init data for get normalized failed");

    /* attempt vector int16 normalize */
    ESP_GOTO_ON_ERROR(vector_int16_normalize(out_handle), err, TAG, "Vector int16 normalize for get normalized failed");

    /* set handle */
    *normalized_handle = out_handle;
    
    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}

esp_err_t vector_int16_rotate(vector_int16_handle_t vector_int16_handle, quaternion_handle_t quaternion_handle) {
    esp_err_t               ret = ESP_OK;
    quaternion_handle_t     p_handle;

    /* validate arguments */
    ESP_ARG_CHECK( vector_int16_handle );

    /* attempt quaternion init */
    ESP_GOTO_ON_ERROR(quaternion_init(&p_handle), err, TAG, "Quaternion init for rotate failed");

    /* attempt quaternion init data */
    ESP_GOTO_ON_ERROR(quaternion_init_data(p_handle, 0 , vector_int16_handle->x, vector_int16_handle->y, vector_int16_handle->z), 
        err, TAG, "Quaternion init data for rotate failed");

    // attempt quaternion multiplication: q * p, stored back in p
    ESP_GOTO_ON_ERROR(quaternion_get_product(quaternion_handle, &p_handle), err, TAG, "Quaternion get product for rotate failed");

    // attempt quaternion multiplication: p * conj(q), stored back in p
    ESP_GOTO_ON_ERROR(quaternion_get_conjugate(quaternion_handle, &p_handle), err, TAG, "Quaternion get conjugate for rotate failed");
    ESP_GOTO_ON_ERROR(quaternion_get_product(quaternion_handle, &p_handle), err, TAG, "Quaternion get product for rotate failed");

    // p quaternion is now [0, x', y', z']
    vector_int16_handle->x = p_handle->x;
    vector_int16_handle->y = p_handle->y;
    vector_int16_handle->z = p_handle->z;
    
    return ESP_OK;

    err:
        /* clean up handle instance */
        free(p_handle);
        return ret;
}

esp_err_t vector_int16_get_rotated(vector_int16_handle_t vector_int16_handle, quaternion_handle_t quaternion_handle, vector_int16_handle_t *rotated_handle) {
    esp_err_t               ret = ESP_OK;
    vector_int16_handle_t   out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( vector_int16_handle );

    /* attempt vector int16 init */
    ESP_GOTO_ON_ERROR(vector_int16_init(&out_handle), err, TAG, "Vector int16 init for get rotated failed");

    /* attempt vector int16 init data */
    ESP_GOTO_ON_ERROR(vector_int16_init_data(out_handle, vector_int16_handle->x, vector_int16_handle->y, vector_int16_handle->z), 
        err, TAG, "Vector int16 init data for get rotated failed");

    /* attempt vector int16 rotate */
    ESP_GOTO_ON_ERROR(vector_int16_rotate(out_handle, quaternion_handle), err, TAG, "Vector int16 rotate for get rotated failed");

    /* set handle */
    *rotated_handle = out_handle;

    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}