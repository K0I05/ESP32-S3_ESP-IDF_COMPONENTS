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
 * @file quaternion.c
 *
 * ESP-IDF math 3d library
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "include/quaternion.h"
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

esp_err_t quaternion_init(quaternion_handle_t *quaternion_handle) {
    esp_err_t           ret = ESP_OK;
    quaternion_handle_t out_handle;

    /* validate memory availability for handle */
    out_handle = (quaternion_handle_t)calloc(1, sizeof(quaternion_t));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for quaternion handle, init failed");

    /* set handle parameters */
    out_handle->w = 1.0f;
    out_handle->x = 0.0f;
    out_handle->y = 0.0f;
    out_handle->z = 0.0f;

    /* set handle */
    *quaternion_handle = out_handle;

    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}

esp_err_t quaternion_init_data(quaternion_handle_t quaternion_handle, float nw, float nx, float ny, float nz) {
    /* validate arguments */
    ESP_ARG_CHECK( quaternion_handle );

    /* set handle parameters */
    quaternion_handle->w = nw;
    quaternion_handle->x = nx;
    quaternion_handle->y = ny;
    quaternion_handle->z = nz;

    return ESP_OK;
}

esp_err_t quaternion_get_product(quaternion_handle_t quaternion_handle, quaternion_handle_t *product_handle) {
    esp_err_t           ret = ESP_OK;
    quaternion_handle_t out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( quaternion_handle );

    /* attempt quaternion init */
    ESP_GOTO_ON_ERROR(quaternion_init(&out_handle), err, TAG, "Quarternion init for get product failed");

    // attempt quaternion init data multiplication as defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
    ESP_GOTO_ON_ERROR(quaternion_init_data(out_handle,
        quaternion_handle->w*out_handle->w - quaternion_handle->x*out_handle->x - quaternion_handle->y*out_handle->y - quaternion_handle->z*out_handle->z,  // new w
        quaternion_handle->w*out_handle->x + quaternion_handle->x*out_handle->w + quaternion_handle->y*out_handle->z - quaternion_handle->z*out_handle->y,  // new x
        quaternion_handle->w*out_handle->y - quaternion_handle->x*out_handle->z + quaternion_handle->y*out_handle->w + quaternion_handle->z*out_handle->x,  // new y
        quaternion_handle->w*out_handle->z + quaternion_handle->x*out_handle->y - quaternion_handle->y*out_handle->x + quaternion_handle->z*out_handle->w), // new z
        err, TAG, "Quarternion init data for get product failed");

    /* set handle */
    *product_handle = out_handle;

    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}

esp_err_t quaternion_get_conjugate(quaternion_handle_t quaternion_handle, quaternion_handle_t *conjugate_handle) {
    esp_err_t           ret = ESP_OK;
    quaternion_handle_t out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( quaternion_handle );

    /* attempt quaternion init */
    ESP_GOTO_ON_ERROR(quaternion_init(&out_handle), err, TAG, "Quarternion init for get conjugate failed");

    /* attempt quaternion init data */
    ESP_GOTO_ON_ERROR(quaternion_init_data(out_handle, quaternion_handle->w, -quaternion_handle->x, -quaternion_handle->y, -quaternion_handle->z), 
        err, TAG, "Quarternion init data for get conjugate failed");

    /* set handle */
    *conjugate_handle = out_handle;
    
    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}

esp_err_t quaternion_get_magnitude(quaternion_handle_t quaternion_handle, float *magnitude) {
    /* validate arguments */
    ESP_ARG_CHECK( quaternion_handle );

    /* compute quaternion magnitude */
    *magnitude = sqrt(quaternion_handle->w*quaternion_handle->w 
                + quaternion_handle->x*quaternion_handle->x 
                + quaternion_handle->y*quaternion_handle->y 
                + quaternion_handle->z*quaternion_handle->z);

    return ESP_OK;
}

esp_err_t quaternion_normalize(quaternion_handle_t quaternion_handle) {
    float m;

    /* validate arguments */
    ESP_ARG_CHECK( quaternion_handle );
    
    /* attempt quaternion magnitude */
    ESP_RETURN_ON_ERROR(quaternion_get_magnitude(quaternion_handle, &m), TAG, "Quarternion get magnitude for normalize failed");

    /* set handle parameters */
    quaternion_handle->w /= m;
    quaternion_handle->x /= m;
    quaternion_handle->y /= m;
    quaternion_handle->z /= m;

    return ESP_OK;
}

esp_err_t quaternion_get_normalized(quaternion_handle_t quaternion_handle, quaternion_handle_t *normalized_handle) {
    esp_err_t           ret = ESP_OK;
    quaternion_handle_t out_handle;

    /* validate arguments */
    ESP_ARG_CHECK( quaternion_handle );

    /* attempt quaternion init */
    ESP_GOTO_ON_ERROR(quaternion_init(&out_handle), err, TAG, "Quarternion init for get normalized failed");

    /* attempt quaternion init data */
    ESP_GOTO_ON_ERROR(quaternion_init_data(out_handle, quaternion_handle->w, quaternion_handle->x, quaternion_handle->y, quaternion_handle->z), 
        err, TAG, "Quarternion init data for get normalized failed");

    /* attempt quaternion normalize */
    ESP_GOTO_ON_ERROR(quaternion_normalize(out_handle), err, TAG, "Quarternion normalize for get normalized failed");

    /* set handle */
    *normalized_handle = out_handle;
    
    return ESP_OK;

    err:
        /* clean up handle instance */
        free(out_handle);
        return ret;
}