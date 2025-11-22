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
 * @file cla_common.c
 *
 * ESP-IDF compact linear algebra (cla) library
 * 
 * https://github.com/nomemory/neat-matrix-library/blob/main/nml.c
 * 
 * 
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <math.h>
#include <float.h>
#include "include/cla_common.h"

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "cla_common";



void cla_swap_values(double *const val1, double *const val2) {
    double temp = *val1;
    *val1 = *val2;
    *val2 = temp;
}

esp_err_t cla_copy_buffer(const void *pa, const void *pb) {
    double a = *(const double*)pa;
    double b = *(const double*)pb;
    if (a < b) return ESP_ERR_INVALID_SIZE;
    if (a > b) return ESP_ERR_INVALID_SIZE;
    return ESP_OK;
}

bool cla_is_value_equal(const double val1, const double val2, const double tolerance) {
    const double diff = fabs(val1 - val2);
    return diff <= tolerance || diff < fmax(fabs(val1), fabs(val2)) * tolerance;
}

bool cla_is_value_power_of_two(const uint16_t val) {
    return val > 0 && (val & (val - 1)) == 0;
}

float cla_get_heading(const float x_axis, const float y_axis) {
    float heading;
    /* honeywell application note AN-203 */
    if(y_axis > 0.0f) { 
        heading = 90.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else if(y_axis < 0.0f) {
        heading = 270.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else {
        if(x_axis < 0.0f) { 
            heading = 180.0f;
        } else if(x_axis > 0.0f) { 
            heading = 0.0f;
        } else {
            heading = 0.0f;
        }
    }
    /* convert to heading to a 0..360 degree range */
    if (heading < 0.0f) {
        heading += 360.0f;
    } else if (heading > 360.0f) {
        heading -= 360.0f;
    }
    return heading;
}

float cla_get_true_heading(const float x_axis, const float y_axis, const float declination) {
    float heading;
    /* honeywell application note AN-203 */
    if(y_axis > 0.0f) { 
        heading = 90.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else if(y_axis < 0.0f) {
        heading = 270.0f - (atanf(x_axis/y_axis) * 180.0f / M_PI);
    } else {
        if(x_axis < 0.0f) { 
            heading = 180.0f;
        } else if(x_axis > 0.0f) { 
            heading = 0.0f;
        } else {
            heading = 0.0f;
        }
    }
    /* apply magnetic declination (+east | -west) */
    heading += declination;
    /* convert to heading to a 0..360 degree range */
    if (heading < 0.0f) {
        heading += 360.0f;
    } else if (heading > 360.0f) {
        heading -= 360.0f;
    }
    return heading;
}

float cla_get_earth_field(const float x_axis, const float y_axis, const float z_axis) {
    return sqrtf(powf(x_axis, 2) + powf(y_axis, 2) + powf(z_axis, 2));
}
