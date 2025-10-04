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
 * @file kalman_motion.c
 *
 * ESP-IDF kalman motion filter library
 * 
 * https://github.com/SMotlaq/open-watch/blob/master/open-watch-firmware/my_watch_software/MyDrivers/KalmanFilter/Kalman.c
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "kalman_motion.h"
#include <string.h>
#include <stdio.h>
#include <esp_check.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "kalman_motion";


esp_err_t kalman_motion_init(kalman_motion_handle_t *kalman_handle) {
    /* validate memory availability for handle */
    kalman_motion_handle_t out_handle = (kalman_motion_handle_t)calloc(1, sizeof(kalman_motion_t));
    ESP_RETURN_ON_FALSE(out_handle, ESP_ERR_NO_MEM, TAG, "no memory for kalman handle, init failed");

    /* We will set the variables like so, these can also be tuned by the user */
    out_handle->q_angle   = 0.001f;
    out_handle->q_bias    = 0.003f;
    out_handle->r_measure = 0.03f;

    out_handle->angle     = 0.0f; // Reset the angle
    out_handle->bias      = 0.0f; // Reset bias

    out_handle->p[0][0]   = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    out_handle->p[0][1]   = 0.0f;
    out_handle->p[1][0]   = 0.0f;
    out_handle->p[1][1]   = 0.0f;

    /* set handle */
    *kalman_handle = out_handle;

    return ESP_OK;
}

esp_err_t kalman_motion_get_angle(kalman_motion_handle_t kalman_handle, const float new_angle, const float new_rate, const float delta_time, float *const angle) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    kalman_handle->rate = new_rate - kalman_handle->bias;
    kalman_handle->angle += delta_time * kalman_handle->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    kalman_handle->p[0][0] += delta_time * (delta_time*kalman_handle->p[1][1] - kalman_handle->p[0][1] - kalman_handle->p[1][0] + kalman_handle->q_angle);
    kalman_handle->p[0][1] -= delta_time * kalman_handle->p[1][1];
    kalman_handle->p[1][0] -= delta_time * kalman_handle->p[1][1];
    kalman_handle->p[1][1] += kalman_handle->q_bias * delta_time;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = new_angle - kalman_handle->angle; // Angle difference

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float s = kalman_handle->p[0][0] + kalman_handle->r_measure; // Estimate error

    /* Step 5 */
    float k[2]; // Kalman gain - This is a 2x1 vector
    k[0] = kalman_handle->p[0][0] / s;
    k[1] = kalman_handle->p[1][0] / s;

    /* Step 6 */
    kalman_handle->angle += k[0] * y;
    kalman_handle->bias += k[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float p00_temp = kalman_handle->p[0][0];
    float p01_temp = kalman_handle->p[0][1];

    kalman_handle->p[0][0] -= k[0] * p00_temp;
    kalman_handle->p[0][1] -= k[0] * p01_temp;
    kalman_handle->p[1][0] -= k[1] * p00_temp;
    kalman_handle->p[1][1] -= k[1] * p01_temp;

    *angle = kalman_handle->angle;

    return ESP_OK;
}

esp_err_t kalman_motion_set_angle(kalman_motion_handle_t kalman_handle, const float angle) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    kalman_handle->angle = angle;

    return ESP_OK;
}

esp_err_t kalman_motion_get_rate(kalman_motion_handle_t kalman_handle, float *const rate) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    *rate = kalman_handle->rate;

    return ESP_OK;
}

esp_err_t kalman_motion_set_q_angle(kalman_motion_handle_t kalman_handle, const float q_angle) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    kalman_handle->q_angle = q_angle;

    return ESP_OK;
}

esp_err_t kalman_motion_set_q_bias(kalman_motion_handle_t kalman_handle, const float q_bias) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    kalman_handle->q_bias = q_bias;

    return ESP_OK;
}

esp_err_t kalman_motion_set_r_measure(kalman_motion_handle_t kalman_handle, const float r_measure) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    kalman_handle->r_measure = r_measure;

    return ESP_OK;
}

esp_err_t kalman_motion_get_q_angle(kalman_motion_handle_t kalman_handle, float *const q_angle) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    *q_angle = kalman_handle->q_angle;

    return ESP_OK;
}

esp_err_t kalman_motion_get_q_bias(kalman_motion_handle_t kalman_handle, float *const q_bias) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    *q_bias = kalman_handle->q_bias;

    return ESP_OK;
}

esp_err_t kalman_motion_get_r_measure(kalman_motion_handle_t kalman_handle, float *const r_measure) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    *r_measure = kalman_handle->r_measure;

    return ESP_OK;
}

esp_err_t kalman_motion_delete(kalman_motion_handle_t kalman_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( kalman_handle );

    if(kalman_handle) free(kalman_handle);

    return ESP_OK;
}