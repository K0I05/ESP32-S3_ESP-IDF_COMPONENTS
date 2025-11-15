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
 * @file kalman_motion.h
 * @defgroup math kalman motion
 * @{
 *
 * ESP-IDF kalman motion filter library
 * 
 * Ported from esp-open-rtos
 * 
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 * https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __KALMAN_MOTION_H__
#define __KALMAN_MOTION_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Kalman motion state structure.
 */
struct kalman_motion_t {
    float q_angle;     // Process noise variance for the accelerometer
    float q_bias;      // Process noise variance for the gyro bias
    float r_measure;   // Measurement noise variance - this is actually the variance of the measurement noise
    float angle;       // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;        // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;        // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    float p[2][2];     // Error covariance matrix - This is a 2x2 matrix
};

/**
 * @brief Kalman motion definition.
 */
typedef struct kalman_motion_t kalman_motion_t;

/**
 * @brief Kalman motion handle definition.
 */
typedef struct kalman_motion_t *kalman_motion_handle_t;

/**
 * @brief Initializes a kalman motion handle instance to filter data from a gyroscope and/or accelerometer.
 * 
 * @param[out] kalman_handle Kalman motion state handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_init(kalman_motion_handle_t *kalman_handle);

/**
 * @brief Calculates angle using a Kalman motion filter.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[in] new_angle New angle in degrees to process.
 * @param[in] new_rate New rate in degrees per second to process.
 * @param[in] delta_time Delta time in seconds.
 * @param[out] angle Calculated angle using Kalman filter.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_get_angle(kalman_motion_handle_t kalman_handle, const float new_angle, const float new_rate, const float delta_time, float *const angle);

/**
 * @brief Sets the angle and should be used to set the starting angle.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[in] angle Angle in degrees.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_set_angle(kalman_motion_handle_t kalman_handle, const float angle);

/**
 * @brief Gets the unbiased rate calculated from the rate and the calculated bias.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[out] rate Rate in degrees per second.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_get_rate(kalman_motion_handle_t kalman_handle, float *const rate);

/**
 * @brief Sets q_angle noise variance for the accelerometer.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[in] q_angle Angle noise variance in degrees.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_set_q_angle(kalman_motion_handle_t kalman_handle, const float q_angle);

/**
 * @brief Sets q_bias noise variance for the gyroscope.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[in] q_bias Bias noise variance.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_set_q_bias(kalman_motion_handle_t kalman_handle, const float q_bias);

/**
 * @brief Sets r_measure noise variance for the measurement.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[in] r_measure Measurement noise variance.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_set_r_measure(kalman_motion_handle_t kalman_handle, const float r_measure);

/**
 * @brief Gets q_angle noise variance for the accelerometer.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[out] q_angle Angle noise variance in degrees.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_get_q_angle(kalman_motion_handle_t kalman_handle, float *const q_angle);

/**
 * @brief Gets q_bias noise variance for the gyroscope.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[out] q_bias Bias noise variance.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_get_q_bias(kalman_motion_handle_t kalman_handle, float *const q_bias);

/**
 * @brief Gets r_measure noise variance for the measurement.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @param[out] r_measure Measurement noise variance.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_get_r_measure(kalman_motion_handle_t kalman_handle, float *const r_measure);

/**
 * @brief Frees Kalman motion handle.
 * 
 * @param[in] kalman_handle Kalman motion state handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t kalman_motion_delete(kalman_motion_handle_t kalman_handle);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __KALMAN_MOTION_H__
