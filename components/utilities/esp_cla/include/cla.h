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
 * @file cla.h
 * @defgroup 
 * @{
 *
 * ESP-IDF compact linear algebra (cla) library
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CLA_H__
#define __CLA_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>

#include "cla_version.h"
#include "matrix.h"
#include "vector.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */
#define CLA_3X3_MATRIX_SIZE                UINT8_C(3)
#define CLA_9X9_MATRIX_SIZE                UINT8_C(9)
#define CLA_ELLIPSOID_COEFF_SIZE           UINT8_C(9)
#define CLA_CAL_SAMPLE_SIZE                UINT8_C(250)

/**
 * public macro definitions
 */


/**
 * public enumerator, union, and structure definitions
 */
typedef cla_vector_ptr_t cla_ellipsoid_coeffs_t;

typedef cla_vector_ptr_t cla_vector_samples_t[CLA_CAL_SAMPLE_SIZE];

/**
 * @brief Checks the quality of magnetometer calibration sample data.
 * 
 * Evaluates:
 * 1. Data range coverage (min/max values per axis)
 * 2. Data distribution (variance per axis)
 * 3. Sample uniqueness (duplicate detection)
 * 4. Sphericity check (expected field strength variation)
 * 
 * @param v_calib_data Array of calibration sample vectors
 * @param expected_field_strength Expected magnetic field magnitude (optional, use 0 to skip)
 * @param quality_report Output structure with quality metrics
 * @return esp_err_t ESP_OK on success
 */
typedef struct cla_calibration_quality_s {
    double min_x, max_x, range_x;
    double min_y, max_y, range_y;
    double min_z, max_z, range_z;
    double variance_x, variance_y, variance_z;
    double mean_magnitude;
    double magnitude_std_dev;
    uint16_t duplicate_count;
    uint16_t unique_count;
    bool has_good_coverage;      // All axes have sufficient range
    bool has_good_distribution;  // Data is well-distributed
    bool has_good_uniqueness;    // Low duplicate count
    uint8_t overall_quality;     // 0-100 score
} cla_calibration_quality_t;

/* -------------------- First-order IIR low-pass filter -------------------- */
typedef struct cla_iir_lowpass_filter_s {
    double alpha;       /* filter coefficient (0..1) */
    double previous;    /* previous output */
    bool   initialized; /* flag to initialize previous on first sample */
} cla_iir_lowpass_filter_t;

/* -------------------- Moving-average (FIR) low-pass filter -------------------- */
typedef struct cla_fir_lowpass_filter_moving_average_s {
    double    *buffer;      /* circular buffer */
    uint32_t   window_len;  /* window length */
    uint32_t   idx;         /* current insertion index */
    double     sum;         /* running sum */
    uint32_t   filled;      /* number of samples filled (<= window_len) */
} cla_fir_lowpass_filter_moving_average_t;

/**
 * public function & subroutine prototype definitions
 */

/**
 * @brief Multiplies a matrix by a vector.
 * 
 * @param m Matrix to multiply.
 * @param v Vector to multiply.
 * @param v_product Vector product of matrix and vector. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_multiply_vector(const cla_matrix_ptr_t m, const cla_vector_ptr_t v, cla_vector_ptr_t *const v_product);

 /**
  * @brief Converts n-1 dimensional matrix to n dimensional vector.
  * 
  * @param m Matrix to convert.
  * @param v Vector result of the conversion.
  * @return esp_err_t ESP_OK on success.
  */
esp_err_t cla_matrix_to_vector(const cla_matrix_ptr_t m, cla_vector_ptr_t *const v);

/**
 * @brief Converts n dimensional vector to n-1 dimensional matrix.
 * 
 * @param v Vector to convert.
 * @param m Matrix result of the conversion.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_vector_to_matrix(const cla_vector_ptr_t v, cla_matrix_ptr_t *const m);

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
 * @brief Calculates ellipsoid coefficients from calibration sample data.
 * 
 * @param v_calib_data Calibration sample data array.
 * @param v_ellip_coeffs Ellipsoid coefficients vector (9x1).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_solve_ellipsoid_coefficients(const cla_vector_samples_t v_calib_data, cla_ellipsoid_coeffs_t *const v_ellip_coeffs);

/**
 * @brief Derives hard and soft-iron calibration parameters from ellipsoid coefficients.
 *
 * @param v_ellip_coeffs Input vector of 9 ellipsoid coefficients.
 * @param v_offset       Output hard-iron offset vector (3x1).
 * @param m_w            Output soft-iron correction matrix (3x3).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_get_calibration_parameters(const cla_vector_ptr_t v_ellip_coeffs, cla_vector_ptr_t *const v_offset, cla_matrix_ptr_t *const m_w);

/**
 * @brief Applies hard and soft-iron calibration parameters to raw vector sensor data.
 *
 * @param v_raw_data Input raw sensor data vector (3x1).
 * @param v_offset   Hard-iron offset vector (3x1).
 * @param m_w        Soft-iron correction matrix (3x3).
 * @param v_cal_data Output calibrated data vector (3x1).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_apply_calibration(const cla_vector_ptr_t v_raw_data, const cla_vector_ptr_t v_offset, const cla_matrix_ptr_t m_w, cla_vector_ptr_t *const v_cal_data);

/**
 * @brief Checks the quality of magnetometer calibration sample data.
 * 
 * @param v_calib_data Array of calibration sample vectors
 * @param expected_field_strength Expected magnetic field magnitude (optional, use 0 to skip)
 * @param quality_report Output structure with quality metrics
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_get_calibration_samples_quality(const cla_vector_samples_t v_calib_data, const double expected_field_strength, cla_calibration_quality_t *quality_report);

/**
 * @brief Prints the quality metrics of magnetometer calibration sample data.
 * 
 * @param quality_report Structure containing quality metrics to print.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_calibration_samples_quality_print(const cla_calibration_quality_t quality_report);

/**
 * @brief Initializes a first-order IIR low-pass filter IIR filter that uses a continuous-time RC 
 * approximation with specified sampling and cutoff frequencies.
 * 
 * @param filter Pointer to the IIR low-pass filter structure to initialize.
 * @param fs Sampling frequency in Hz.
 * @param fc Cutoff frequency in Hz.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_iir_lowpass_init_cutoff(cla_iir_lowpass_filter_t *const filter, const double fs, const double fc);

/**
 * @brief Initializes a first-order IIR low-pass filter that uses a continuous-time RC 
 * approximation with specified alpha coefficient.
 * 
 * @param filter Pointer to the IIR low-pass filter structure to initialize.
 * @param alpha Filter coefficient (0 < alpha < 1).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_iir_lowpass_init_alpha(cla_iir_lowpass_filter_t *const filter, const double alpha);

/**
 * @brief Applies the first-order IIR low-pass filter that uses a continuous-time RC 
 * approximation to an input sample.
 * 
 * @param filter Pointer to the IIR low-pass filter structure.
 * @param x Input sample to filter.
 * @param fv Pointer to store the filtered output value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_iir_lowpass_apply(cla_iir_lowpass_filter_t *const filter, const double x, double *const fv);

/**
 * @brief Initializes a moving average FIR low-pass filter with a specified window length.
 * 
 * @param ma_filter Pointer to the moving average filter structure to initialize.
 * @param window_len Length of the moving average window.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_fir_lowpass_moving_average_init(cla_fir_lowpass_filter_moving_average_t *const ma_filter, const uint32_t window_len);

/**
 * @brief Deletes a moving average FIR low-pass filter, freeing allocated resources.
 * 
 * @param ma_filter Pointer to the moving average filter structure to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_fir_lowpass_moving_average_delete(cla_fir_lowpass_filter_moving_average_t *const ma_filter);

/**
 * @brief Applies the moving average FIR low-pass filter to an input sample.
 * 
 * @param ma_filter Pointer to the moving average filter structure.
 * @param x Input sample to filter.
 * @param fv Pointer to store the filtered output value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_fir_lowpass_moving_average_apply(cla_fir_lowpass_filter_moving_average_t *const ma_filter, const double x, double *const fv);

/**
 * @brief Calculates the median of an array of double-precision floating-point values using sorting.
 * 
 * @param data Input array of double values.
 * @param n Number of elements in the input array.
 * @param median Pointer to store the calculated median value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_get_median_by_sort(const double *data, const uint16_t n, double *const median);

/**
 * @brief Calculates the median of an array of double-precision floating-point values using the Quickselect algorithm 
 * that does not modify the input array.
 * 
 * @param data Input array of double values.
 * @param n Number of elements in the input array.
 * @param median Pointer to store the calculated median value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_get_median_by_quickselect(const double *data, const uint16_t n, double *const median);

/**
 * @brief Clamps a double-precision floating-point value within specified minimum and maximum bounds.  A 
 * value outside the minimum or maximum bounds will be set the respective bound.  If the minimum bound is
 * not set, there is no lower bound.  If the maximum bound is not set, there is no upper bound.
 * 
 * @param min The lower bound of the range.
 * @param max The upper bound of the range.
 * @param ignore_out_of_range Ignores values outside the specified range when set to true.
 * @param value The value to clamp.
 * @return double Clamped value.
 */
double cla_clamp_value(const double min, const double max, const bool ignore_out_of_range, const double value);

/**
 * @brief Converts `cla` firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* `cla` firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* cla_get_fw_version(void);

/**
 * @brief Converts `cla` firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t `cla` firmware version number.
 */
int32_t cla_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CLA_H__
