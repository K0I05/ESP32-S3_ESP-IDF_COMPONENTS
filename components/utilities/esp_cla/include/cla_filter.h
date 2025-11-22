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
 * @file cla_filter.h
 * @defgroup 
 * @{
 *
 * ESP-IDF compact linear algebra (cla) library
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CLA_FILTER_H__
#define __CLA_FILTER_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_check.h>

#include "cla_common.h"


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


/* -------------------- First-order IIR low-pass filter -------------------- */
typedef struct cla_filter_iir_lowpass_s {
    double alpha;       /* filter coefficient (0..1) */
    double previous;    /* previous output */
    bool   initialized; /* flag to initialize previous on first sample */
} cla_filter_iir_lowpass_t;

/* -------------------- Moving-average (FIR) low-pass filter -------------------- */
typedef struct cla_filter_fir_lowpass_moving_average_s {
    double    *buffer;      /* circular buffer */
    uint32_t   window_len;  /* window length */
    uint32_t   idx;         /* current insertion index */
    double     sum;         /* running sum */
    uint32_t   filled;      /* number of samples filled (<= window_len) */
} cla_filter_fir_lowpass_moving_average_t;

/**
 * public function & subroutine prototype definitions
 */



/**
 * @brief Initializes a first-order IIR low-pass filter IIR filter that uses a continuous-time RC 
 * approximation with specified sampling and cutoff frequencies.
 * 
 * @param filter Pointer to the IIR low-pass filter structure to initialize.
 * @param fs Sampling frequency in Hz.
 * @param fc Cutoff frequency in Hz.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_iir_lowpass_init_cutoff(cla_filter_iir_lowpass_t *const filter, const double fs, const double fc);

/**
 * @brief Initializes a first-order IIR low-pass filter that uses a continuous-time RC 
 * approximation with specified alpha coefficient.
 * 
 * @param filter Pointer to the IIR low-pass filter structure to initialize.
 * @param alpha Filter coefficient (0 < alpha < 1).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_iir_lowpass_init_alpha(cla_filter_iir_lowpass_t *const filter, const double alpha);

/**
 * @brief Applies the first-order IIR low-pass filter that uses a continuous-time RC 
 * approximation to an input sample.
 * 
 * @param filter Pointer to the IIR low-pass filter structure.
 * @param x Input sample to filter.
 * @param fv Pointer to store the filtered output value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_iir_lowpass_apply(cla_filter_iir_lowpass_t *const filter, const double x, double *const fv);

/**
 * @brief Initializes a moving average FIR low-pass filter with a specified window length.
 * 
 * @param ma_filter Pointer to the moving average filter structure to initialize.
 * @param window_len Length of the moving average window.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_fir_lowpass_moving_average_init(cla_filter_fir_lowpass_moving_average_t *const ma_filter, const uint32_t window_len);

/**
 * @brief Deletes a moving average FIR low-pass filter, freeing allocated resources.
 * 
 * @param ma_filter Pointer to the moving average filter structure to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_fir_lowpass_moving_average_delete(cla_filter_fir_lowpass_moving_average_t *const ma_filter);

/**
 * @brief Applies the moving average FIR low-pass filter to an input sample.
 * 
 * @param ma_filter Pointer to the moving average filter structure.
 * @param x Input sample to filter.
 * @param fv Pointer to store the filtered output value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_fir_lowpass_moving_average_apply(cla_filter_fir_lowpass_moving_average_t *const ma_filter, const double x, double *const fv);

/**
 * @brief Calculates the median of an array of double-precision floating-point values using sorting.
 * 
 * @param data Input array of double values.
 * @param n Number of elements in the input array.
 * @param median Pointer to store the calculated median value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_get_median_by_sort(const double *data, const uint16_t n, double *const median);

/**
 * @brief Calculates the median of an array of double-precision floating-point values using the Quickselect algorithm 
 * that does not modify the input array.
 * 
 * @param data Input array of double values.
 * @param n Number of elements in the input array.
 * @param median Pointer to store the calculated median value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_filter_get_median_by_quickselect(const double *data, const uint16_t n, double *const median);

/**
 * @brief Clamps a double-precision floating-point value within specified `minimum` and `maximum` bounds.  The 
 * `minimum` bound must be smaller than the `maximum` bound, otherwise, the bounds will be swapped.  A value 
 * outside the `minimum` or `maximum` bounds will be set to the respective bound.  If the `minimum` bound 
 * is not set, there is no lower bound.  If the `maximum` bound is not set, there is no upper bound.  If the 
 * `ignore_out_of_range` flag is set to true, values outside the specified range will be returned as `NAN`.
 * 
 * @param min The lower bound of the range.
 * @param max The upper bound of the range.
 * @param ignore_out_of_range Ignores values outside the specified range when set to true.
 * @param value The value to clamp.
 * @return double The results of the clamped value.
 */
double cla_filter_clamp(const double min, const double max, const bool ignore_out_of_range, const double value);

/**
 * @brief Computes the discrete Fourier transform (DFT) of real and imaginary input arrays using the FFT algorithm.
 * 
 * @param real Pointer to the array of real parts of the input signal.
 * @param imag Pointer to the array of imaginary parts of the input signal.
 * @param n Number of elements in the input arrays (must be a power of two).
 * @param dft Pointer to an array to store the DFT results (can be NULL if not needed).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if n is not a power of two.
 */
esp_err_t cla_filter_get_fft_dft(double *real, double *imag, const uint16_t n, int16_t *const dft);

/**
 * @brief Computes the inverse discrete Fourier transform (IDFT) of real and imaginary input arrays using the FFT algorithm.
 * 
 * @param real Pointer to the array of real parts of the input signal.
 * @param imag Pointer to the array of imaginary parts of the input signal.
 * @param n Number of elements in the input arrays (must be a power of two).
 * @param dft Pointer to an array to store the IDFT results (can be NULL if not needed).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if n is not a power of two.
 */
esp_err_t cla_filter_get_fft_idft(double *real, double *imag, const uint16_t n, int16_t *const dft);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CLA_FILTER_H__
