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
 * @file cla_filter.c
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
#include "include/cla_filter.h"

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "cla_filter";





/* In-place bit-reversal permutation for arrays real[], imag[] of length n (n power of two) */
static inline void cla_filter_bit_reversal_permutation(double *real, double *imag, const uint16_t n) {
    uint16_t i, j, k;
    j = 0;
    for (i = 1; i < n; ++i) {
        k = n >> 1;
        while (j & k) {
            j ^= k;
            k >>= 1;
        }
        j ^= k;
        if (i < j) {
            const double tr = real[i];
            const double ti = imag[i];
            real[i] = real[j];
            imag[i] = imag[j];
            real[j] = tr;
            imag[j] = ti;
        }
    }
}

/* Forward/inverse FFT core. inverse == false => forward FFT, inverse == true => inverse FFT.
 * Returns ESP_OK on success, ESP_ERR_INVALID_ARG on invalid n.
 */
static inline esp_err_t cla_filter_fft_core(double *real, double *imag, const uint16_t n, const bool inverse) {
    if (!cla_is_value_power_of_two(n)) return ESP_ERR_INVALID_ARG;

    cla_filter_bit_reversal_permutation(real, imag, n);

    for (int len = 2; len <= n; len <<= 1) {
        const double ang = 2.0 * M_PI / (double)len * (inverse ? 1.0 : -1.0); /* sign: - for forward, + for inverse */
        const double wlen_cos = cos(ang);
        const double wlen_sin = sin(ang);

        for (int i = 0; i < n; i += len) {
            double wr = 1.0;
            double wi = 0.0;
            const int half = len >> 1;
            for (int j = 0; j < half; ++j) {
                const int idx1 = i + j;
                const int idx2 = i + j + half;
                const double ur = real[idx1];
                const double ui = imag[idx1];
                const double vr = real[idx2] * wr - imag[idx2] * wi;
                const double vi = real[idx2] * wi + imag[idx2] * wr;

                real[idx1] = ur + vr;
                imag[idx1] = ui + vi;
                real[idx2] = ur - vr;
                imag[idx2] = ui - vi;

                /* multiply wr+ i wi by wlen_cos + i wlen_sin */
                const double tmp = wr * wlen_cos - wi * wlen_sin;
                wi = wr * wlen_sin + wi * wlen_cos;
                wr = tmp;
            }
        }
    }

    return ESP_OK;
}


/* ---------- Quickselect (nth smallest) ----------
 * In-place selection algorithm (Hoare/ Lomuto style, randomized pivot avoided by median-of-three).
 * Returns the k-th smallest element (0-based). Requires n > 0 and k < n.
 */
static inline uint16_t cla_filter_get_median_of_three_index(const double *a, const uint16_t left, const uint16_t right) {
    const uint16_t mid = left + ((right - left) >> 1);
    const double al = a[left], am = a[mid], ar = a[right];

    /* Nested simple conditionals to reduce cyclomatic complexity */
    if (al < am) {
        if (am < ar) {
            return mid;
        } else {
            if (al < ar) {
                return right;
            } else {
                return left;
            }
        }
    } else { /* al >= am */
        if (al < ar) {
            return left;
        } else {
            if (am < ar) {
                return right;
            } else {
                return mid;
            }
        }
    }
}

/* helper to select pivot (median-of-three) and partition; returns final pivot index */
static inline uint16_t cla_filter_select_pivot_and_partition(double *a, const uint16_t left, const uint16_t right) {
    const uint16_t pidx  = cla_filter_get_median_of_three_index(a, left, right);
    const double   pivot = a[pidx];
    cla_swap_values(&a[pidx], &a[right]); /* move pivot to end */
    uint16_t store = left;
    for (uint16_t i = left; i < right; ++i) {
        if (a[i] < pivot) {
            cla_swap_values(&a[store], &a[i]);
            store++;
        }
    }
    cla_swap_values(&a[store], &a[right]); /* place pivot */
    return store;
}

static inline esp_err_t cla_filter_median_quickselect_kth(double *a, const uint16_t n, const uint16_t k, double *const v) {
    if (n == 0 || k >= n) return ESP_ERR_INVALID_ARG;
    uint16_t left = 0, right = n - 1;
    while (left <= right) {
        if (left == right) {
            *v = a[left];
            return ESP_OK;
        }
        const uint16_t store = cla_filter_select_pivot_and_partition(a, left, right);
        if (k == store) {
            *v = a[store];
            return ESP_OK;
        } else if (k < store) {
            if (store == 0) break;
            right = store - 1;
        } else {
            left = store + 1;
        }
    }
    return ESP_ERR_INVALID_STATE;
}

esp_err_t cla_filter_get_median_by_sort(const double *data, const uint16_t n, double *const median) {
    if (n == 0) return ESP_ERR_INVALID_ARG;
    double *buf = (double*)malloc(n * sizeof(double));
    if (!buf) return ESP_ERR_NO_MEM;
    memcpy(buf, data, n * sizeof(double));
    qsort(buf, n, sizeof(double), cla_copy_buffer);
    double med;
    if (n & 1) {
        med = buf[n/2];
    } else {
        med = 0.5 * (buf[n/2 - 1] + buf[n/2]);
    }
    free(buf);
    *median = med;
    return ESP_OK;
}

esp_err_t cla_filter_get_median_by_quickselect(const double *data, const uint16_t n, double *const median) {
    if (n == 0) return ESP_ERR_INVALID_ARG;
    double *buf = (double*)malloc(n * sizeof(double));
    if (!buf) return ESP_ERR_NO_MEM;
    memcpy(buf, data, n * sizeof(double));
    double med;
    if (n & 1) {
        cla_filter_median_quickselect_kth(buf, n, n/2, &med);
    } else {
        /* need the two middle values: find lower middle and upper middle */
        double v1, v2;
        cla_filter_median_quickselect_kth(buf, n, n/2 - 1, &v1);
        cla_filter_median_quickselect_kth(buf, n, n/2, &v2);
        med = 0.5 * (v1 + v2);
    }
    free(buf);
    *median = med;
    return ESP_OK;
}

/* helper to prepare bounds and ensure order */
static inline void cla_filter_prepare_bounds(double *const min_in, double *const max_in, bool *const has_min, bool *const has_max) {
    *has_min = !isnan(*min_in);
    *has_max = !isnan(*max_in);
    if (*has_min && *has_max && *min_in > *max_in) {
        double t = *min_in;
        *min_in = *max_in;
        *max_in = t;
    }
}

double cla_filter_clamp(const double min, const double max, const bool ignore_out_of_range, const double value) {
    if (isnan(value)) return NAN;

    double min_in = min;
    double max_in = max;
    bool has_min, has_max;
    cla_filter_prepare_bounds(&min_in, &max_in, &has_min, &has_max);

    double v = value;

    if (has_min && v < min_in) {
        if (ignore_out_of_range) return NAN;
        v = min_in;
    } else if (has_max && v > max_in) {
        if (ignore_out_of_range) return NAN;
        v = max_in;
    }

    return v;
}


esp_err_t cla_filter_iir_lowpass_init_cutoff(cla_filter_iir_lowpass_t *const filter, const double fs, const double fc) {
    // Validate arguments
    ESP_ARG_CHECK(filter);

    /* calculate alpha from cutoff frequency and sampling frequency */
    if (fs <= 0.0 || fc <= 0.0) {
        /* fallback to identity filter */
        filter->alpha = 1.0;
    } else {
        const double dt = 1.0 / fs;
        const double tau = 1.0 / (2.0 * M_PI * fc); /* RC time constant equivalent */
        filter->alpha = dt / (tau + dt);        /* alpha in (0,1) */
        if (filter->alpha < 0.0) filter->alpha = 0.0;
        if (filter->alpha > 1.0) filter->alpha = 1.0;
    }

    filter->previous    = 0.0;
    filter->initialized = false;

    return ESP_OK;
}

esp_err_t cla_filter_iir_lowpass_init_alpha(cla_filter_iir_lowpass_t *const filter, const double alpha) {
    // Validate arguments
    ESP_ARG_CHECK(filter);

    /* set alpha directly with clamping */
    double a = alpha;
    if (a < 0.0) {
        a = 0.0;
    } else if (a > 1.0) {
        a = 1.0;
    }
    filter->alpha = a;

    filter->previous    = 0.0;
    filter->initialized = false;

    return ESP_OK;
}


esp_err_t cla_filter_iir_lowpass_apply(cla_filter_iir_lowpass_t *const filter, const double x, double *const fv) {
    // Validate arguments
    ESP_ARG_CHECK(filter && fv);

    /* first run, initialize previous output to input value */
    if (!filter->initialized) {
        filter->previous = x;
        filter->initialized = true;

        /* set output parameter */
        *fv = filter->previous;

        return ESP_OK;
    }

    /* apply IIR low-pass filter */
    filter->previous += filter->alpha * (x - filter->previous);

    /* set output parameter */
    *fv = filter->previous;

    return ESP_OK;
}

esp_err_t cla_filter_fir_lowpass_moving_average_init(cla_filter_fir_lowpass_moving_average_t *const ma_filter, const uint32_t window_len) {
    // Validate arguments
    ESP_ARG_CHECK(ma_filter);

    ESP_RETURN_ON_FALSE( (window_len > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid moving average window length, window length must be greater than 0" );

    ma_filter->buffer = (double *)calloc(window_len, sizeof(double));

    ESP_RETURN_ON_FALSE( (ma_filter->buffer != NULL), ESP_ERR_NO_MEM, TAG, "Unable to allocate memory for moving average buffer" );

    ma_filter->window_len = window_len;
    ma_filter->idx        = 0;
    ma_filter->sum        = 0.0;
    ma_filter->filled     = 0;

    return ESP_OK;
}

esp_err_t cla_filter_fir_lowpass_moving_average_delete(cla_filter_fir_lowpass_moving_average_t *const ma_filter) {
    // Validate arguments
    ESP_ARG_CHECK(ma_filter);

    if (ma_filter->buffer) {
        free(ma_filter->buffer);
        ma_filter->buffer = NULL;
    }

    ma_filter->window_len = 0;
    ma_filter->idx        = 0;
    ma_filter->sum        = 0.0;
    ma_filter->filled     = 0;

    return ESP_OK;
}

esp_err_t cla_filter_fir_lowpass_moving_average_apply(cla_filter_fir_lowpass_moving_average_t *const ma_filter, const double x, double *const fv) {
    // Validate arguments
    ESP_ARG_CHECK(ma_filter && fv);
    ESP_RETURN_ON_FALSE( (ma_filter->window_len > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid moving average window length, window length must be greater than 0" );
    ESP_RETURN_ON_FALSE( (ma_filter->buffer != NULL), ESP_ERR_INVALID_ARG, TAG, "Invalid moving average buffer, buffer is NULL, initialize moving average filter" );

    /* subtract oldest, add new */
    ma_filter->sum -= ma_filter->buffer[ma_filter->idx];
    ma_filter->buffer[ma_filter->idx] = x;
    ma_filter->sum += x;
    ma_filter->idx = (ma_filter->idx + 1) % ma_filter->window_len;

    if (ma_filter->filled < ma_filter->window_len) ma_filter->filled++;

    *fv = ma_filter->sum / (double)ma_filter->filled;

    return ESP_OK;
}

esp_err_t cla_filter_get_fft_dft(double *real, double *imag, const uint16_t n, int16_t *const dft) {
    return cla_filter_fft_core(real, imag, n, false);
}

esp_err_t cla_filter_get_fft_idft(double *real, double *imag, const uint16_t n, int16_t *const dft) {
    esp_err_t r = cla_filter_fft_core(real, imag, n, true);

    if(r != ESP_OK) {
        return r;
    }

    /* normalize */
    for (uint16_t i = 0; i < n; ++i) {
        real[i] /= (double)n;
        imag[i] /= (double)n;
    }

    return ESP_OK;
}

