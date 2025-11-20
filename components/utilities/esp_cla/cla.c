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
 * @file cla.c
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
#include "include/cla.h"

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "cla";



esp_err_t cla_matrix_multiply_vector(const cla_matrix_ptr_t m, const cla_vector_ptr_t v, cla_vector_ptr_t *const v_product) {
    ESP_ARG_CHECK(m && v);
    ESP_RETURN_ON_FALSE( (m->num_cols == v->num_cmps), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix and vector dimensions, number of columns in matrix must match number of components in vector" );
    ESP_RETURN_ON_ERROR( cla_vector_create(m->num_rows, v_product), TAG, "Unable to create vector instance, matrix-vector multiplication failed" );
    for(uint16_t i = 0; i < m->num_rows; i++) {
        (*v_product)->data[i] = 0.0;
        for(uint16_t j = 0; j < m->num_cols; j++) {
            (*v_product)->data[i] += m->data[i][j] * v->data[j];
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_to_vector(const cla_matrix_ptr_t m, cla_vector_ptr_t *const v) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (m->num_cols == 1 && m->num_rows > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, matrix must be n x 1 to convert to vector" );
    ESP_RETURN_ON_ERROR( cla_vector_create(m->num_rows, v), TAG, "Unable to create vector instance, matrix to vector conversion failed" );
    for(uint16_t i = 0; i < m->num_rows; i++) {
        (*v)->data[i] = m->data[i][0];
    }
    return ESP_OK;
}

esp_err_t cla_vector_to_matrix(const cla_vector_ptr_t v, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(v);
    ESP_RETURN_ON_FALSE( (v->num_cmps > 0), ESP_ERR_INVALID_ARG, TAG, "Invalid vector dimensions, number of components must be greater than 0 to convert to matrix" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(v->num_cmps, 1, m), TAG, "Unable to create matrix instance, vector to matrix conversion failed" );
    for(uint16_t i = 0; i < v->num_cmps; i++) {
        (*m)->data[i][0] = v->data[i];
    }
    return ESP_OK;
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

bool cla_is_value_equal(const double val1, const double val2, const double tolerance) {
    const double diff = fabs(val1 - val2);
    return diff <= tolerance || diff < fmax(fabs(val1), fabs(val2)) * tolerance;
}

esp_err_t cla_solve_ellipsoid_coefficients(const cla_vector_samples_t v_calib_data, cla_ellipsoid_coeffs_t *const v_ellip_coeffs) {
    esp_err_t ret = ESP_OK;

    ESP_ARG_CHECK(v_calib_data);

    // ellipsoid fitting algorithm

    // Create the design matrix D (N x 9)
    cla_matrix_ptr_t d_matrix = NULL;
    ESP_GOTO_ON_ERROR(cla_matrix_create(CLA_CAL_SAMPLE_SIZE, CLA_ELLIPSOID_COEFF_SIZE, &d_matrix), cleanup, TAG, "Failed to create design matrix");

    // Create the observation vector O (N x 1)
    cla_matrix_ptr_t o_matrix = NULL;
    ESP_GOTO_ON_ERROR(cla_matrix_create(CLA_CAL_SAMPLE_SIZE, 1, &o_matrix), cleanup, TAG, "Failed to create observation matrix");

    // Populate the D and O matrices
    for(uint16_t i = 0; i < CLA_CAL_SAMPLE_SIZE; i++) {
        const cla_vector_ptr_t sample = v_calib_data[i];
        if (sample == NULL || sample->num_cmps != 3) {
            ret = ESP_ERR_INVALID_ARG;
            goto cleanup;
        }
        const double x = sample->data[0];
        const double y = sample->data[1];
        const double z = sample->data[2];

        // Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
        d_matrix->data[i][0] = x * x;
        d_matrix->data[i][1] = y * y;
        d_matrix->data[i][2] = z * z;
        d_matrix->data[i][3] = x * y;
        d_matrix->data[i][4] = x * z;
        d_matrix->data[i][5] = y * z;
        d_matrix->data[i][6] = x;
        d_matrix->data[i][7] = y;
        d_matrix->data[i][8] = z;

        o_matrix->data[i][0] = 1.0; // J = -1, so -J = 1
    }

    // Solve QR decomposition
    cla_matrix_ptr_t v_matrix = NULL;
    ESP_GOTO_ON_ERROR(cla_matrix_qr_solve(d_matrix, o_matrix, &v_matrix), cleanup, TAG, "Failed to solve with QR decomposition");

    // Convert the resulting matrix to a vector
    ESP_GOTO_ON_ERROR(cla_matrix_to_vector(v_matrix, v_ellip_coeffs), cleanup, TAG, "Failed to convert result matrix to vector");

cleanup:
    cla_matrix_delete(d_matrix);
    cla_matrix_delete(o_matrix);
    cla_matrix_delete(v_matrix);

    if (*v_ellip_coeffs == NULL) ret = ESP_FAIL;

    return ret;
}

esp_err_t cla_get_calibration_parameters(const cla_vector_ptr_t v_ellip_coeffs, cla_vector_ptr_t *const v_offset, cla_matrix_ptr_t *const m_w) {
    esp_err_t ret = ESP_OK;

    // 1. Construct the M matrix and b vector from coefficients
    cla_matrix_ptr_t M = NULL;
    cla_vector_ptr_t b = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create(3, 3, &M), TAG, "Unable to create matrix instance, M matrix creation failed" );
    ESP_RETURN_ON_ERROR( cla_vector_create(3, &b), TAG, "Unable to create vector instance, b vector creation failed" );

    M->data[0][0] = v_ellip_coeffs->data[0]; M->data[0][1] = v_ellip_coeffs->data[3] / 2.0; M->data[0][2] = v_ellip_coeffs->data[4] / 2.0; // A, D, E
    M->data[1][0] = v_ellip_coeffs->data[3] / 2.0; M->data[1][1] = v_ellip_coeffs->data[1]; M->data[1][2] = v_ellip_coeffs->data[5] / 2.0; // D, B, F
    M->data[2][0] = v_ellip_coeffs->data[4] / 2.0; M->data[2][1] = v_ellip_coeffs->data[5] / 2.0; M->data[2][2] = v_ellip_coeffs->data[2]; // E, F, C

    b->data[0] = v_ellip_coeffs->data[6]; // G
    b->data[1] = v_ellip_coeffs->data[7]; // H
    b->data[2] = v_ellip_coeffs->data[8]; // I

    // 2. Calculate Hard-Iron offset: offset = -0.5 * M_inv * b
    cla_matrix_lup_ptr_t M_lup = NULL;
    cla_matrix_ptr_t M_inv = NULL;
    cla_vector_ptr_t M_inv_b = NULL;

    ESP_GOTO_ON_ERROR(cla_matrix_lup_solve(M, &M_lup), cleanup, TAG, "LUP decomp failed");
    ESP_GOTO_ON_ERROR(cla_matrix_lup_get_inverse(M_lup, &M_inv), cleanup, TAG, "Matrix inverse failed");
    ESP_GOTO_ON_ERROR(cla_matrix_multiply_vector(M_inv, b, &M_inv_b), cleanup, TAG, "Matrix-vector multiply failed");
    ESP_GOTO_ON_ERROR(cla_vector_scale(M_inv_b, -0.5, v_offset), cleanup, TAG, "Vector scaling failed");

    // 3. Calculate the scalar 's'
    double s = 0;
    cla_vector_ptr_t b_transpose_M_inv = NULL;
    cla_matrix_ptr_t b_transpose = NULL;
    ESP_GOTO_ON_ERROR(cla_vector_to_matrix(b, &b_transpose), cleanup, TAG, "Vector to matrix conversion failed for transpose");
    ESP_GOTO_ON_ERROR(cla_matrix_transpose(b_transpose, &b_transpose), cleanup, TAG, "Matrix transpose failed");
    ESP_GOTO_ON_ERROR(cla_matrix_multiply_vector(M_inv, b, &b_transpose_M_inv), cleanup, TAG, "Matrix-vector multiply for scalar 's' failed");
    double b_dot_Minv_b = b->data[0] * b_transpose_M_inv->data[0] + b->data[1] * b_transpose_M_inv->data[1] + b->data[2] * b_transpose_M_inv->data[2];
    // The equation is solved for ... = 1, so J = -1. s = (1/4) * b^T * M_inv * b - J
    s = 1.0 / ((b_dot_Minv_b / 4.0) + 1.0);

    // 4. Calculate Soft-Iron correction matrix W from scaled M
    cla_matrix_ptr_t M_scaled = NULL;
    ESP_GOTO_ON_ERROR(cla_matrix_scale(M, s, &M_scaled), cleanup, TAG, "Matrix scaling failed");

    // Use Cholesky Decomposition: W = L^T
    cla_matrix_ptr_t cholesky_l = NULL;

    ESP_GOTO_ON_ERROR(cla_matrix_get_cholesky_decomposition(M_scaled, &cholesky_l), cleanup, TAG, "Cholesky decomposition failed");
    ESP_GOTO_ON_ERROR(cla_matrix_transpose(cholesky_l, m_w), cleanup, TAG, "Failed to transpose Cholesky matrix");

cleanup:
    cla_matrix_delete(M);
    cla_vector_delete(b);
    cla_matrix_lup_delete(M_lup);
    cla_matrix_delete(M_inv);
    cla_vector_delete(M_inv_b);
    cla_vector_delete(b_transpose_M_inv);
    cla_matrix_delete(b_transpose);
    cla_matrix_delete(M_scaled);
    cla_matrix_delete(cholesky_l);

    return ret;
}

esp_err_t cla_apply_calibration(const cla_vector_ptr_t v_raw_data, const cla_vector_ptr_t v_offset, const cla_matrix_ptr_t m_w, cla_vector_ptr_t *v_cal_data) {
    esp_err_t ret = ESP_OK;
    cla_vector_ptr_t centered_data = NULL;

    // Hard-iron correction: centered_data = raw_data - offset
    ESP_GOTO_ON_ERROR(cla_vector_subtract(v_raw_data, v_offset, &centered_data), cleanup, TAG, "Vector subtract failed");

    // Soft-iron correction: cal_data = W * centered_data
    ESP_GOTO_ON_ERROR(cla_matrix_multiply_vector(m_w, centered_data, v_cal_data), cleanup, TAG, "Matrix-vector multiply failed");

cleanup:
    cla_vector_delete(centered_data);
    return ret;
}

esp_err_t cla_get_calibration_samples_quality(const cla_vector_samples_t v_calib_data, const double expected_field_strength, cla_calibration_quality_t *quality_report) {
    ESP_ARG_CHECK(v_calib_data);
    
    memset(quality_report, 0, sizeof(cla_calibration_quality_t));
    
    // Initialize min/max
    quality_report->min_x = quality_report->min_y = quality_report->min_z = DBL_MAX;
    quality_report->max_x = quality_report->max_y = quality_report->max_z = -DBL_MAX;
    
    double sum_x = 0, sum_y = 0, sum_z = 0;
    double sum_sq_x = 0, sum_sq_y = 0, sum_sq_z = 0;
    double sum_magnitude = 0;
    double *magnitudes = malloc(CLA_CAL_SAMPLE_SIZE * sizeof(double));
    if (!magnitudes) return ESP_ERR_NO_MEM;
    
    // Pass 1: Calculate statistics
    for (uint16_t i = 0; i < CLA_CAL_SAMPLE_SIZE; i++) {
        if (v_calib_data[i] == NULL || v_calib_data[i]->num_cmps != 3) {
            free(magnitudes);
            return ESP_ERR_INVALID_ARG;
        }
        
        const double x = v_calib_data[i]->data[0];
        const double y = v_calib_data[i]->data[1];
        const double z = v_calib_data[i]->data[2];
        
        // Min/Max
        if (x < quality_report->min_x) quality_report->min_x = x;
        if (x > quality_report->max_x) quality_report->max_x = x;
        if (y < quality_report->min_y) quality_report->min_y = y;
        if (y > quality_report->max_y) quality_report->max_y = y;
        if (z < quality_report->min_z) quality_report->min_z = z;
        if (z > quality_report->max_z) quality_report->max_z = z;
        
        // Sums for mean and variance
        sum_x += x;
        sum_y += y;
        sum_z += z;
        sum_sq_x += x * x;
        sum_sq_y += y * y;
        sum_sq_z += z * z;
        
        // Magnitude
        magnitudes[i] = sqrt(x*x + y*y + z*z);
        sum_magnitude += magnitudes[i];
    }
    
    // Calculate ranges
    quality_report->range_x = quality_report->max_x - quality_report->min_x;
    quality_report->range_y = quality_report->max_y - quality_report->min_y;
    quality_report->range_z = quality_report->max_z - quality_report->min_z;
    
    // Calculate means
    const double mean_x = sum_x / CLA_CAL_SAMPLE_SIZE;
    const double mean_y = sum_y / CLA_CAL_SAMPLE_SIZE;
    const double mean_z = sum_z / CLA_CAL_SAMPLE_SIZE;
    quality_report->mean_magnitude = sum_magnitude / CLA_CAL_SAMPLE_SIZE;
    
    // Calculate variances
    quality_report->variance_x = (sum_sq_x / CLA_CAL_SAMPLE_SIZE) - (mean_x * mean_x);
    quality_report->variance_y = (sum_sq_y / CLA_CAL_SAMPLE_SIZE) - (mean_y * mean_y);
    quality_report->variance_z = (sum_sq_z / CLA_CAL_SAMPLE_SIZE) - (mean_z * mean_z);
    
    // Calculate magnitude standard deviation
    double sum_mag_sq_diff = 0;
    for (uint16_t i = 0; i < CLA_CAL_SAMPLE_SIZE; i++) {
        double diff = magnitudes[i] - quality_report->mean_magnitude;
        sum_mag_sq_diff += diff * diff;
    }
    quality_report->magnitude_std_dev = sqrt(sum_mag_sq_diff / CLA_CAL_SAMPLE_SIZE);
    
    // Check for duplicates (within tolerance)
    const double duplicate_tolerance = 1.0; // Same reading within 1 unit
    quality_report->unique_count = CLA_CAL_SAMPLE_SIZE;
    for (uint16_t i = 0; i < CLA_CAL_SAMPLE_SIZE; i++) {
        for (uint16_t j = i + 1; j < CLA_CAL_SAMPLE_SIZE; j++) {
            const double dx = v_calib_data[i]->data[0] - v_calib_data[j]->data[0];
            const double dy = v_calib_data[i]->data[1] - v_calib_data[j]->data[1];
            const double dz = v_calib_data[i]->data[2] - v_calib_data[j]->data[2];
            const double dist = sqrt(dx*dx + dy*dy + dz*dz);
            
            if (dist < duplicate_tolerance) {
                quality_report->duplicate_count++;
                quality_report->unique_count--;
                break; // Only count once per sample
            }
        }
    }
    
    free(magnitudes);
    
    // Quality assessment
    // 1. Good coverage: Each axis should span at least 80% of expected range
    const double min_expected_range = quality_report->mean_magnitude * 1.6; // Should span ~2x radius
    quality_report->has_good_coverage = 
        (quality_report->range_x >= min_expected_range * 0.8) &&
        (quality_report->range_y >= min_expected_range * 0.8) &&
        (quality_report->range_z >= min_expected_range * 0.8);
    
    // 2. Good distribution: Variance should be reasonable (not too clustered)
    const double min_variance = (quality_report->mean_magnitude * quality_report->mean_magnitude) / 4.0;
    quality_report->has_good_distribution = 
        (quality_report->variance_x >= min_variance) &&
        (quality_report->variance_y >= min_variance) &&
        (quality_report->variance_z >= min_variance);
    
    // 3. Good uniqueness: Less than 20% duplicates
    quality_report->has_good_uniqueness = 
        (quality_report->duplicate_count < (CLA_CAL_SAMPLE_SIZE / 5));
    
    // 4. Calculate overall quality score (0-100)
    const uint8_t coverage_score = quality_report->has_good_coverage ? 40 : 0;
    const uint8_t distribution_score = quality_report->has_good_distribution ? 30 : 0;
    const uint8_t uniqueness_score = quality_report->has_good_uniqueness ? 30 : 0;
    quality_report->overall_quality = coverage_score + distribution_score + uniqueness_score;
    
    ESP_LOGI(TAG, "Calibration Quality Report:");
    ESP_LOGI(TAG, "  X: [%.2f, %.2f] range=%.2f, var=%.2f", 
             quality_report->min_x, quality_report->max_x, 
             quality_report->range_x, quality_report->variance_x);
    ESP_LOGI(TAG, "  Y: [%.2f, %.2f] range=%.2f, var=%.2f", 
             quality_report->min_y, quality_report->max_y, 
             quality_report->range_y, quality_report->variance_y);
    ESP_LOGI(TAG, "  Z: [%.2f, %.2f] range=%.2f, var=%.2f", 
             quality_report->min_z, quality_report->max_z, 
             quality_report->range_z, quality_report->variance_z);
    ESP_LOGI(TAG, "  Magnitude: mean=%.2f, std_dev=%.2f", 
             quality_report->mean_magnitude, quality_report->magnitude_std_dev);
    ESP_LOGI(TAG, "  Samples: %d unique, %d duplicates", 
             quality_report->unique_count, quality_report->duplicate_count);
    ESP_LOGI(TAG, "  Coverage: %s, Distribution: %s, Uniqueness: %s",
             quality_report->has_good_coverage ? "GOOD" : "POOR",
             quality_report->has_good_distribution ? "GOOD" : "POOR",
             quality_report->has_good_uniqueness ? "GOOD" : "POOR");
    ESP_LOGI(TAG, "  Overall Quality: %d/100 %s", 
             quality_report->overall_quality,
             quality_report->overall_quality >= 70 ? "(GOOD)" : "(POOR)");
    
    return ESP_OK;
}

esp_err_t cla_calibration_samples_quality_print(const cla_calibration_quality_t quality_report) {
    printf("Calibration Samples Quality Report:\n");
    printf("  X: [%.2f, %.2f] range=%.2f, var=%.2f\n", 
             quality_report.min_x, quality_report.max_x, 
             quality_report.range_x, quality_report.variance_x);
    printf("  Y: [%.2f, %.2f] range=%.2f, var=%.2f\n", 
             quality_report.min_y, quality_report.max_y, 
             quality_report.range_y, quality_report.variance_y);
    printf("  Z: [%.2f, %.2f] range=%.2f, var=%.2f\n", 
             quality_report.min_z, quality_report.max_z, 
             quality_report.range_z, quality_report.variance_z);
    printf("  Magnitude: mean=%.2f, std_dev=%.2f\n", 
             quality_report.mean_magnitude, quality_report.magnitude_std_dev);
    printf("  Samples: %d unique, %d duplicates\n", 
             quality_report.unique_count, quality_report.duplicate_count);
    printf("  Coverage: %s, Distribution: %s, Uniqueness: %s\n",
             quality_report.has_good_coverage ? "GOOD" : "POOR",
             quality_report.has_good_distribution ? "GOOD" : "POOR",
             quality_report.has_good_uniqueness ? "GOOD" : "POOR");
    printf("  Overall Quality: %d/100 %s\n", 
             quality_report.overall_quality,
             quality_report.overall_quality >= 70 ? "(GOOD)" : "(POOR)");
    return ESP_OK;
}

esp_err_t cla_iir_lowpass_init_cutoff(cla_iir_lowpass_filter_t *const filter, const double fs, const double fc) {
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

esp_err_t cla_iir_lowpass_init_alpha(cla_iir_lowpass_filter_t *const filter, const double alpha) {
    // Validate arguments
    ESP_ARG_CHECK(filter);

    /* set alpha directly */
    if (alpha < 0.0) filter->alpha = 0.0;
    if (alpha > 1.0) filter->alpha = 1.0;

    filter->previous    = 0.0;
    filter->initialized = false;

    return ESP_OK;
}

esp_err_t cla_iir_lowpass_apply(cla_iir_lowpass_filter_t *const filter, const double x, double *const fv) {
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

esp_err_t cla_fir_lowpass_moving_average_init(cla_fir_lowpass_filter_moving_average_t *const ma_filter, const uint32_t window_len) {
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

esp_err_t cla_fir_lowpass_moving_average_delete(cla_fir_lowpass_filter_moving_average_t *const ma_filter) {
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

esp_err_t cla_fir_lowpass_moving_average_apply(cla_fir_lowpass_filter_moving_average_t *const ma_filter, const double x, double *const fv) {
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




const char* cla_get_fw_version(void) {
    return (const char*)CLA_FW_VERSION_STR;
}

int32_t cla_get_fw_version_number(void) {
    return (int32_t)CLA_FW_VERSION_INT32;
}