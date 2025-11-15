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
 * @file matrix.c
 *
 * ESP-IDF compact linear algebra (cla) matrix library
 * 
 * https://github.com/nomemory/neat-matrix-library/blob/main/nml.c
 *
 * https://github.com/molinab297/smlc/blob/master/matrix.c
 * 
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <math.h>
#include <float.h>
#include <stdint.h>
#include "include/matrix.h"


#define CLA_MATRIX_MIN_COEF         (0.000000000000001)
#define CLA_MATRIX_ROW_SIZE_MAX     UINT16_C(INT16_MAX)
#define CLA_MATRIX_COL_SIZE_MAX     UINT16_C(INT16_MAX)


/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "cla_matrix";

/**
 * @brief Swaps two values.
 * 
 * @param val1 The first value to swap with the second value.
 * @param val2 The second value to swap with the first value.
 */
static inline void cla_matrix_swap_values(double *const val1, double *const val2) {
    double temp = *val1;
    *val1 = *val2;
    *val2 = temp;
}

/**
 * @brief Gets the absolute maximum identifier on the column (starting from col_idx -> num_rows).
 * This method is used for pivoting in LUP decomposition.
 * 
 * @param m Matrix to search for absolute maximum value.
 * @param col_idx Column index to search for absolute maximum value.
 * @param max_row_idx Row index of the absolute maximum value found in the specified column.
 * @return esp_err_t ESP_OK on success. 
 */
static inline esp_err_t cla_matrix_get_abs_max_row(const cla_matrix_ptr_t m, const uint16_t col_idx, uint16_t *const max_row_idx) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (col_idx < m->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    double max_value = fabs(m->data[col_idx][col_idx]);
    *max_row_idx = col_idx;
    for(uint16_t i = col_idx + 1; i < m->num_rows; i++) {
        if(fabs(m->data[i][col_idx]) > max_value) {
            max_value = fabs(m->data[i][col_idx]);
            *max_row_idx = i;
        }
    }
    return ESP_OK;
}

/**
 * @brief Gets the first non-zero element on the column (col_idx), under the row (row_idx).
 * This is used to determine the pivot index for Gaussian elimination.
 * 
 * @param m Matrix to search for pivot index.
 * @param row_idx Row index to start searching from.
 * @param col_idx Column index to search for pivot index.
 * @param pivot_row_idx Pivot row index result. -1 if no pivot found.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t cla_matrix_get_pivot_idx(const cla_matrix_ptr_t m, const uint16_t row_idx, const uint16_t col_idx, int16_t *const pivot_row_idx) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < m->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_FALSE( (col_idx < m->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    for(uint16_t i = row_idx; i < m->num_rows; i++) {
        if(fabs(m->data[i][col_idx]) > CLA_MATRIX_MIN_COEF) {
            *pivot_row_idx = i;
            return ESP_OK;
        }
    }
    *pivot_row_idx = -1;
    return ESP_OK;
}

/**
 * @brief Gets the maximum element on the column (col_idx), under the row (row_idx).
 * This is used to determine the pivot index for Gauss-Jordan elimination.
 * 
 * @param m Matrix to search for maximum pivot index.
 * @param row_idx Row index to start searching from.
 * @param col_idx Column index to search for pivot index.
 * @param pivot_row_idx Pivot row index result. -1 if no pivot found.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t cla_matrix_get_max_pivot_idx(const cla_matrix_ptr_t m, const uint16_t row_idx, const uint16_t col_idx, int16_t *const pivot_row_idx) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < m->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_FALSE( (col_idx < m->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    double max = fabs(m->data[row_idx][col_idx]);
    uint16_t maxi = row_idx;
    for(uint16_t i = row_idx + 1; i < m->num_rows; i++) {
        double micol = fabs(m->data[i][col_idx]);
        if(micol > max) {
            max = micol;
            maxi = i;
        }
    }
    *pivot_row_idx = (max < CLA_MATRIX_MIN_COEF) ? -1 : maxi;
    return ESP_OK;
}

/**
 * @brief Performs a single Jacobi rotation to zero out an off-diagonal element.
 */
static void cla_matrix_jacobi_rotate(cla_matrix_ptr_t a, cla_matrix_ptr_t v, int p, int q) {
    if (fabs(a->data[p][q]) < 1e-12) {
        return;
    }

    double theta, t, c, s;
    double g = 100.0 * fabs(a->data[p][q]);

    if (g > 1e-12) {
        double h = a->data[q][q] - a->data[p][p];
        if (fabs(h) + g == fabs(h)) {
            t = (a->data[p][q]) / h;
        } else {
            theta = 0.5 * h / (a->data[p][q]);
            t = 1.0 / (fabs(theta) + sqrt(1.0 + theta * theta));
            if (theta < 0.0) {
                t = -t;
            }
        }
        c = 1.0 / sqrt(1 + t * t);
        s = t * c;
        double tau = s / (1.0 + c);
        h = t * a->data[p][q];

        a->data[p][p] -= h;
        a->data[q][q] += h;
        a->data[p][q] = 0.0;

        for (int i = 0; i < p; i++) { double g_val = a->data[i][p]; double h_val = a->data[i][q]; a->data[i][p] = g_val - s * (h_val + g_val * tau); a->data[i][q] = h_val + s * (g_val - h_val * tau); }
        for (int i = p + 1; i < q; i++) { double g_val = a->data[p][i]; double h_val = a->data[i][q]; a->data[p][i] = g_val - s * (h_val + g_val * tau); a->data[i][q] = h_val + s * (g_val - h_val * tau); }
        for (int i = q + 1; i < a->num_rows; i++) { double g_val = a->data[p][i]; double h_val = a->data[q][i]; a->data[p][i] = g_val - s * (h_val + g_val * tau); a->data[q][i] = h_val + s * (g_val - h_val * tau); }
        for (int i = 0; i < a->num_rows; i++) { double g_val = v->data[i][p]; double h_val = v->data[i][q]; v->data[i][p] = g_val - s * (h_val + g_val * tau); v->data[i][q] = h_val + s * (g_val - h_val * tau); }
    } else {
        a->data[p][q] = 0.0;
    }
}

/**
 * 
 * Matrix Operations - Creation and Deletion
 * 
 */

esp_err_t cla_matrix_create(const uint16_t num_rows, const uint16_t num_cols, cla_matrix_ptr_t *const m) {
    ESP_RETURN_ON_FALSE( (num_rows != 0 || num_cols != 0), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of rows and columns must be greater than 0" );
    ESP_RETURN_ON_FALSE( (num_rows <= CLA_MATRIX_ROW_SIZE_MAX), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of rows exceeds maximum allowed" );
    ESP_RETURN_ON_FALSE( (num_cols <= CLA_MATRIX_COL_SIZE_MAX), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of columns exceeds maximum allowed" );
    cla_matrix_ptr_t matrix = (cla_matrix_ptr_t)calloc(1, sizeof(cla_matrix_t));
    ESP_RETURN_ON_FALSE( (matrix != NULL) , ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, unable to allocate memory for matrix instance" );
    matrix->num_rows  = num_rows;
    matrix->num_cols  = num_cols;
    matrix->is_square = (matrix->num_rows == matrix->num_cols) ? true : false;
    matrix->data      = (double**)calloc(matrix->num_rows, sizeof(double));
    ESP_RETURN_ON_FALSE( (matrix->data != NULL) , ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, unable to allocate memory for matrix rows" );
    for(uint8_t i = 0; i < matrix->num_rows; i++) {
        matrix->data[i] = (double*)calloc(matrix->num_cols, sizeof(double));
        ESP_RETURN_ON_FALSE( (matrix->data[i] != NULL) , ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, unable to allocate memory for matrix columns" );
    }
    *m = matrix;
    return ESP_OK;
}

esp_err_t cla_matrix_create_square(const uint16_t size, cla_matrix_ptr_t *const m) {
    ESP_RETURN_ON_FALSE( (size != 0), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, size must be greater than 0" );
    ESP_RETURN_ON_FALSE( (size <= CLA_MATRIX_ROW_SIZE_MAX), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, size exceeds maximum allowed" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(size, size, m), TAG, "Unable to create matrix instance, create square matrix failed" );
    return ESP_OK;
}

esp_err_t cla_matrix_create_identity(const uint16_t size, cla_matrix_ptr_t *const m) {
    ESP_RETURN_ON_FALSE( (size != 0), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, size must be greater than 0" );
    ESP_RETURN_ON_FALSE( (size <= CLA_MATRIX_ROW_SIZE_MAX), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, size exceeds maximum allowed" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(size, size, m), TAG, "Unable to create matrix instance, create square matrix failed" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        (*m)->data[i][i] = 1.0;
    }
    return ESP_OK;
}

esp_err_t cla_matrix_delete(cla_matrix_ptr_t m) {
    ESP_ARG_CHECK(m);
    for(uint16_t i = 0; i < m->num_rows; i++) {
        free(m->data[i]);
    }
    free(m->data);
    free(m);
    return ESP_OK;
}

esp_err_t cla_matrix_print(cla_matrix_ptr_t m) {
    ESP_ARG_CHECK(m);
    const char *fmt = "%6.6lf\t";
    printf("\n");
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j < m->num_cols; j++) {
            printf(fmt, m->data[i][j]);
        }
        printf("\n");
    }
    printf("\n");
    return ESP_OK;
}

/**
 * 
 * Matrix Operations
 * 
 */

esp_err_t cla_matrix_add(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_sum) {
    ESP_ARG_CHECK(m1 && m2);
    ESP_RETURN_ON_FALSE( (m1->num_rows == m2->num_rows && m1->num_cols == m2->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of rows and columns must be equal for both matrices" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m1->num_rows, m1->num_cols, m_sum), TAG, "Unable to create matrix instance, matrix addition failed" );
    for(uint16_t i = 0; i < (*m_sum)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m_sum)->num_cols; j++) {
            (*m_sum)->data[i][j] = m1->data[i][j] + m2->data[i][j];
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_subtract(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_difference) {
    ESP_ARG_CHECK(m1 && m2);
    ESP_RETURN_ON_FALSE( (m1->num_rows == m2->num_rows && m1->num_cols == m2->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of rows and columns must be equal for both matrices" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m1->num_rows, m1->num_cols, m_difference), TAG, "Unable to create matrix instance, matrix subtraction failed" );
    for(uint16_t i = 0; i < (*m_difference)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m_difference)->num_cols; j++) {
            (*m_difference)->data[i][j] = m1->data[i][j] - m2->data[i][j];
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_multiply(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_product) {
    ESP_ARG_CHECK(m1 && m2);
    ESP_RETURN_ON_FALSE( (m1->num_cols == m2->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of columns in matrix 1 must match number of rows in matrix 2" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m1->num_rows, m2->num_cols, m_product), TAG, "Unable to create matrix instance, matrix multiplication failed" );
    for(uint16_t i = 0; i < (*m_product)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m_product)->num_cols; j++) {
            (*m_product)->data[i][j] = 0.0;
            for(uint16_t k = 0; k < m1->num_cols; k++) {
                (*m_product)->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }
    return ESP_OK;
}



esp_err_t cla_matrix_get_dot_product(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_dot) {
    ESP_ARG_CHECK(m1 && m2);
    ESP_RETURN_ON_FALSE( (m1->num_cols == m2->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of columns in matrix 1 must match number of rows in matrix 2" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m1->num_rows, m2->num_cols, m_dot), TAG, "Unable to create matrix instance, dot product failed" );
    for(uint16_t i = 0; i < (*m_dot)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m_dot)->num_cols; j++) {
            (*m_dot)->data[i][j] = 0.0;
            for(uint16_t k = 0; k < m1->num_cols; k++) {
                (*m_dot)->data[i][j] += m1->data[i][k] * m2->data[k][j];
            }
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_vector_dot_product(const cla_matrix_ptr_t m1, const uint16_t m1_col_idx, const cla_matrix_ptr_t m2, const uint16_t m2_col_idx, double *const dot) {
    ESP_ARG_CHECK(m1 && m2);
    ESP_RETURN_ON_FALSE( (m2->num_rows == m1->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, m1 must be n x 1 and m2 must be 1 x n with matching n dimensions to compute dot product" );
    ESP_RETURN_ON_FALSE( (m1_col_idx < m1->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index for matrix m1" );
    ESP_RETURN_ON_FALSE( (m2_col_idx < m2->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index for matrix m2" );
    *dot = 0.0;
    for(uint16_t i = 0; i < m1->num_rows; i++) {
        *dot += m1->data[i][m1_col_idx] * m2->data[i][m2_col_idx];
    }
    return ESP_OK;  
}

esp_err_t cla_matrix_get_inverse(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_inverse) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (m->is_square), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, only square matrices can be inverted" );
    cla_matrix_ptr_t augmented_matrix = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_rows, m->num_cols * 2, &augmented_matrix), TAG, "Unable to create augmented matrix for inversion" );
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j < m->num_cols; j++) {
            augmented_matrix->data[i][j] = m->data[i][j];
        }
        for(uint16_t j = 0; j < m->num_cols; j++) {
            augmented_matrix->data[i][j + m->num_cols] = (i == j) ? 1.0 : 0.0;
        }
    }
    for(uint16_t i = 0; i < m->num_rows; i++) {
        int16_t pivot_row = -1;
        ESP_RETURN_ON_ERROR( cla_matrix_get_max_pivot_idx(augmented_matrix, i, i, &pivot_row), TAG, "Unable to get pivot index during inversion" );
        ESP_RETURN_ON_FALSE( (pivot_row != -1), ESP_ERR_INVALID_ARG, TAG, "Matrix is singular and cannot be inverted" );
        if(pivot_row != i) {
            for(uint16_t j = 0; j < augmented_matrix->num_cols; j++) {
                cla_matrix_swap_values(&augmented_matrix->data[i][j], &augmented_matrix->data[pivot_row][j]);
            }
        }
        double pivot_value = augmented_matrix->data[i][i];
        for(uint16_t j = 0; j < augmented_matrix->num_cols; j++) {
            augmented_matrix->data[i][j] /= pivot_value;
        }
        for(uint16_t k = 0; k < augmented_matrix->num_rows; k++) {
            if(k != i) {
                double factor = augmented_matrix->data[k][i];
                for(uint16_t j = 0; j < augmented_matrix->num_cols; j++) {
                    augmented_matrix->data[k][j] -= factor * augmented_matrix->data[i][j];
                }
            }
        }
    }
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j < m->num_cols; j++) {
            augmented_matrix->data[i][j] = augmented_matrix->data[i][j + m->num_cols];
        }
    }
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_rows, m->num_cols, m_inverse), TAG, "Unable to create inverse matrix" );
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j < m->num_cols; j++) {
            (*m_inverse)->data[i][j] = augmented_matrix->data[i][j];
        }
    }
    cla_matrix_delete(augmented_matrix);
    return ESP_OK;
}

esp_err_t cla_matrix_transpose(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_transpose) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_cols, m->num_rows, m_transpose), TAG, "Unable to create matrix instance, transpose matrix failed" );
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j < m->num_cols; j++) {
            (*m_transpose)->data[j][i] = m->data[i][j];
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_trace(const cla_matrix_ptr_t m, double *const trace) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (m->is_square), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, trace can only be calculated for square matrices" );
    *trace = 0.0;
    for(uint16_t i = 0; i < m->num_rows; i++) {
        *trace += m->data[i][i];
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_cholesky_decomposition(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_cholesky) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (m->is_square), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, Cholesky decomposition can only be performed on square matrices" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_rows, m->num_cols, m_cholesky), TAG, "Unable to create matrix instance, Cholesky decomposition failed" );
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j < m->num_cols; j++) {
            (*m_cholesky)->data[i][j] = 0.0;
        }
    }
    for(uint16_t i = 0; i < m->num_rows; i++) {
        for(uint16_t j = 0; j <= i; j++) {
            double sum = 0.0;
            for(uint16_t k = 0; k < j; k++) {
                sum += (*m_cholesky)->data[i][k] * (*m_cholesky)->data[j][k];
            }
            if(i == j) {
                double diag = m->data[i][i] - sum;
                ESP_RETURN_ON_FALSE( (diag > 0.0), ESP_ERR_INVALID_ARG, TAG, "Matrix is not positive definite, Cholesky decomposition failed" );
                (*m_cholesky)->data[i][j] = sqrt(diag);
            } else {
                (*m_cholesky)->data[i][j] = (m->data[i][j] - sum) / (*m_cholesky)->data[j][j];
            }
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_eigen_decomposition(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_eigenvectors, cla_matrix_ptr_t *const m_eigenvalues) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE(m->is_square, ESP_ERR_INVALID_ARG, TAG, "Matrix must be square for eigenvalue decomposition.");

    cla_matrix_ptr_t a_copy = NULL;
    ESP_RETURN_ON_ERROR(cla_matrix_copy(m, &a_copy), TAG, "Failed to copy matrix for eigen decomposition.");
    ESP_RETURN_ON_ERROR(cla_matrix_create_identity(m->num_rows, m_eigenvectors), TAG, "Failed to create eigenvector matrix.");

    for (int iter = 0; iter < 50; iter++) {
        for (int p = 0; p < m->num_rows; p++) {
            for (int q = p + 1; q < m->num_rows; q++) {
                cla_matrix_jacobi_rotate(a_copy, *m_eigenvectors, p, q);
            }
        }
    }

    ESP_RETURN_ON_ERROR(cla_matrix_create(m->num_rows, m->num_rows, m_eigenvalues), TAG, "Failed to create eigenvalue matrix.");
    for (int i = 0; i < m->num_rows; i++) {
        for (int j = 0; j < m->num_rows; j++) {
            if (i == j) {
                (*m_eigenvalues)->data[i][j] = a_copy->data[i][j];
            } else {
                (*m_eigenvalues)->data[i][j] = 0.0;
            }
        }
    }
    cla_matrix_delete(a_copy);
    return ESP_OK;
}

esp_err_t cla_matrix_copy(const cla_matrix_ptr_t m_src, cla_matrix_ptr_t *const m_dst) {
    ESP_ARG_CHECK(m_src);
    ESP_RETURN_ON_ERROR( cla_matrix_create(m_src->num_rows, m_src->num_cols, m_dst), TAG, "Unable to create matrix instance, copy matrix failed" );
    for(uint16_t i = 0; i < (*m_dst)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m_dst)->num_cols; j++) {
            (*m_dst)->data[i][j] = m_src->data[i][j];
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_scale(const cla_matrix_ptr_t m, const double scalar, cla_matrix_ptr_t *const m_scaled) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_rows, m->num_cols, m_scaled), TAG, "Unable to create matrix instance, scale matrix failed" );
    for(uint16_t i = 0; i < (*m_scaled)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m_scaled)->num_cols; j++) {
            (*m_scaled)->data[i][j] = m->data[i][j] * scalar;
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_is_dimension_equal(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, bool *const is_equal) {
    ESP_ARG_CHECK(m1 && m2);
    *is_equal = (m1->num_rows == m2->num_rows) && (m1->num_cols == m2->num_cols);
    return ESP_OK;
}

esp_err_t cla_matrix_is_equal(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, const double tolerance, bool *const is_equal) {
    ESP_ARG_CHECK(m1 && m2);
    ESP_RETURN_ON_FALSE( (m1->num_rows == m2->num_rows && m1->num_cols == m2->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of rows and columns must be equal for both matrices" );
    for(uint16_t i = 0; i < m1->num_rows; i++) {
        for(uint16_t j = 0; j < m1->num_cols; j++) {
            const double diff = fabs(m1->data[i][j] - m2->data[i][j]);
            if(diff > tolerance || diff > fmax(fabs(m1->data[i][j]), fabs(m2->data[i][j])) * tolerance) {
                *is_equal = false;
                return ESP_OK;
            }
        }
    }
    *is_equal = true;
    return ESP_OK;
}

esp_err_t cla_matrix_is_empty(const cla_matrix_ptr_t m, bool *const is_empty) {
    ESP_ARG_CHECK(m);
    *is_empty = (m->num_rows == 0 || m->num_cols == 0) ? true : false;
    return ESP_OK;
}

esp_err_t cla_matrix_is_square(const cla_matrix_ptr_t m, bool *const is_square) {
    ESP_ARG_CHECK(m);
    *is_square = (m->num_rows == m->num_cols) ? true : false;
    return ESP_OK;
}

esp_err_t cla_matrix_get_value(const uint16_t row_idx, const uint16_t col_idx, const cla_matrix_ptr_t m, double *const value) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < m->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_FALSE( (col_idx < m->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    *value = m->data[row_idx][col_idx];
    return ESP_OK;
}

esp_err_t cla_matrix_set_value(const uint16_t row_idx, const uint16_t col_idx, const double value, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_FALSE( (col_idx < (*m)->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    (*m)->data[row_idx][col_idx] = value;
    return ESP_OK;
}

esp_err_t cla_matrix_set_values(const double value, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m)->num_cols; j++) {
            (*m)->data[i][j] = value;
        }
    }
    return ESP_OK;
}

esp_err_t cla_matrix_set_diagonal_values(const double value, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( ((*m)->is_square), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, diagonal values can only be set for square matrices" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        (*m)->data[i][i] = value;
    }
    return ESP_OK;
}

esp_err_t cla_matrix_zero_values(cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m)->num_cols; j++) {
            (*m)->data[i][j] = 0.0;
        }
    }
    return ESP_OK;
}

/**
 * 
 * Matrix Column Operations - Accessors and Modifiers
 * 
 */

esp_err_t cla_matrix_add_column(cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    uint16_t col_idx = (*m)->num_cols;
    cla_matrix_ptr_t m_d = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create((*m)->num_rows, (*m)->num_cols + 1, &m_d), TAG, "Unable to create matrix instance, add column to matrix failed" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m)->num_cols; j++) {
            m_d->data[i][j] = (*m)->data[i][j];
        }
    }
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        m_d->data[i][col_idx] = 0.0;
    }
    cla_matrix_delete(*m);
    *m = m_d;
    return ESP_OK;
}

esp_err_t cla_matrix_delete_column(const uint16_t col_idx, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (col_idx < (*m)->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    cla_matrix_ptr_t m_d = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create((*m)->num_rows, (*m)->num_cols - 1, &m_d), TAG, "Unable to create matrix instance, delete column from matrix failed" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        for(uint16_t j = 0, k = 0; j < (*m)->num_cols; j++) {
            if(col_idx != j) {
                m_d->data[i][k++] = (*m)->data[i][j];
            }
        }
    }
    cla_matrix_delete(*m);
    *m = m_d;
    return ESP_OK;
}

esp_err_t cla_matrix_swap_column(const uint16_t col_idx1, const uint16_t col_idx2, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (col_idx1 < (*m)->num_cols && col_idx2 < (*m)->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        double temp = (*m)->data[i][col_idx1];
        (*m)->data[i][col_idx1] = (*m)->data[i][col_idx2];
        (*m)->data[i][col_idx2] = temp;
    }
    return ESP_OK;
}

esp_err_t cla_matrix_swap_columns(cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    uint16_t col_right = (*m)->num_cols - 1;
    for(uint16_t col_left = 0; col_left < col_right; col_left++, col_right--) {
        for(uint16_t row = 0; row < (*m)->num_rows; row++) {
            cla_matrix_swap_values(*((*m)->data+row)+col_left, *((*m)->data+row)+col_right);
        }
        col_right--;
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_column(const cla_matrix_ptr_t m, const uint16_t col_idx, cla_matrix_ptr_t *const m_col) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (col_idx < m->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_rows, 1, m_col), TAG, "Unable to create matrix instance, get column from matrix failed" );
    for(uint16_t i = 0; i < (*m_col)->num_rows; i++) {
        (*m_col)->data[i][0] = m->data[i][col_idx];
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_column_l2norm(const cla_matrix_ptr_t m, const uint16_t col_idx, double *const col_norm) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (col_idx < m->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    double sum = 0.0;
    for(uint16_t i = 0; i < m->num_rows; i++) {
        sum += m->data[i][col_idx] * m->data[i][col_idx];
    }
    *col_norm = sqrt(sum);
    return ESP_OK;
}

esp_err_t cla_matrix_get_columns_l2norm(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_norm) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_ERROR( cla_matrix_create(1, m->num_cols, m_norm), TAG, "Unable to create matrix instance, get columns l2norm from matrix failed" );
    for(uint16_t i = 0; i < (*m_norm)->num_cols; i++) {
        cla_matrix_get_column_l2norm(m, i, &((*m_norm)->data[0][i]));
    }
    return ESP_OK;
}

esp_err_t cla_matrix_multiply_column(const uint16_t col_idx, const double scalar, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (col_idx < (*m)->num_cols), ESP_ERR_INVALID_ARG, TAG, "Invalid column index, column index must be lower than the number of columns" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        (*m)->data[i][col_idx] *= scalar;
    }
    return ESP_OK;
}


/**
 * 
 * Matrix Row Operations - Accessors and Modifiers
 * 
 */

esp_err_t cla_matrix_add_scaled_row(const uint16_t row_idx, const uint16_t row_idx_to_add, const double scalar, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_FALSE( (row_idx_to_add < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index to add, row index must be lower than the number of rows" );
    for(uint16_t j = 0; j < (*m)->num_cols; j++) {
        (*m)->data[row_idx_to_add][j] += scalar * (*m)->data[row_idx][j];
    }
    return ESP_OK;  
}

esp_err_t cla_matrix_add_row(cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    uint16_t row_idx = (*m)->num_rows;
    cla_matrix_ptr_t m_d = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create((*m)->num_rows + 1, (*m)->num_cols, &m_d), TAG, "Unable to create matrix instance, add row to matrix failed" );
    for(uint16_t i = 0; i < (*m)->num_rows; i++) {
        for(uint16_t j = 0; j < (*m)->num_cols; j++) {
            m_d->data[i][j] = (*m)->data[i][j];
        }
    }
    for(uint16_t j = 0; j < (*m)->num_cols; j++) {
        m_d->data[row_idx][j] = 0.0;
    }
    cla_matrix_delete(*m);
    *m = m_d;
    return ESP_OK;
}

esp_err_t cla_matrix_delete_row(const uint16_t row_idx, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    cla_matrix_ptr_t m_d = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create((*m)->num_rows - 1, (*m)->num_cols, &m_d), TAG, "Unable to create matrix instance, delete row from matrix failed" );
    for(uint16_t i = 0, k = 0; i < (*m)->num_rows; i++) {
        if(row_idx != i) {
            for(uint16_t j = 0; j < (*m)->num_cols; j++) {
                m_d->data[k][j] = (*m)->data[i][j];
            }
            k++;
        }
    }
    cla_matrix_delete(*m);
    *m = m_d;
    return ESP_OK;
}

esp_err_t cla_matrix_swap_row(const uint16_t row_idx1, const uint16_t row_idx2, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx1 < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid first row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_FALSE( (row_idx2 < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid second row index, row index must be lower than the number of rows" );
    double *tmp = (*m)->data[row_idx2];
    (*m)->data[row_idx2] = (*m)->data[row_idx1];
    (*m)->data[row_idx1] = tmp;
    return ESP_OK;
}

esp_err_t cla_matrix_swap_rows(const uint16_t row_idx1, const uint16_t row_idx2, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx1 < (*m)->num_rows && row_idx2 < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    for(uint16_t i = 0; i < (*m)->num_cols; i++) {
        const double temp  = (*m)->data[row_idx1][i];
        (*m)->data[row_idx1][i] = (*m)->data[row_idx2][i];
        (*m)->data[row_idx2][i] = temp;
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_row(const cla_matrix_ptr_t m, const uint16_t row_idx, cla_matrix_ptr_t *const m_row) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < m->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(1, m->num_cols, m_row), TAG, "Unable to create matrix instance, get row from matrix failed" );
    memcpy((*m_row)->data[0], m->data[row_idx], m->num_cols * sizeof(*(*m_row)->data[0]));
    return ESP_OK;
}

esp_err_t cla_matrix_divide_row(const uint16_t row_idx, const double divisor, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    for(uint16_t i = 0; i < (*m)->num_cols; i++) {
        (*m)->data[row_idx][i] /= divisor; // Reduce row by dividing by a common divisor
    }
    return ESP_OK;
}

esp_err_t cla_matrix_multiply_row(const uint16_t row_idx, const double scalar, cla_matrix_ptr_t *const m) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (row_idx < (*m)->num_rows), ESP_ERR_INVALID_ARG, TAG, "Invalid row index, row index must be lower than the number of rows" );
    for(uint16_t i = 0; i < (*m)->num_cols; i++) {
        (*m)->data[row_idx][i] *= scalar;
    }
    return ESP_OK;
}


/**
 * 
 * Matrix LUP Operations - Creation and Deletion
 * 
 */

esp_err_t cla_matrix_lup_create(const cla_matrix_ptr_t m_l, const cla_matrix_ptr_t m_u, const cla_matrix_ptr_t m_p, const uint16_t num_permutations, cla_matrix_lup_ptr_t *const m_lup) {
    ESP_RETURN_ON_FALSE( (m_l != NULL && m_u != NULL && m_p != NULL), ESP_ERR_INVALID_ARG, TAG, "Invalid L, U, or P matrix, cannot be NULL" );
    cla_matrix_lup_ptr_t lup = (cla_matrix_lup_ptr_t)calloc(1, sizeof(cla_matrix_lup_t));
    ESP_RETURN_ON_FALSE( (lup != NULL), ESP_ERR_NO_MEM, TAG, "Unable to allocate memory for LUP decomposition matrix structure" );
    lup->l = m_l;
    lup->u = m_u;
    lup->p = m_p;
    lup->num_permutations = num_permutations;
    *m_lup = lup;
    return ESP_OK;
}

esp_err_t cla_matrix_lup_delete(cla_matrix_lup_ptr_t m_lup) {
    ESP_ARG_CHECK(m_lup);
    ESP_RETURN_ON_ERROR( cla_matrix_delete(m_lup->l), TAG, "Unable to delete L matrix from LUP decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_delete(m_lup->u), TAG, "Unable to delete U matrix from LUP decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_delete(m_lup->p), TAG, "Unable to delete P matrix from LUP decomposition" );
    free(m_lup);
    return ESP_OK;
}

esp_err_t cla_matrix_lup_print(cla_matrix_lup_ptr_t m_lup) {
    ESP_ARG_CHECK(m_lup);
    printf("L Matrix:\n");
    ESP_RETURN_ON_ERROR( cla_matrix_print(m_lup->l), TAG, "Unable to print L matrix from LUP decomposition" );
    printf("U Matrix:\n");
    ESP_RETURN_ON_ERROR( cla_matrix_print(m_lup->u), TAG, "Unable to print U matrix from LUP decomposition" );
    printf("P Matrix:\n");
    ESP_RETURN_ON_ERROR( cla_matrix_print(m_lup->p), TAG, "Unable to print P matrix from LUP decomposition" );
    printf("Number of permutations: %u\n", m_lup->num_permutations);
    return ESP_OK;
}

/**
 * 
 * Matrix LUP Operations
 * 
 */

esp_err_t cla_matrix_lup_copy(const cla_matrix_lup_ptr_t m_lup_src, cla_matrix_lup_ptr_t *const m_lup_dst) {
    ESP_ARG_CHECK(m_lup_src);
    cla_matrix_ptr_t l_copy = NULL;
    cla_matrix_ptr_t u_copy = NULL;
    cla_matrix_ptr_t p_copy = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_lup_src->l, &l_copy), TAG, "Unable to copy L matrix from LUP decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_lup_src->u, &u_copy), TAG, "Unable to copy U matrix from LUP decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_lup_src->p, &p_copy), TAG, "Unable to copy P matrix from LUP decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_lup_create(l_copy, u_copy, p_copy, m_lup_src->num_permutations, m_lup_dst), TAG, "Unable to create LUP decomposition matrix copy" );
    return ESP_OK;
}

esp_err_t cla_matrix_lup_solve(const cla_matrix_ptr_t m, cla_matrix_lup_ptr_t *const m_lup) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_FALSE( (m->is_square), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix, LUP decomposition can only be set for square matrices" );
    cla_matrix_ptr_t l = NULL;
    cla_matrix_ptr_t u = NULL;
    cla_matrix_ptr_t p = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create(m->num_rows, m->num_cols, &l), TAG, "Unable to create matrix instance, create L matrix failed" );
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m, &u), TAG, "Unable to copy matrix, copy U matrix failed" );
    ESP_RETURN_ON_ERROR( cla_matrix_create_identity(m->num_rows, &p), TAG, "Unable to create matrix identity instance, create P matrix failed" );
    uint16_t num_permutations = 0;
    for(uint16_t j = 0; j < u->num_cols; j++) {
        uint16_t pivot_row;
        ESP_RETURN_ON_ERROR( cla_matrix_get_abs_max_row(u, j, &pivot_row), TAG, "Unable to get absolute maximum row for pivoting" );
        ESP_RETURN_ON_FALSE( fabs(u->data[pivot_row][j]) > CLA_MATRIX_MIN_COEF, ESP_ERR_INVALID_ARG, TAG, "Matrix is singular, cannot perform LUP decomposition" );
        if(pivot_row != j) {
            ESP_RETURN_ON_ERROR( cla_matrix_swap_rows(j, pivot_row, &u), TAG, "Unable to swap rows in U matrix for pivoting" );
            ESP_RETURN_ON_ERROR( cla_matrix_swap_rows(j, pivot_row, &l), TAG, "Unable to swap rows in L matrix for pivoting" );
            ESP_RETURN_ON_ERROR( cla_matrix_swap_rows(j, pivot_row, &p), TAG, "Unable to swap rows in P matrix for pivoting" );
            num_permutations++;
        }
        for(uint16_t i = j + 1; i < u->num_rows; i++) {
            double factor = u->data[i][j] / u->data[j][j];
            ESP_RETURN_ON_ERROR( cla_matrix_add_scaled_row(j, i, -factor, &u), TAG, "Unable to add row to U matrix during elimination" );
            l->data[i][j] = factor;
        }
    }
    ESP_RETURN_ON_ERROR( cla_matrix_set_diagonal_values(1.0f, &l), TAG, "Unable to set diagonal values for L matrix" );
    ESP_RETURN_ON_ERROR( cla_matrix_lup_create(l, u, p, num_permutations, m_lup), TAG, "Unable to create LUP decomposition matrix" );
    return ESP_OK;
}

esp_err_t cla_matrix_lup_get_inverse(const cla_matrix_lup_ptr_t m_lup, cla_matrix_ptr_t *const m_inverse) {
    ESP_ARG_CHECK(m_lup);
    const uint16_t num_cols = m_lup->l->num_cols;
    const uint16_t num_rows = m_lup->l->num_rows;
    ESP_RETURN_ON_ERROR( cla_matrix_create_square(num_cols, m_inverse), TAG, "Unable to create square matrix instance, create inverse matrix failed" );
    cla_matrix_ptr_t i = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create_identity(num_rows, &i), TAG, "Unable to create matrix instance, create identity matrix failed" );
    for(uint16_t j = 0; j < num_cols; j++) {
        cla_matrix_ptr_t invx = NULL;
        cla_matrix_ptr_t ix = NULL;
        ESP_RETURN_ON_ERROR( cla_matrix_get_column(i, j, &ix), TAG, "Unable to get column from identity matrix" );
        ESP_RETURN_ON_ERROR( cla_matrix_ls_solve(m_lup, ix, &invx), TAG, "Unable to solve linear system for inverse matrix column" );
        for(uint16_t i = 0; i < num_rows; i++) {
            (*m_inverse)->data[i][j] = invx->data[i][0];
        }
        cla_matrix_delete(invx);
        cla_matrix_delete(ix);
    }
    cla_matrix_delete(i);
    return ESP_OK;
}

esp_err_t cla_matrix_lup_get_determinant(const cla_matrix_lup_ptr_t m_lup, double *const determinant) {
    ESP_ARG_CHECK(m_lup);
    int8_t sign = (m_lup->num_permutations%2==0) ? 1 : -1;
    cla_matrix_ptr_t u = m_lup->u;
    double product = 1.0;
    for(uint16_t i = 0; i < u->num_rows; i++) {
        product *= u->data[i][i];
    }
    *determinant = sign * product;
    return ESP_OK;
}

esp_err_t cla_matrix_lup_get_lu(const cla_matrix_lup_ptr_t m_lup, cla_matrix_ptr_t *const m_lu) {
    ESP_ARG_CHECK(m_lup);
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_lup->u, m_lu), TAG, "Unable to copy matrix, copy LU matrix failed" );
    for(uint16_t i = 1; i < m_lup->l->num_rows; i++) {
        for(uint16_t j = 0; j < i; j++) {
            (*m_lu)->data[i][j] = m_lup->l->data[i][j];
        }
    }
    return ESP_OK;
}




/**
 * 
 * Matrix QR Operations - Creation and Deletion
 * 
 */

esp_err_t cla_matrix_qr_create(const cla_matrix_ptr_t m_q, const cla_matrix_ptr_t m_r, cla_matrix_qr_ptr_t *const m_qr) {
    ESP_ARG_CHECK(m_q && m_r);
    cla_matrix_qr_ptr_t qr = (cla_matrix_qr_ptr_t)calloc(1, sizeof(cla_matrix_qr_t));
    ESP_RETURN_ON_FALSE( (qr != NULL), ESP_ERR_NO_MEM, TAG, "Unable to allocate memory for QR decomposition matrix structure" );
    qr->q = m_q;
    qr->r = m_r;
    *m_qr = qr;
    return ESP_OK;
}

esp_err_t cla_matrix_qr_delete(cla_matrix_qr_ptr_t m_qr) {
    ESP_ARG_CHECK(m_qr);
    ESP_RETURN_ON_ERROR( cla_matrix_delete(m_qr->q), TAG, "Unable to delete Q matrix from QR decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_delete(m_qr->r), TAG, "Unable to delete R matrix from QR decomposition" );
    free(m_qr);
    return ESP_OK;
}

esp_err_t cla_matrix_qr_print(cla_matrix_qr_ptr_t m_qr) {
    ESP_ARG_CHECK(m_qr);
    printf("Q Matrix:\n");
    ESP_RETURN_ON_ERROR( cla_matrix_print(m_qr->q), TAG, "Unable to print Q matrix from QR decomposition" );
    printf("R Matrix:\n");
    ESP_RETURN_ON_ERROR( cla_matrix_print(m_qr->r), TAG, "Unable to print R matrix from QR decomposition" );
    return ESP_OK;
}

esp_err_t cla_matrix_qr_copy(const cla_matrix_qr_ptr_t m_qr_src, cla_matrix_qr_ptr_t *const m_qr_dst) {
    ESP_ARG_CHECK(m_qr_src);
    cla_matrix_ptr_t q = NULL;
    cla_matrix_ptr_t r = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_qr_src->q, &q), TAG, "Unable to copy Q matrix in QR decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_qr_src->r, &r), TAG, "Unable to copy R matrix in QR decomposition" );
    ESP_RETURN_ON_ERROR( cla_matrix_qr_create(q, r, m_qr_dst), TAG, "Unable to create QR decomposition instance, copy QR decomposition failed" );
    return ESP_OK;
}

esp_err_t cla_matrix_qr_get_decomposition(const cla_matrix_ptr_t m_a, cla_matrix_qr_ptr_t *const m_qr_a) {
    esp_err_t ret = ESP_OK;

    ESP_ARG_CHECK(m_a);

    cla_matrix_ptr_t q = NULL, r = NULL, u = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_create(m_a->num_rows, m_a->num_cols, &q), TAG, "Failed to create Q matrix" );
    ESP_RETURN_ON_ERROR( cla_matrix_create(m_a->num_cols, m_a->num_cols, &r), TAG, "Failed to create R matrix" );
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_a, &u), TAG, "Failed to copy matrix to U" );

    for (uint16_t j = 0; j < m_a->num_cols; j++) {
        cla_matrix_ptr_t u_j = NULL, q_i = NULL;
        ESP_GOTO_ON_ERROR( cla_matrix_get_column(u, j, &u_j), cleanup, TAG, "Failed to get column from U" );

        for (uint16_t i = 0; i < j; i++) {
            ESP_GOTO_ON_ERROR( cla_matrix_get_column(q, i, &q_i), cleanup, TAG, "Failed to get column from Q" );
            double dot_product;
            ESP_GOTO_ON_ERROR( cla_matrix_get_vector_dot_product(q_i, 0, u_j, 0, &dot_product), cleanup, TAG, "Failed to compute dot product" );
            r->data[i][j] = dot_product;

            cla_matrix_ptr_t q_i_scaled = NULL;
            ESP_GOTO_ON_ERROR( cla_matrix_scale(q_i, dot_product, &q_i_scaled), cleanup, TAG, "Failed to scale Q column" );
            
            cla_matrix_ptr_t temp_sub = NULL;
            ESP_GOTO_ON_ERROR( cla_matrix_subtract(u_j, q_i_scaled, &temp_sub), cleanup, TAG, "Failed to subtract scaled Q column" );
            cla_matrix_delete(u_j);
            u_j = temp_sub;
            cla_matrix_delete(q_i_scaled);
            cla_matrix_delete(q_i);
        }

        double norm;
        ESP_GOTO_ON_ERROR( cla_matrix_get_column_l2norm(u_j, 0, &norm), cleanup, TAG, "Failed to compute L2 norm" );
        r->data[j][j] = norm;

        for (uint16_t i = 0; i < m_a->num_rows; i++) {
            q->data[i][j] = u_j->data[i][0] / norm;
        }
        cla_matrix_delete(u_j);
    }

    ESP_RETURN_ON_ERROR( cla_matrix_qr_create(q, r, m_qr_a), TAG, "Failed to create QR struct" );

cleanup:
    cla_matrix_delete(u);
    return ret;
}

esp_err_t cla_matrix_qr_solve(const cla_matrix_ptr_t m_a, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x) {
    esp_err_t ret = ESP_OK;

    ESP_ARG_CHECK(m_a && m_b);

    cla_matrix_qr_ptr_t qr = NULL;
    cla_matrix_ptr_t q_transpose = NULL;
    cla_matrix_ptr_t q_transpose_b = NULL;

    ESP_GOTO_ON_ERROR( cla_matrix_qr_get_decomposition(m_a, &qr), cleanup, TAG, "QR decomposition failed" );
    ESP_GOTO_ON_ERROR( cla_matrix_transpose(qr->q, &q_transpose), cleanup, TAG, "Failed to transpose Q" );
    ESP_GOTO_ON_ERROR( cla_matrix_multiply(q_transpose, m_b, &q_transpose_b), cleanup, TAG, "Failed to compute Q_transpose * b" );
    ESP_GOTO_ON_ERROR( cla_matrix_ls_solve_bck(qr->r, q_transpose_b, m_x), cleanup, TAG, "Failed to solve back substitution" );

cleanup:
    cla_matrix_qr_delete(qr);
    cla_matrix_delete(q_transpose);
    cla_matrix_delete(q_transpose_b);
    ret = (*m_x != NULL) ? ESP_OK : ESP_FAIL;
    return ret;
}




esp_err_t cla_matrix_ls_solve_fwd(const cla_matrix_ptr_t m_l, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x) {
    ESP_ARG_CHECK(m_l && m_b);
    ESP_RETURN_ON_ERROR( cla_matrix_create(m_l->num_cols, 1, m_x), TAG, "Unable to create matrix instance, create matrix failed" );
    for(uint16_t i = 0; i < m_l->num_cols; i++) {
        double tmp = m_b->data[i][0];
        for(uint16_t j = 0; j < i; j++) {
            tmp -= m_l->data[i][j] * (*m_x)->data[j][0];
        }
        (*m_x)->data[i][0] = tmp / m_l->data[i][i];
    }
    return ESP_OK;
}

esp_err_t cla_matrix_ls_solve_bck(const cla_matrix_ptr_t m_u, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x) {
    ESP_ARG_CHECK(m_u && m_b);
    ESP_RETURN_ON_ERROR( cla_matrix_create(m_u->num_rows, 1, m_x), TAG, "Unable to create matrix instance, create matrix failed" );
    for(int16_t i = m_u->num_cols - 1; i >= 0; i--) {
        double tmp = m_b->data[i][0];
        for(uint16_t j = i + 1; j < m_u->num_cols; j++) {
            tmp -= m_u->data[i][j] * (*m_x)->data[j][0];
        }
        (*m_x)->data[i][0] = tmp / m_u->data[i][i];
    }
    return ESP_OK;
}

esp_err_t cla_matrix_ls_solve(const cla_matrix_lup_ptr_t m_lup_lu, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x) {
    ESP_ARG_CHECK(m_lup_lu && m_b);
    ESP_RETURN_ON_FALSE( (m_lup_lu->u->num_rows == m_b->num_rows && m_b->num_cols == 1), ESP_ERR_INVALID_ARG, TAG, "Invalid matrix dimensions, number of rows in LUP decomposition must match number of rows in b matrix" );
    cla_matrix_ptr_t pb = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_get_dot_product(m_lup_lu->p, m_b, &pb), TAG, "Unable to compute matrix dot product, LUP permutation matrix and b matrix multiplication failed" );
    cla_matrix_ptr_t y = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_ls_solve_fwd(m_lup_lu->l, pb, &y), TAG, "Unable to solve forward substitution, LUP lower triangular matrix and permuted b matrix failed" );
    ESP_RETURN_ON_ERROR( cla_matrix_ls_solve_bck(m_lup_lu->u, y, m_x), TAG, "Unable to solve back substitution, LUP upper triangular matrix and intermediate y matrix failed" );
    cla_matrix_delete(pb);
    cla_matrix_delete(y);
    return ESP_OK;
}

esp_err_t cla_matrix_get_row_echelon_form(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_ref) {
    ESP_ARG_CHECK(m);
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m, m_ref), TAG, "Unable to copy matrix instance, copy matrix failed" );
    uint16_t j = 0;
    uint16_t i = 0;
    while(j < (*m_ref)->num_cols && i < (*m_ref)->num_rows) {
        // Find the pivot - the first non-zero entry in the first column of the matrix
        int16_t pivot_row = -1;
        ESP_RETURN_ON_ERROR( cla_matrix_get_pivot_idx(*m_ref, i, j, &pivot_row), TAG, "Unable to get pivot index, get pivot index failed" );
        if(pivot_row < 0) {
            j++;
            continue;
        }
        // We interchange rows moving the pivot to the first row that doesn't have already a pivot in place
        if(pivot_row != i) {
            ESP_RETURN_ON_ERROR( cla_matrix_swap_rows(i, pivot_row, m_ref), TAG, "Unable to swap rows, swap rows failed" );
        }
        // Multiply each element in the pivot row by the inverse of the pivot
        ESP_RETURN_ON_ERROR( cla_matrix_multiply_row(i, 1.0 / (*m_ref)->data[i][j], m_ref), TAG, "Unable to multiply row, multiply row failed" );
        // We add multiplies of the pivot so every element on the column equals 0
        for(uint16_t k = i+1; k < (*m_ref)->num_rows; k++) {
            if(fabs((*m_ref)->data[k][j]) > CLA_MATRIX_MIN_COEF) {
                ESP_RETURN_ON_ERROR( cla_matrix_add_scaled_row(i, k, -(*m_ref)->data[k][j], m_ref), TAG, "Unable to add scaled row, add scaled row failed" );
            }
        }
        i++;
        j++;
    }
    return ESP_OK;
}

esp_err_t cla_matrix_get_reduced_row_echelon_form(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_rref) {
    ESP_ARG_CHECK(m);
    cla_matrix_ptr_t m_echelon = NULL;
    ESP_RETURN_ON_ERROR( cla_matrix_get_row_echelon_form(m, &m_echelon), TAG, "Unable to get row echelon form, get row echelon form failed" );
    ESP_RETURN_ON_ERROR( cla_matrix_copy(m_echelon, m_rref), TAG, "Unable to copy matrix instance, copy matrix failed" );
    for (int16_t i = (*m_rref)->num_rows - 1; i >= 0; i--) {
        int16_t pivot_col = -1;
        for (int16_t j = 0; j < (*m_rref)->num_cols; j++) {
            if (fabs((*m_rref)->data[i][j]) > CLA_MATRIX_MIN_COEF) {
                pivot_col = j;
                break;
            }
        }
        if (pivot_col != -1) {
            for (int16_t k = i - 1; k >= 0; k--) {
                ESP_RETURN_ON_ERROR( cla_matrix_add_scaled_row(i, k, -(*m_rref)->data[k][pivot_col], m_rref), TAG, "Unable to add scaled row, add scaled row failed" );
            }
        }
    }
    cla_matrix_delete(m_echelon);
    return ESP_OK;
}