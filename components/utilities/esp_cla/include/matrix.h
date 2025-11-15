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
 * @file matrix.h
 * @defgroup matrix
 * @{
 *
 * ESP-IDF compact linear algebra (cla) matrix library
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CLA_MATRIX_H__
#define __CLA_MATRIX_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_check.h>


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

/**
 * @brief Matrix structure definition.
 */
typedef struct cla_matrix_s {
    uint16_t num_rows;   /*!< number of rows in the matrix */
    uint16_t num_cols;   /*!< number of columns in the matrix */
    double **data;       /*!< 2 dimensional array for matrix rows and columns */
    bool is_square;     /*!< boolean flag indicating matrix is square */
} cla_matrix_t;

/**
 * @brief Pointer to a matrix structure.
 */
typedef cla_matrix_t* cla_matrix_ptr_t;

/**
 * @brief LUP decomposition matrix structure definition.
 */
typedef struct cla_matrix_lup_s {
    cla_matrix_ptr_t l; /*!< L matrix */
    cla_matrix_ptr_t u; /*!< U matrix */
    cla_matrix_ptr_t p; /*!< P matrix */
    uint16_t num_permutations;
} cla_matrix_lup_t;

/**
 * @brief Pointer to an LUP decomposition matrix structure.
 */
typedef cla_matrix_lup_t* cla_matrix_lup_ptr_t;

/**
 * @brief QR decomposition matrix structure definition.
 */
typedef struct cla_matrix_qr_s {
    cla_matrix_ptr_t q; /*!< Q matrix */
    cla_matrix_ptr_t r; /*!< R matrix */
} cla_matrix_qr_t;

/**
 * @brief Pointer to a QR decomposition matrix structure.
 */
typedef cla_matrix_qr_t* cla_matrix_qr_ptr_t;




/**
 * public function prototypes
 */

/**
 * @brief Instantiates a matrix by the number of rows and number of columns.
 * 
 * @param num_rows The number of rows (maximum rows supported is 32,767) that should be created for the matrix.
 * @param num_cols The number of columns (maximum columns supported is 32,767) that should be created for the matrix.
 * @param m An instantiated matrix structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_create(const uint16_t num_rows, const uint16_t num_cols, cla_matrix_ptr_t *const m);

/**
 * @brief Instantiates a square matrix by size.
 * 
 * @param size The size of the square matrix to create (num_rows == num_cols).
 * @param m_sqr An instantiated square matrix structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_create_square(const uint16_t size, cla_matrix_ptr_t *const m_sqr);

/**
 * @brief Instantiates an identity matrix by size.
 * 
 * @param size The size of the identity matrix to create (num_rows == num_cols).
 * @param m_eye An instantiated identity matrix structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_create_identity(const uint16_t size, cla_matrix_ptr_t *const m_eye);

/**
 * @brief Deletes a matrix instance to free memory.
 * 
 * @param m Matrix instance to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_delete(cla_matrix_ptr_t m);

/**
 * @brief Prints the matrix elements to the console.
 * 
 * @param m Matrix to print to the console.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_print(cla_matrix_ptr_t m);




/**
 * @brief Instantiates an LUP decomposition structure from lower, upper, and permutation matrices.
 * 
 * @param m_l Lower triangular matrix.
 * @param m_u Upper triangular matrix.
 * @param m_p Permutation matrix.
 * @param num_permutations Number of row permutations performed during LUP decomposition.
 * @param m_lup An instantiated LUP decomposition matrix structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_create(const cla_matrix_ptr_t m_l, const cla_matrix_ptr_t m_u, const cla_matrix_ptr_t m_p, const uint16_t num_permutations, cla_matrix_lup_ptr_t *const m_lup);

/**
 * @brief Deletes an LUP decomposition matrix instance to free memory.
 * 
 * @param m_lup LUP decomposition matrix instance to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_delete(cla_matrix_lup_ptr_t m_lup);

/**
 * @brief Prints the LUP matrix to the console.
 * 
 * @param m_lup LUP matrix to print to the console.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_print(cla_matrix_lup_ptr_t m_lup);


/**
 * @brief Solves a system of linear equations Ax = b using LUP decomposition.
 * 
 * @param m Matrix A in the equation Ax = b.
 * @param m_lup LUP decomposition of matrix A.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_solve(const cla_matrix_ptr_t m, cla_matrix_lup_ptr_t *const m_lup);

/**
 * @brief Calculates the inverse of an LUP decomposition matrix.
 * 
 * @param m_lup LUP decomposition matrix.
 * @param m_inverse Inverse matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_get_inverse(const cla_matrix_lup_ptr_t m_lup, cla_matrix_ptr_t *const m_inverse);

/**
 * @brief Gets the determinant of an LUP decomposition matrix.
 * 
 * @param m_lup LUP decomposition matrix.
 * @param determinant Determinant value result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_get_determinant(const cla_matrix_lup_ptr_t m_lup, double *const determinant);

/**
 * @brief Gets the combined L and U matrices from an LUP decomposition.
 * 
 * @param m_lup LUP decomposition matrix.
 * @param m_lu LU combined matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_get_lu(const cla_matrix_lup_ptr_t m_lup, cla_matrix_ptr_t *const m_lu);

/**
 * @brief Copies an LUP decomposition matrix from source to destination.
 * 
 * @param m_lup_src LUP decomposition source matrix to copy from.
 * @param m_lup_dst LUP decomposition destination matrix to copy to.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_lup_copy(const cla_matrix_lup_ptr_t m_lup_src, cla_matrix_lup_ptr_t *const m_lup_dst);



/**
 * @brief Instantiates a QR decomposition structure from orthogonal and upper triangular matrices.
 * 
 * @param m_q 
 * @param m_r 
 * @param m_qr An instantiated QR decomposition matrix structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_qr_create(const cla_matrix_ptr_t m_q, const cla_matrix_ptr_t m_r, cla_matrix_qr_ptr_t *const m_qr);

/**
 * @brief Deletes a QR decomposition matrix instance to free memory.
 * 
 * @param m_qr QR decomposition matrix instance to delete.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_qr_delete(cla_matrix_qr_ptr_t m_qr);

/**
 * @brief Prints the QR matrix to the console.
 * 
 * @param m_qr QR matrix to print to the console.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_qr_print(cla_matrix_qr_ptr_t m_qr);


/**
 * @brief Copies a QR decomposition matrix from source to destination.
 * 
 * @param m_qr_src QR decomposition source matrix to copy from.
 * @param m_qr_dst QR decomposition destination matrix to copy to.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_qr_copy(const cla_matrix_qr_ptr_t m_qr_src, cla_matrix_qr_ptr_t *const m_qr_dst);

/**
 * @brief Solves a system of linear equations Ax = b using QR decomposition.
 * 
 * @param m_a Matrix A in the equation Ax = b.
 * @param m_qr_a QR decomposition of matrix A.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_qr_get_decomposition(const cla_matrix_ptr_t m_a, cla_matrix_qr_ptr_t *const m_qr_a);

/**
 * @brief 
 * 
 * @param m_a Matrix A in the equation Ax = b.
 * @param m_b Matrix b in the equation Ax = b.
 * @param m_x Solution matrix x.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_qr_solve(const cla_matrix_ptr_t m_a, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x);



/**
 * @brief Adds two matrices together.
 * 
 * @param m1 Matrix 1.
 * @param m2 Matrix 2.
 * @param m_sum Sum of matrix 1 and matrix 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_add(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_sum);

/**
 * @brief Subtracts matrix 2 from matrix 1.
 * 
 * @param m1 Matrix 1.
 * @param m2 Matrix 2.
 * @param m_difference Difference of matrix 1 minus matrix 2. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_subtract(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_difference);

/**
 * @brief Multiplies two matrices together.
 * 
 * @param m1 Matrix 1.
 * @param m2 Matrix 2.
 * @param m_product Matrix product of matrix 1 and matrix 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_multiply(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_product);

esp_err_t cla_matrix_normalize(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_normalize);

/**
 * @brief Calculates the DOT product of two multi-dimensional matrices.
 * 
 * @param m1 Matrix 1.
 * @param m2 Matrix 2.
 * @param m_dot DOT product result matrix.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_dot_product(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_dot);

/**
 * @brief Calculates the DOT product between two column vectors stored as single columns in matrices.
 * 
 * @param m1 Matrix containing the first column vector.
 * @param m1_col_idx Column index of the first vector in matrix m1.
 * @param m2 Matrix containing the second column vector.
 * @param m2_col_idx Column index of the second vector in matrix m2.
 * @param dot DOT product result between the two column vectors.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_vector_dot_product(const cla_matrix_ptr_t m1, const uint16_t m1_col_idx, const cla_matrix_ptr_t m2, const uint16_t m2_col_idx, double *const dot);


/**
 * @brief Calculates the inverse of a matrix using Gauss-Jordan elimination.
 * 
 * @param m Matrix to invert.
 * @param m_inverse Inverse matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_inverse(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_inverse);

/**
 * @brief Transposes a matrix.
 * 
 * @param m Matrix to transpose.
 * @param m_transpose Transposed matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_transpose(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_transpose);

/**
 * @brief Calculates the trace of a matrix.
 * 
 * @param m Matrix to calculate trace from.
 * @param trace Matrix containing the trace value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_trace(const cla_matrix_ptr_t m, double *const trace);

/**
 * @brief Calculates the Cholesky decomposition of a matrix.
 * 
 * @param m Matrix to decompose.
 * @param m_cholesky Cholesky decomposition result matrix.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_cholesky_decomposition(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_cholesky);

/**
 * @brief Calculates the Eigen decomposition of a symmetric matrix.
 * 
 * @param m Matrix to decompose.
 * @param m_eigenvectors Resulting matrix of eigenvectors.
 * @param m_eigenvalues Resulting diagonal matrix of eigenvalues.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_eigen_decomposition(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_eigenvectors, cla_matrix_ptr_t *const m_eigenvalues);

/**
 * @brief Copies a matrix from source to destination.
 * 
 * @param m_src Source matrix to copy from.
 * @param m_dst Destination matrix to copy to.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_copy(const cla_matrix_ptr_t m_src, cla_matrix_ptr_t *const m_dst);

/**
 * @brief Scales matrix elements by a scalar value.
 * 
 * @param m Matrix to scale.
 * @param scalar Scaling factor.
 * @param m_scaled Scaled matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_scale(const cla_matrix_ptr_t m, const double scalar, cla_matrix_ptr_t *const m_scaled);

/**
 * @brief Checks if two matrices have equal dimensions.
 * 
 * @param m1 Matrix 1.
 * @param m2 Matrix 2.
 * @param is_equal True if dimensions are equal, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_is_dimension_equal(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, bool *const is_equal);

/**
 * @brief Checks if two matrices are equal in dimensions and element values.
 * 
 * @param m1 Matrix 1.
 * @param m2 Matrix 2.
 * @param tolerance Tolerance value for element comparison.
 * @param is_equal True if matrices are equal, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_is_equal(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, const double tolerance, bool *const is_equal);

/**
 * @brief Checks if a matrix is empty (has zero rows or zero columns).
 * 
 * @param m Matrix to check.
 * @param is_empty True if matrix is empty, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_is_empty(const cla_matrix_ptr_t m, bool *const is_empty);

/**
 * @brief Checks if a matrix is square (num_rows == num_cols).
 * 
 * @param m Matrix to check.
 * @param is_square True if matrix is square, false otherwise.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_is_square(const cla_matrix_ptr_t m, bool *const is_square);

/**
 * @brief Gets the value of a matrix element by row and column indexes.
 * 
 * @param row_idx Row index of the matrix element to get value from.
 * @param col_idx Column index of the matrix element to get value from.
 * @param m Matrix to get element value from.
 * @param value Element value retrieved from the matrix.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_value(const uint16_t row_idx, const uint16_t col_idx, const cla_matrix_ptr_t m, double *const value);

/**
 * @brief Sets the value of a matrix element by row and column indexes.
 * 
 * @param row_idx Row index of the matrix element to set value for.
 * @param col_idx Column index of the matrix element to set value for.
 * @param value Element value to set in the matrix.
 * @param m Matrix to set element value in.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_set_value(const uint16_t row_idx, const uint16_t col_idx, const double value, cla_matrix_ptr_t *const m);

/**
 * @brief Sets all elements of the matrix to a specified value.
 * 
 * @param value Value to set all matrix elements to.
 * @param m Matrix to set all element values in.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_set_values(const double value, cla_matrix_ptr_t *const m);

/**
 * @brief Sets all diagonal elements of the matrix to a specified value.
 * 
 * @param value Value to set all diagonal matrix elements to.
 * @param m Matrix to set diagonal element values in.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_set_diagonal_values(const double value, cla_matrix_ptr_t *const m);

/**
 * @brief Sets all elements of the matrix to 0.
 * 
 * @param m Matrix to set all element values to 0.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_zero_values(cla_matrix_ptr_t *const m);


/**
 * @brief Adds a column to the matrix.
 * 
 * @param m Matrix to add column on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_add_column(cla_matrix_ptr_t *const m);

/**
 * @brief Deletes a column from a matrix by column index.
 * 
 * @param col_idx Column index in the matrix to delete.
 * @param m Matrix to delete column from.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_delete_column(const uint16_t col_idx, cla_matrix_ptr_t *const m);

/**
 * @brief Swaps two columns from a matrix by column indexes.
 * 
 * @param col_idx1 Column index of the first column to swap with the second column index.
 * @param col_idx2 Column index of the second column to swap with the first column index..
 * @param m Matrix to swap columns on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_swap_column(const uint16_t col_idx1, const uint16_t col_idx2, cla_matrix_ptr_t *const m);

/**
 * @brief Swaps two data column values from a matrix.
 * 
 * @param m Matrix to swap column values on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_swap_columns(cla_matrix_ptr_t *const m);

/**
 * @brief Gets a column from a matrix by column index.
 * 
 * @param m Matrix to get column from.
 * @param col_idx Column index to get from the matrix.
 * @param m_col Column matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_column(const cla_matrix_ptr_t m, const uint16_t col_idx, cla_matrix_ptr_t *const m_col);

/**
 * @brief Calculates the L-2 norm of a column from a matrix by column index.
 * 
 * @param m Matrix to get column from.
 * @param col_idx Column index to get from the matrix.
 * @param col_norm L-2 norm result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_column_l2norm(const cla_matrix_ptr_t m, const uint16_t col_idx, double *const col_norm);

/**
 * @brief Calculates the L-2 norms of all columns from a matrix.
 * 
 * @param m Matrix to get all columns from.
 * @param m_norm L-2 norms of all columns result 1-row matrix.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_columns_l2norm(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_norm);

/**
 * @brief Multiplies a column by a scalar value in a matrix by column index.
 * 
 * @param col_idx Column index to multiply in the matrix.
 * @param scalar Scaling factor.
 * @param m Matrix to multiply column on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_multiply_column(const uint16_t col_idx, const double scalar, cla_matrix_ptr_t *const m);


/**
 * @brief Adds a scaled row to another row in a matrix by row and column indexes.
 * 
 * @param row_idx Row index to add to in the matrix.
 * @param row_idx_to_add Row index to scale and add from in the matrix. 
 * @param scalar Scaling factor.
 * @param m Matrix to add scaled row on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_add_scaled_row(const uint16_t row_idx, const uint16_t row_idx_to_add, const double scalar, cla_matrix_ptr_t *const m);

/**
 * @brief Adds a row in the matrix.
 * 
 * @param m Matrix to add row on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_add_row(cla_matrix_ptr_t *const m);

/**
 * @brief Deletes a row from a matrix by row index.
 * 
 * @param row_idx Row index in the matrix to delete.
 * @param m Matrix to delete row from.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_delete_row(const uint16_t row_idx, cla_matrix_ptr_t *const m);

/**
 * @brief Swaps two rows from a matrix by row indexes.
 * 
 * @param row_idx1 Row index of the first row to swap with the second row index.
 * @param row_idx2 Row index of the second row to swap with the first row index..
 * @param m Matrix to swap rows on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_swap_row(const uint16_t row_idx1, const uint16_t row_idx2, cla_matrix_ptr_t *const m);

/**
 * @brief Swaps two data row values from a matrix.
 * 
 * @param row_idx1 Row index of the first row to swap with the second row index.
 * @param row_idx2 Row index of the second row to swap with the first row index..
 * @param m Matrix to swap row values on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_swap_rows(const uint16_t row_idx1, const uint16_t row_idx2, cla_matrix_ptr_t *const m);

/**
 * @brief Gets a row from a matrix by row index.
 * 
 * @param m Matrix to get row from.
 * @param row_idx Row index to get from the matrix.
 * @param m_row Matrix row result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_row(const cla_matrix_ptr_t m, const uint16_t row_idx, cla_matrix_ptr_t *const m_row);

/**
 * @brief Divides a row by a divisor from a matrix by row index.
 * 
 * @param row_idx Row index to divide in the matrix.
 * @param divisor Divisor value.
 * @param m Matrix to divide row on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_divide_row(const uint16_t row_idx, const double divisor, cla_matrix_ptr_t *const m);

/**
 * @brief Multiplies a row by a scalar value in a matrix by row index.
 * 
 * @param row_idx Row index to multiply in the matrix.
 * @param scalar Scaling factor.
 * @param m Matrix to multiply row on.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_multiply_row(const uint16_t row_idx, const double scalar, cla_matrix_ptr_t *const m);


/**
 * @brief Forward substitution algorithm to solve lower triangular system of equations.
 * Solves the linear system L * x = b. This function is usually used with an L matrix 
 * from a LU decomposition.
 * 
 * In case L is not a lower triangular matrix, the algorithm will try to select only 
 * the lower triangular part of the matrix L and solve the system with it.
 * 
 * In case any of the diagonal elements (L[i][i]) are 0 the system cannot be solved.
 * 
 * @param m_l Lower triangular matrix of size n x n.
 * @param m_b Column matrix of size n x 1.
 * @param m_x Solution column matrix of size n x 1.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_ls_solve_fwd(const cla_matrix_ptr_t m_l, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x);

/**
 * @brief Back substitution algorithm to solve upper triangular system of equations.
 * Solves the linear system U *x = b. This function is usually used with an U matrix
 * from a LU decomposition.
 * 
 * In case U is not an upper triangular matrix, the algorithm will try to select 
 * only the upper triangular part of the matrix U and solve the system with it.
 * 
 * In case any of the diagonal elements (U[i][i]) are 0 the system cannot be solved.
 * 
 * @param m_u Upper triangular matrix of size n x n.
 * @param m_b Column matrix of size n x 1.
 * @param m_x Solution column matrix of size n x 1.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_ls_solve_bck(const cla_matrix_ptr_t m_u, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x);

/**
 * @brief Solves a system of linear equations Ax = b using LUP decomposition. 
 * 
 * A[n][n] is a square matrix, m contains matrices L, U, P for A[n][n] so that P*A = L*U.
 * 
 * The linear system is: A*x=b  =>  P*A*x = P*b  =>  L*U*x = P*b  => (where b is a matrix[n][1], and x is a matrix[n][1])
 * 
 * if y = U*x , we solve two systems:
 *   L * y = P b (forward substitution)
 *   U * x = y (backward substitution)
 * 
 * @param m_lup_lu 
 * @param m_b 
 * @param m_x 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_ls_solve(const cla_matrix_lup_ptr_t m_lup_lu, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x);


/**
 * @brief Calculates the row echelon form of a matrix using Gaussian elimination.
 * 
 * @param m Matrix to convert to row echelon form.
 * @param m_ref Row echelon form matrix result.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_row_echelon_form(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_ref);

/**
 * @brief Calculates the reduced row echelon form of a matrix using Gauss-Jordan elimination.
 * 
 * @param m Matrix to convert to reduced row echelon form. 
 * @param m_rref Reduced row echelon form matrix result. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t cla_matrix_get_reduced_row_echelon_form(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_rref);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CLA_MATRIX_H__
