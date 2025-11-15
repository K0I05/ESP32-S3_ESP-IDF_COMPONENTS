# ESP CLA - Matrix Library

This component provides a compact linear-algebra (CLA) matrix library for ESP-IDF-based projects. The API is declared in `matrix.h` and offers basic matrix creation/destruction, arithmetic, decompositions (LUP, QR), linear system solvers, and utility routines commonly used in numerical algorithms.

Header reference: components/esp_cla/include/matrix.h
(see: https://github.com/K0I05/ESP32-S3_CLA_COMPONENT_UNIT_TESTS/blob/4e139b94527ec610f46c432c4cbdc62a1f31e983/components/esp_cla/include/matrix.h)

Table of contents
- Overview
- Key types
- Important constraints / notes
- API quick reference
- Usage examples
- Integration / build notes
- Memory & error handling
- License

Overview
--------
The CLA matrix library exposes a lightweight matrix abstraction (cla_matrix_t) and helper types for LUP and QR decompositions. It is intended for embedded use with ESP-IDF and returns `esp_err_t` codes (ESP_OK on success). Most operations allocate or return new matrix objects and therefore require explicit deletion to avoid leaks.

Key types
---------
- cla_matrix_t
  - uint16_t num_rows
  - uint16_t num_cols
  - double **data  (2D row-major array)
  - bool is_square
- typedef cla_matrix_t* cla_matrix_ptr_t

- LUP:
  - cla_matrix_lup_t { l, u, p (cla_matrix_ptr_t), uint16_t num_permutations }
  - typedef cla_matrix_lup_t* cla_matrix_lup_ptr_t

- QR:
  - cla_matrix_qr_t { q, r (cla_matrix_ptr_t) }
  - typedef cla_matrix_qr_t* cla_matrix_qr_ptr_t

Important constraints / notes
---------------------------
- Many functions return `esp_err_t` and document `ESP_OK` on success. See `esp_err.h` for other error codes.
- Matrices use `double` elements.
- Caller is typically responsible for deleting matrix objects created by create/copy/get functions using `cla_matrix_delete`, `cla_matrix_lup_delete`, or `cla_matrix_qr_delete`.
- Several routines expect square matrices where noted (e.g., decompositions, some solvers).
- The header documents a maximum index type of `uint16_t` (so values up to 65535 can be represented), though larger matrices may be impractical on embedded hardware due to memory.

API quick reference
-------------------
This is a condensed reference grouped by purpose. See `matrix.h` for full function parameter descriptions.

Matrix lifecycle
- esp_err_t cla_matrix_create(uint16_t num_rows, uint16_t num_cols, cla_matrix_ptr_t *const m)
- esp_err_t cla_matrix_create_square(uint16_t size, cla_matrix_ptr_t *const m_sqr)
- esp_err_t cla_matrix_create_identity(uint16_t size, cla_matrix_ptr_t *const m_eye)
- esp_err_t cla_matrix_delete(cla_matrix_ptr_t m)
- esp_err_t cla_matrix_print(cla_matrix_ptr_t m)

LUP decomposition helpers
- esp_err_t cla_matrix_lup_create(const cla_matrix_ptr_t m_l, const cla_matrix_ptr_t m_u, const cla_matrix_ptr_t m_p, uint16_t num_permutations, cla_matrix_lup_ptr_t *const m_lup)
- esp_err_t cla_matrix_lup_delete(cla_matrix_lup_ptr_t m_lup)
- esp_err_t cla_matrix_lup_print(cla_matrix_lup_ptr_t m_lup)
- esp_err_t cla_matrix_lup_solve(const cla_matrix_ptr_t m, cla_matrix_lup_ptr_t *const m_lup)
- esp_err_t cla_matrix_lup_get_inverse(const cla_matrix_lup_ptr_t m_lup, cla_matrix_ptr_t *const m_inverse)
- esp_err_t cla_matrix_lup_get_determinant(const cla_matrix_lup_ptr_t m_lup, double *const determinant)
- esp_err_t cla_matrix_lup_get_lu(const cla_matrix_lup_ptr_t m_lup, cla_matrix_ptr_t *const m_lu)
- esp_err_t cla_matrix_lup_copy(const cla_matrix_lup_ptr_t m_lup_src, cla_matrix_lup_ptr_t *const m_lup_dst)

QR decomposition helpers
- esp_err_t cla_matrix_qr_create(const cla_matrix_ptr_t m_q, const cla_matrix_ptr_t m_r, cla_matrix_qr_ptr_t *const m_qr)
- esp_err_t cla_matrix_qr_delete(cla_matrix_qr_ptr_t m_qr)
- esp_err_t cla_matrix_qr_print(cla_matrix_qr_ptr_t m_qr)
- esp_err_t cla_matrix_qr_copy(const cla_matrix_qr_ptr_t m_qr_src, cla_matrix_qr_ptr_t *const m_qr_dst)
- esp_err_t cla_matrix_qr_get_decomposition(const cla_matrix_ptr_t m_a, cla_matrix_qr_ptr_t *const m_qr_a)
- esp_err_t cla_matrix_qr_solve(const cla_matrix_ptr_t m_a, const cla_matrix_ptr_t m_b, cla_matrix_ptr_t *const m_x)

Arithmetic / algebra
- esp_err_t cla_matrix_add(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_sum)
- esp_err_t cla_matrix_subtract(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_difference)
- esp_err_t cla_matrix_multiply(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_product)
- esp_err_t cla_matrix_scale(const cla_matrix_ptr_t m, const double scalar, cla_matrix_ptr_t *const m_scaled)
- esp_err_t cla_matrix_normalize(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_normalize)
- esp_err_t cla_matrix_get_dot_product(const cla_matrix_ptr_t m1, const cla_matrix_ptr_t m2, cla_matrix_ptr_t *const m_dot)
- esp_err_t cla_matrix_get_vector_dot_product(const cla_matrix_ptr_t m1, uint16_t m1_col_idx, const cla_matrix_ptr_t m2, uint16_t m2_col_idx, double *const dot)

Matrix transforms / properties
- esp_err_t cla_matrix_get_inverse(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_inverse)
- esp_err_t cla_matrix_transpose(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_transpose)
- esp_err_t cla_matrix_get_trace(const cla_matrix_ptr_t m, double *const trace)
- esp_err_t cla_matrix_get_cholesky_decomposition(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_cholesky)
- esp_err_t cla_matrix_get_eigen_decomposition(const cla_matrix_ptr_t m, cla_matrix_ptr_t *const m_eigenvectors, cla_matrix_ptr_t *const m_eigenvalues)
- esp_err_t cla_matrix_copy(const cla_matrix_ptr_t m_src, cla_matrix_ptr_t *const m_dst)

Row & column helpers
- Creation / deletion:
  - cla_matrix_add_column / cla_matrix_delete_column
  - cla_matrix_add_row / cla_matrix_delete_row
- Access / extract:
  - cla_matrix_get_column, cla_matrix_get_row
- Operations:
  - cla_matrix_get_column_l2norm, cla_matrix_get_columns_l2norm
  - cla_matrix_multiply_column, cla_matrix_multiply_row, cla_matrix_divide_row
  - cla_matrix_add_scaled_row
  - cla_matrix_swap_column / cla_matrix_swap_columns
  - cla_matrix_swap_row / cla_matrix_swap_rows

Linear solvers & forms
- Forward / backward substitution:
  - cla_matrix_ls_solve_fwd, cla_matrix_ls_solve_bck
- LUP-based solver:
  - cla_matrix_ls_solve (accepts a cla_matrix_lup_ptr_t and a right-hand side)
- Row echelon / reduced row echelon:
  - cla_matrix_get_row_echelon_form
  - cla_matrix_get_reduced_row_echelon_form

Utility checks / setters
- cla_matrix_is_dimension_equal, cla_matrix_is_equal
- cla_matrix_is_empty, cla_matrix_is_square
- cla_matrix_get_value, cla_matrix_set_value
- cla_matrix_set_values (fill)
- cla_matrix_set_diagonal_values
- cla_matrix_zero_values

Usage examples
--------------
Minimal example: create a 2x2 matrix, set values, multiply, print and cleanup.

```c
#include "matrix.h"
#include <stdio.h>

void example_matrix_multiply(void)
{
    cla_matrix_ptr_t a = NULL;
    cla_matrix_ptr_t b = NULL;
    cla_matrix_ptr_t prod = NULL;

    // create matrices
    if (cla_matrix_create(2, 2, &a) != ESP_OK) return;
    if (cla_matrix_create(2, 2, &b) != ESP_OK) { cla_matrix_delete(a); return; }

    // set values (row, col, value)
    cla_matrix_set_value(0, 0, 1.0, &a);
    cla_matrix_set_value(0, 1, 2.0, &a);
    cla_matrix_set_value(1, 0, 3.0, &a);
    cla_matrix_set_value(1, 1, 4.0, &a);

    cla_matrix_set_value(0, 0, 5.0, &b);
    cla_matrix_set_value(0, 1, 6.0, &b);
    cla_matrix_set_value(1, 0, 7.0, &b);
    cla_matrix_set_value(1, 1, 8.0, &b);

    // multiply
    if (cla_matrix_multiply(a, b, &prod) == ESP_OK) {
        cla_matrix_print(prod);
        cla_matrix_delete(prod);
    }

    // cleanup
    cla_matrix_delete(a);
    cla_matrix_delete(b);
}
```

Solving linear systems (high level)
----------------------------------
- Perform LUP or QR decomposition (provided functions exist to create and manage LUP/QR structures).
- Use forward/backward substitution (cla_matrix_ls_solve_fwd / cla_matrix_ls_solve_bck) or higher-level routines (cla_matrix_ls_solve, cla_matrix_qr_solve).

Integration / build notes
-------------------------
- This component is written to integrate into an ESP-IDF project. Add this component directory to your project's components/ directory or add the component path in CMakeLists as needed.
- Include header: #include "matrix.h" or with full include path if your project requires: #include "components/esp_cla/include/matrix.h"
- Link against the component as usual in your ESP-IDF project. If using CMake, ensure the component's CMakeLists.txt exports include paths and is found by the project.

Memory & error handling
-----------------------
- Many functions allocate memory. Always free returned matrices and decomposition structures with the corresponding delete functions:
  - cla_matrix_delete, cla_matrix_lup_delete, cla_matrix_qr_delete
- Functions return `esp_err_t`. On success they return `ESP_OK`. Check return values before using output pointers.
- The library uses `double` which increases memory usage on embedded targets — be mindful of available RAM.

License
-------
- The library is MIT licensed (see header in `matrix.h`).

Where to go next
----------------
- Read `matrix.h` for detailed parameter descriptions.
- Try the example above and add unit tests to exercise edge cases (empty matrices, non-square inputs where square required, singular matrices for inversion, etc.).
- If you’d like, I can prepare sample unit tests or a small example application demonstrating decomposition & solver flows.

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
