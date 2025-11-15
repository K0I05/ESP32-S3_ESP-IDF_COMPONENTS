# ESP CLA — Vector Library

This README documents the public API declared in components/esp_cla/include/vector.h for the ESP-IDF compact linear-algebra (CLA) vector utilities.

Header: components/esp_cla/include/vector.h  
Source: <https://github.com/K0I05/ESP32-S3_CLA_COMPONENT_UNIT_TESTS/blob/4e139b94527ec610f46c432c4cbdc62a1f31e983/components/esp_cla/include/vector.h>

## Overview

The CLA vector library provides a lightweight vector abstraction and basic vector operations intended for embedded use. The API uses `esp_err_t` return codes (ESP_OK on success). Most operations return newly allocated vectors via output parameters — free them with `cla_vector_delete`.

Primary capabilities:

- Vector lifecycle: create, delete, print
- Component-wise arithmetic: add, subtract, multiply, divide
- Products: dot (multi-dimensional), cross (3D)
- Normalization, scaling, copying
- Component manipulation: add/delete components
- Utilities: get/set values, equality/dimension checks, 2D/3D checks

## Key types

- cla_vector_t
  - uint16_t num_cmps
  - double *data
  - bool is_2d
  - bool is_3d

- cla_vector_ptr_t — pointer to cla_vector_t

## Important notes & constraints

- Header documents a maximum of 128 components.
- Components are `double` — use caution with memory on embedded devices.
- Many functions allocate results; always delete returned vectors with `cla_vector_delete`.
- Dot product API returns a vector in the header (often a single-component vector) — check implementation semantics before assuming it returns a scalar.
- Cross product is defined for 3-component vectors only.
- Indexing and sizes use `uint16_t`.

## API quick reference

Lifecycle

- cla_vector_create(uint16_t num_cmps, cla_vector_ptr_t *v)
- cla_vector_delete(cla_vector_ptr_t v)
- cla_vector_print(cla_vector_ptr_t v)

Arithmetic

- cla_vector_add, cla_vector_subtract, cla_vector_multiply, cla_vector_divide

Products & normalization

- cla_vector_get_dot_product(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *v_dot)
- cla_vector_get_cross_product(const cla_vector_ptr_t v1, const cla_vector_ptr_t v2, cla_vector_ptr_t *v_cross)  // 3D
- cla_vector_normalize(const cla_vector_ptr_t v, cla_vector_ptr_t *v_normalized)
- cla_vector_scale(const cla_vector_ptr_t v, double scalar, cla_vector_ptr_t *v_scaled)

Accessors & checks

- cla_vector_copy
- cla_vector_get_value, cla_vector_set_value
- cla_vector_set_values, cla_vector_zero_values
- cla_vector_is_dimension_equal, cla_vector_is_equal (with tolerance)
- cla_vector_is_empty, cla_vector_is_2d, cla_vector_is_3d

Component manipulation

- cla_vector_add_component
- cla_vector_delete_component(uint16_t cmp_idx)

Iterator (commented)

- The header contains a commented iterator API sketch (begin, end, next, previous, etc.). That sketch is not required for basic use but can be implemented if needed.

## Example snippet

Create two 3-component vectors, compute dot & cross, normalize, and cleanup:

```c
cla_vector_ptr_t a = NULL, b = NULL, dot = NULL, cross = NULL, an = NULL;

if (cla_vector_create(3, &a) != ESP_OK) return;
if (cla_vector_create(3, &b) != ESP_OK) { cla_vector_delete(a); return; }

cla_vector_set_value(0, 1.0, &a);
cla_vector_set_value(1, 0.0, &a);
cla_vector_set_value(2, 0.0, &a);

cla_vector_set_value(0, 0.0, &b);
cla_vector_set_value(1, 1.0, &b);
cla_vector_set_value(2, 0.0, &b);

if (cla_vector_get_dot_product(a, b, &dot) == ESP_OK) {
    cla_vector_print(dot); // dot may be returned as a 1-component vector
    cla_vector_delete(dot);
}

if (cla_vector_get_cross_product(a, b, &cross) == ESP_OK) {
    cla_vector_print(cross);
    cla_vector_delete(cross);
}

if (cla_vector_normalize(a, &an) == ESP_OK) {
    cla_vector_print(an);
    cla_vector_delete(an);
}

cla_vector_delete(a);
cla_vector_delete(b);
```

## Memory & error handling

- Check `esp_err_t` return values on every call.
- Free all returned vectors with `cla_vector_delete`.
- `double` component type increases RAM usage — keep vector sizes small when possible.
- Expect error returns for invalid indices, mismatched dimensions, division-by-zero, or attempts to cross non-3D vectors.

## Best practices & tests

- Always check dimension compatibility before arithmetic operations (use cla_vector_is_dimension_equal).
- For cross product, ensure vectors have exactly 3 components.
- For normalization, guard against zero-length vectors.
- Unit tests to add: dimension mismatch errors, dot/cross product correctness, normalization of unit/non-unit vectors, division-by-zero handling, component add/delete behavior.

## License

Refer to the header of vector.h — the code is MIT licensed.

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
