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
 * @file 3d_math.h
 * @defgroup math 3d_math
 * @{
 *
 * ESP-IDF 3d math library
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MATH3D_H__
#define __MATH3D_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
typedef struct {
    float w;
    float x;
    float y;
    float z;
    void init(void) {
        w = 0.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
    void init(float nw, float nx, float ny, float nz) {
        w = nw;
        x = nx;
        y = ny;
        z = nz;
    }
    quaternion_t get_product(quaternion_t q) {
        quaternion_t product;
        // Quaternion multiplication is defined by:
        //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)
        product.init(
            w*q.w - x*q.x - y*q.y - z*q.z,  // new w
            w*q.x + x*q.w + y*q.z - z*q.y,  // new x
            w*q.y - x*q.z + y*q.w + z*q.x,  // new y
            w*q.z + x*q.y - y*q.x + z*q.w); // new z
        return product;
    }
    quaternion_t get_conjugate() {
        quaternion_t conjugate;
        conjugate.init(w, -x, -y, -z);
        return conjugate;
    }
    float get_magnitude(void) {
        return sqrt(w*w + x*x + y*y + z*z);
    }
    void normalize(void) {
        float m = get_magnitude();
        w /= m;
        x /= m;
        y /= m;
        z /= m;
    }
    quaternion_t get_normalized(void) {
        quaternion_t normalized;
        normalized.init(w, x, y, z);
        normalized.normalize();
        return normalized;
    }
} quaternion_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    void init(void) {
        x = 0;
        y = 0;
        z = 0;
    }
    void init(int16_t nx, int16_t ny, int16_t nz) {
        x = nx;
        y = ny;
        z = nz;
    }
    float get_magnitude() {
        return sqrt(x*x + y*y + z*z);
    }
    void normalize() {
        float m = get_magnitude();
        x /= m;
        y /= m;
        z /= m;
    }
    vector_int16_t get_normalized() {
        vector_int16_t normalized;
        normalized.init(x, y, z);
        normalized.normalize();
        return normalized;
    }
    void rotate(quaternion_t q) {
        quaternion_t p;
        p.init(0, x, y, z);
        // quaternion multiplication: q * p, stored back in p
        p = q.get_product(p);
        // quaternion multiplication: p * conj(q), stored back in p
        p = p.get_product(q.get_conjugate());
        // p quaternion is now [0, x', y', z']
        x = p.x;
        y = p.y;
        z = p.z;
    }
    vector_int16_t get_rotated(quaternion_t q) {
        vector_int16_t r;
        r.init(x, y, z);
        r.rotate(q);
        return r;
    }
} vector_int16_t;

typedef struct {
    float x;
    float y;
    float z;
    void init(void) {
        x = 0;
        y = 0;
        z = 0;
    }
    void init(float nx, float ny, float nz) {
        x = nx;
        y = ny;
        z = nz;
    }
    float get_magnitude() {
        return sqrt(x*x + y*y + z*z);
    }
    void normalize() {
        float m = get_magnitude();
        x /= m;
        y /= m;
        z /= m;
    }
    vector_float_t get_normalized() {
        vector_float_t normalized;
        normalized.init(x, y, z);
        normalized.normalize();
        return normalized;
    }
    void rotate(quaternion_t q) {
        quaternion_t p;
        p.init(0, x, y, z);
        // quaternion multiplication: q * p, stored back in p
        p = q.get_product(p);
        // quaternion multiplication: p * conj(q), stored back in p
        p = p.get_product(q.get_conjugate());
        // p quaternion is now [0, x', y', z']
        x = p.x;
        y = p.y;
        z = p.z;
    }
    vector_float_t get_rotated(quaternion_t q) {
        vector_float_t r;
        r.init(x, y, z);
        r.rotate(q);
        return r;
    }
} vector_float_t;
*/

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MATH3D_H__
