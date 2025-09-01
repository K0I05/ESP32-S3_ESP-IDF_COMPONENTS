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
 * @file uuid.c
 *
 * ESP-IDF UUID generator
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

 #include "include/uuid.h"
 #include <string.h>
 #include <stdio.h>
 #include <stdarg.h>
 #include <esp_timer.h>
 
 /* constant definitions */
 #define UUID_RANDOM_SIZE    4
 #define UUID_BUFFER_SIZE    37
 #define UUID_HASH_MAX_SIZE  60
 #define UUID_ARGS_SIZE      2
 
 /* Marsaglia initializer 'constants' */
 static uint32_t uuid_m_w = 0;
 static uint32_t uuid_m_z = 0;
 /* uuid mode - variant-4 (default) */
 static uuid_modes_t uuid_mode = UUID_MODE_VARIANT4;
 
 /**
  * @brief Pseudo-random number generator (PRNG) using Marsaglia's MWC.
  *
  * @return uint32_t Random generated number.
  */
 static uint32_t uuid_random(void) {
     uuid_m_z = 36969 * (uuid_m_z & 65535) + (uuid_m_z >> 16);
     uuid_m_w = 18000 * (uuid_m_w & 65535) + (uuid_m_w >> 16);
     return (uuid_m_z << 16) + uuid_m_w;
 }
 
 /**
  * @brief Simple hash function for seeding.
  *
  * @param str Hash seeding string.
  * @return uint32_t Generated hash.
  */
 static uint32_t uuid_hash(const char *str) {
     uint32_t hash = 0;
     const char *p = str;
     while (*p != 0 && (p - str) < UUID_HASH_MAX_SIZE) {
         hash = hash * 2000099957 + (uint8_t)*p++;
     }
     return hash;
 }
 
 void uuid_init(void) {
     // Seed with compile-time constants and esp_timer_get_time()
     uint32_t s2 = uuid_hash(__TIME__) * (uint32_t)esp_timer_get_time();
     uint32_t s1 = s2 ^ uuid_hash(__DATE__);
     s2 ^= uuid_hash(__FILE__);
     uuid_seed(UUID_ARGS_SIZE, s1, s2);
     uuid_set_mode(UUID_MODE_VARIANT4);
 }
 
 void uuid_seed(uint8_t size, ...) {
     va_list args;
     va_start(args, size);
     if (size == 1) {
         uint32_t seed1 = va_arg(args, uint32_t);
         // Prevent 0 as seed value
         uuid_m_w = seed1 == 0 ? 1 : seed1;
         uuid_m_z = 2;
     } else if (size >= 2) {
         uint32_t seed1 = va_arg(args, uint32_t);
         uint32_t seed2 = va_arg(args, uint32_t);
         // Prevent 0 as seed value
         uuid_m_w = seed1 == 0 ? 1 : seed1;
         uuid_m_z = seed2 == 0 ? 2 : seed2;
     } else {
         // Default seed values
         uuid_m_w = 1;
         uuid_m_z = 2;
     }
     va_end(args);
 }
 
 const char *uuid_generate(void) {
     static char uuid_buffer[UUID_BUFFER_SIZE];
     uint32_t ar[UUID_RANDOM_SIZE];
 
     // Generate 4 random numbers
     for (int32_t i = 0; i < UUID_RANDOM_SIZE; i++) {
         ar[i] = uuid_random();
     }
 
     // Apply RFC-4122 version and variant bits
     if (uuid_mode == UUID_MODE_VARIANT4) {
         ar[1] = (ar[1] & 0xFFF0FFFF) | 0x00040000; // Version 4
         ar[2] = (ar[2] & 0x3FFFFFFF) | 0x80000000; // Variant 10xx
     }
 
     // Build the UUID string
     int j = 0;
     for (int i = 0; i < 16; i++) {
         if (i == 4 || i == 6 || i == 8 || i == 10) {
             uuid_buffer[j++] = '-';
         }
         uint8_t byte_index = i / 4;
         uint8_t byte = (ar[byte_index] >> ((i % 4) * 8)) & 0xFF;
         uuid_buffer[j++] = "0123456789abcdef"[(byte >> 4) & 0x0F];
         uuid_buffer[j++] = "0123456789abcdef"[byte & 0x0F];
     }
 
     uuid_buffer[UUID_BUFFER_SIZE - 1] = '\0';
     return (const char *)uuid_buffer;
 }
 
 void uuid_set_mode(const uuid_modes_t mode) {
     uuid_mode = mode;
 }
 
 uuid_modes_t uuid_get_mode(void) {
     return uuid_mode;
 }
 
 const char *uuid_get_fw_version(void) {
     return (const char*)UUID_FW_VERSION_STR;
 }
 
 int32_t uuid_get_fw_version_number(void) {
     return (int32_t)UUID_FW_VERSION_INT32;
 }
 