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
 * @file lin.c
 *
 * ESP-IDF driver for LIN bus.
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/lin.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/**
 * static constant declarations
 */

static const char* TAG = "lin";



lin_pid_t lin_get_parity_id(const lin_pid_t pid) {
  uint8_t  result;       // result = protected ID
  uint8_t  tmp;       // temporary variable for calculating parity bits

  // copy (unprotected) ID
  result = pid;

  // protect ID  with parity bits
  result  = (uint8_t) (result & 0x3F);                                          // clear upper bit 6&7
  tmp  = (uint8_t) ((result ^ (result>>1) ^ (result>>2) ^ (result>>4)) & 0x01);       // -> pid[6] = PI0 = ID0^ID1^ID2^ID4
  result |= (uint8_t) (tmp << 6);
  tmp  = (uint8_t) (~((result>>1) ^ (result>>3) ^ (result>>4) ^ (result>>5)) & 0x01); // -> pid[6] = PI1 = ~(ID1^ID3^ID4^ID5)
  result |= (uint8_t) (tmp << 7);

  // return protected identifier
  return result;
} 


lin_checksum_t lin_get_checksum(const lin_version_t version, uint8_t id, uint8_t num_data, uint8_t *data) {
  uint16_t chk=0x00;

  // protect the ID
  id = lin_get_parity_id(id);

  // LIN2.x uses extended checksum which includes protected ID, i.e. including parity bits
  // LIN1.x uses classical checksum only over data bytes
  // Diagnostic frames with ID 0x3C and 0x3D/0x7D always use classical checksum (see LIN spec "2.3.1.5 Checksum")
  if (!((version == LIN_V1) || (id == 0x3C) || (id == 0x7D)))    // if version 2  & no diagnostic frames (0x3C=60 (PID=0x3C) or 0x3D=61 (PID=0x7D))
    chk = (uint16_t) id;

  // loop over data bytes
  for (uint8_t i = 0; i < num_data; i++) {
    chk += (uint16_t) (data[i]);
    if (chk>255)
      chk -= 255;
  }
  chk = (uint8_t)(0xFF - ((uint8_t) chk));   // bitwise invert

  // return frame checksum
  return chk;
}