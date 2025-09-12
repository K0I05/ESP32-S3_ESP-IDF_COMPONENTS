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
 * @file vl53l4cx.c
 *
 * ESP-IDF driver for VL53L4CX time of flight sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/vl53l4cx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * VL53L4CX definitions
*/


#define VL53L4CX_POWERUP_DELAY_MS   UINT16_C(5)     /*!< vl53l4cx delay on power-up before attempting I2C transactions */
#define VL53L4CX_APPSTART_DELAY_MS  UINT16_C(10)    /*!< vl53l4cx delay after initialization before application start-up */
#define VL53L4CX_CMD_DELAY_MS       UINT16_C(5)     /*!< vl53l4cx delay before attempting I2C transactions after a command is issued */
#define VL53L4CX_RETRY_DELAY_MS     UINT16_C(2)     /*!< vl53l4cx delay between an I2C receive transaction retry */
#define VL53L4CX_TX_RX_DELAY_MS     UINT16_C(10)    /*!< vl53l4cx delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief VL53L4CX device descriptor structure definition.
 */
typedef struct vl53l4cx_device_s {
    vl53l4cx_config_t                       config;                 /*!< vl53l4cx device configuration */
    i2c_master_dev_handle_t                 i2c_handle;             /*!< vl53l4cx i2c device handle */
} vl53l4cx_device_t;

/*
* static constant declarations
*/
static const char *TAG = "vl53l4cx";



const char* vl53l4cx_get_fw_version(void) {
    return (const char*)VL53L4CX_FW_VERSION_STR;
}

int32_t vl53l4cx_get_fw_version_number(void) {
    return (int32_t)VL53L4CX_FW_VERSION_INT32;
}