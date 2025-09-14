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
 * @file tcs3472.c
 *
 * ESP-IDF driver for TCS3472 RGB sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/tcs3472.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * TCS3472 I2C register definitions
*/

#define TCS3472_REG_ENABLE_RW       UINT8_C(0x00)   /*!< tcs3472 enable register */
#define TCS3472_REG_ATIME_RW        UINT8_C(0x01)   /*!< tcs3472 RGBC integration time register */
#define TCS3472_REG_WTIME_RW        UINT8_C(0x03)   /*!< tcs3472 wait time register */
#define TCS3472_REG_AILTL_RW        UINT8_C(0x04)   /*!< tcs3472 clear interrupt low threshold low byte register */
#define TCS3472_REG_AILTH_RW        UINT8_C(0x05)   /*!< tcs3472 clear interrupt low threshold high byte register */
#define TCS3472_REG_AIHTL_RW        UINT8_C(0x06)   /*!< tcs3472 clear interrupt high threshold low byte register */
#define TCS3472_REG_AIHTH_RW        UINT8_C(0x07)   /*!< tcs3472 clear interrupt high threshold high byte register */
#define TCS3472_REG_PERS_RW         UINT8_C(0x0C)   /*!< tcs3472 interrupt persistence filter register */
#define TCS3472_REG_CONFIG_RW       UINT8_C(0x0D)   /*!< tcs3472 configuration register */
#define TCS3472_REG_CONTROL_RW      UINT8_C(0x0F)   /*!< tcs3472 gain control register */
#define TCS3472_REG_ID_R            UINT8_C(0x12)   /*!< tcs3472 device ID register */
#define TCS3472_REG_STATUS_R        UINT8_C(0x13)   /*!< tcs3472 device status register */
#define TCS3472_REG_CDATAL_R        UINT8_C(0x14)   /*!< tcs3472 clear data low byte register */
#define TCS3472_REG_CDATAH_R        UINT8_C(0x15)   /*!< tcs3472 clear data high byte register */
#define TCS3472_REG_RDATAL_R        UINT8_C(0x16)   /*!< tcs3472 red data low byte register */
#define TCS3472_REG_RDATAH_R        UINT8_C(0x17)   /*!< tcs3472 red data high byte register */
#define TCS3472_REG_GDATAL_R        UINT8_C(0x18)   /*!< tcs3472 green data low byte register */
#define TCS3472_REG_GDATAH_R        UINT8_C(0x19)   /*!< tcs3472 green data high byte register */
#define TCS3472_REG_BDATAL_R        UINT8_C(0x1A)   /*!< tcs3472 blue data low byte register */
#define TCS3472_REG_BDATAH_R        UINT8_C(0x1B)   /*!< tcs3472 blue data high byte register */


#define TCS3472_POWERUP_DELAY_MS   UINT16_C(5)     /*!< tcs3472 delay on power-up before attempting I2C transactions */
#define TCS3472_APPSTART_DELAY_MS  UINT16_C(10)    /*!< tcs3472 delay after initialization before application start-up */
#define TCS3472_CMD_DELAY_MS       UINT16_C(5)     /*!< tcs3472 delay before attempting I2C transactions after a command is issued */
#define TCS3472_RETRY_DELAY_MS     UINT16_C(2)     /*!< tcs3472 delay between an I2C receive transaction retry */
#define TCS3472_TX_RX_DELAY_MS     UINT16_C(10)    /*!< tcs3472 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief TCS3472 device descriptor structure definition.
 */
typedef struct tcs3472_device_s {
    tcs3472_config_t            config;             /*!< tcs3472 device configuration */
    i2c_master_dev_handle_t     i2c_handle;         /*!< tcs3472 i2c device handle */
} tcs3472_device_t;

/*
* static constant declarations
*/
static const char *TAG = "tcs3472";




const char* tcs3472_get_fw_version(void) {
    return (const char*)TCS3472_FW_VERSION_STR;
}

int32_t tcs3472_get_fw_version_number(void) {
    return (int32_t)TCS3472_FW_VERSION_INT32;
}
