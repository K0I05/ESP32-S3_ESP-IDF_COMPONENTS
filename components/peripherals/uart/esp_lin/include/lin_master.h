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
 * @file lin_master.h
 * @defgroup drivers lin_master
 * @{
 *
 * ESP-IDF driver for LIN bus.
 * 
 * https://github.com/gicking/LIN_master_Arduino/blob/master/src/LIN_master.cpp
 * 
 * https://github.com/CW-B-W/open-LIN-c/blob/9aced2c4852af033dbf5e7f6dd6a34c71c919ee0/open_lin_types.h
 * 
 * 
 * https://esp32.com/viewtopic.php?t=46151
 * 
 * https://github.com/Ordspilleren/esp32-linak-desk-control/blob/main/main/main.c
 * 
 * 
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __LIN_MASTER_H__
#define __LIN_MASTER_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include "lin.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */
#define LIN_MASTER_PORT_NUM         UART_NUM_2          /*!<  */
#define LIN_MASTER_TXD_IO_NUM       GPIO_NUM_17         /*!<  */
#define LIN_MASTER_RXD_IO_NUM       GPIO_NUM_16         /*!<  */
#define LIN_MASTER_WAKE_IO_NUM      GPIO_NUM_18         /*!<  */


typedef struct lin_master_config_s {
    uint16_t            uart_port;             /*!< mcu uart port number for  serial interface */
    gpio_num_t          uart_tx_io_num;        /*!< mcu uart transmit pin for  serial interface */
    gpio_num_t          uart_rx_io_num;        /*!< mcu uart receive pin for  serial interface */
    gpio_num_t          uart_wake_io_num;      /*!<  */
} lin_master_config_t;

/**
 * @brief LIN master opaque handle structure definition.
 */
typedef void* lin_master_handle_t;



esp_err_t lin_master_init(const lin_master_config_t *lin_master_config, lin_master_handle_t *const lin_master_handle);








/**
 * @brief Removes the MUX4052A's uart and frees handle.
 * 
 * @param handle MUX4052A device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t lin_master_delete(lin_master_handle_t handle);

/**
 * @brief Converts MUX4052A firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* MUX4052A firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* lin_master_get_fw_version(void);

/**
 * @brief Converts MUX4052A firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t MUX4052A firmware version number.
 */
int32_t lin_master_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __LIN_MASTER_H__