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
 * @file lin.h
 * @defgroup drivers lin
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
#ifndef __LIN_H__
#define __LIN_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/uart_periph.h>
//#include "lin_version.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LIN_MAX_FRAME_LENGTH 0x8u
#define LIN_SYNCH_BYTE 0x55u
#define LIN_GET_PID_BIT(x,y) (((x) >> (y)) & 0x01u)
#define LIN_ID_MASK 0x3Fu
#define LIN_P0_FLAG 6
#define LIN_P1_FLAG 7

#define LIN_DIAG_REQUEST  0x3C
#define LIN_DIAG_RESPONSE 0x3D

#define LIN_UART_TX_BUF_SIZE      (1024)                    /*!< uart tx buffer size */
#define LIN_UART_BAUD_RATE        (19200)                   /*!<  */
#define LIN_UART_DATA_BITS        UART_DATA_8_BITS          /*!<  */
#define LIN_UART_PARITY           UART_PARITY_DISABLE       /*!<  */
#define LIN_UART_STOP_BITS        UART_STOP_BITS_1          /*!<  */
#define LIN_UART_FLOW_CTRL        UART_HW_FLOWCTRL_DISABLE  /*!<  */

typedef uint8_t lin_pid_t;
typedef uint8_t lin_checksum_t;

typedef enum lin_version_e {
  LIN_V1            = 1,          //!< LIN protocol version 1
  LIN_V2            = 2           //!< LIN protocol version 2
} lin_version_t;

typedef enum lin_frame_types_e {
	LIN_FRAME_TYPE_TRANSMIT,
	LIN_FRAME_TYPE_RECEIVE
} lin_frame_types_t;

typedef enum lin_error_types_e {
	LIN_NO_ERROR,
	LIN_SLAVE_ERROR_INVALID_DATA_RX,
	LIN_SLAVE_ERROR_INVALID_CHECKSUM,
	LIN_SLAVE_ERROR_PID_PARITY,
	LIN_SLAVE_ERROR_INVALID_SYNCH,
	LIN_SLAVE_ERROR_INVALID_BREAK,
	LIN_SLAVE_ERROR_ID_NOT_FOUND,
	LIN_SLAVE_ERROR_HW_TX,
	LIN_MASTER_ERROR_CHECKSUM,
	LIN_MASTER_ERROR_HEADER_TX,
	LIN_MASTER_ERROR_DATA_TX,
	LIN_MASTER_ERROR_DATA_RX,
	LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT
} lin_error_types_t;


typedef struct lin_data_frame_s {
	lin_pid_t      pid; /**< frame identifier field t_open_lin_frame#pid. */
	uint8_t        length;
	uint8_t*       data_ptr;
	lin_checksum_t checksum;
} lin_data_frame_t;


lin_pid_t lin_get_parity_id(const lin_pid_t pid);

lin_checksum_t lin_get_checksum(const lin_version_t version, lin_pid_t id, uint8_t num_data, uint8_t *data);

void lin_error_handler(lin_error_types_t error_code);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __LIN_H__