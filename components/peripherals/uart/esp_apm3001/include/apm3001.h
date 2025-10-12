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
 * @file apm3001.h
 * @defgroup drivers apm3001
 * @{
 *
 * ESP-IDF driver for apm3001.
 * 
 * https://github.com/zapzrat/PMS7003/tree/main
 * 
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __APM3001_H__
#define __APM3001_H__

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

#ifdef __cplusplus
extern "C" {
#endif


#define APM3001_UART_TX_BUF_SIZE      (1024)                    /*!< uart tx buffer size */
#define APM3001_UART_BAUD_RATE        (19200)                   /*!<  */
#define APM3001_UART_DATA_BITS        UART_DATA_8_BITS          /*!<  */
#define APM3001_UART_PARITY           UART_PARITY_DISABLE       /*!<  */
#define APM3001_UART_STOP_BITS        UART_STOP_BITS_1          /*!<  */
#define APM3001_UART_FLOW_CTRL        UART_HW_FLOWCTRL_DISABLE  /*!<  */

#define APM3001_FRAME_MSG_BEGIN       (0x424D)    /*!< start of message */


typedef enum apm3001_command_types_e {
    APM3001_CMD_SET_MODE    = 0xE1,
    APM3001_CMD_READ_MODE   = 0xE2,
    APM3001_CMD_POWER_MODE  = 0xE4,
} apm3001_command_types_t;


typedef union apm3001_tx_packet_u {
    union apm3001_frame_u {
        struct apm3001_datagram_s {
            struct apm3001_header_s {
                union apm3001_msg_begin_u {
                    uint16_t value;
                    uint8_t  bytes[2];
                } msg_begin;
            } header;
            struct apm3001_body_s {
                union apm3001_command_u {
                    apm3001_command_types_t command;
                    uint8_t                 reserved;
                    uint8_t                 option;
                    uint8_t                 bytes[3];
                } command;
            } body;
        } datagram;
        uint8_t bytes[5];
        union apm3001_checksum_u {
            uint16_t value;
            uint8_t  bytes[2];
        } checksum;
    } frame;
    uint8_t bytes[7];
} apm3001_tx_packet_t;


typedef union apm3001_rx_packet_u {
    struct apm3001_frame_s {
        struct apm3001_header_s {
            union apm3001_msg_begin_u {
                uint16_t value;
                uint8_t  bytes[2];
            } msg_begin;
            union apm3001_msg_length_u {
                uint16_t value;
                uint8_t  bytes[2];
            } msg_length;
        } header;
        struct apm3001_body_s {
            union apm3001_pm01_tsi_u {
                uint16_t value;
                uint8_t  bytes[2];
            } pm01_tsi;
            union apm3001_pm25_tsi_u {
                uint16_t value;
                uint8_t  bytes[2];
            } pm25_tsi;
            union apm3001_pm10_tsi_u {
                uint16_t value;
                uint8_t  bytes[2];
            } pm10_tsi;
            union apm3001_pm01_std_u {
                uint16_t value;
                uint8_t  bytes[2];
            } pm01_std;
            union apm3001_pm25_std_u {
                uint16_t value;
                uint8_t  bytes[2];
            } pm25_std;
            union apm3001_pm10_std_u {
                uint16_t value;
                uint8_t  bytes[2];
            } pm10_std;
            union apm3001_reserved1_u {
                uint16_t value;
                uint8_t  bytes[2];
            } reserved1;
            union apm3001_reserved2_u {
                uint16_t value;
                uint8_t  bytes[2];
            } reserved2;
            union apm3001_reserved3_u {
                uint16_t value;
                uint8_t  bytes[2];
            } reserved3;
            union apm3001_checksum_u {
                uint16_t value;
                uint8_t  bytes[2];
            } checksum;
        } body;
    } frame;
    uint8_t bytes[24];
} apm3001_rx_packet_t;


static inline void apm3001_rx_packet_init(apm3001_rx_packet_t *const packet) {
    if (packet) {
        for (size_t i = 0; i < sizeof(apm3001_rx_packet_t); i++) {
            packet->bytes[i] = 0;
        }

       // packet->frame.body.pm01_tsi.value 
    }
}

static inline void apm3001_tx_packet_init(const apm3001_command_types_t command, const uint8_t option, apm3001_tx_packet_t *const packet) {
    if (packet) {
        for (size_t i = 0; i < sizeof(apm3001_tx_packet_t); i++) {
            packet->bytes[i] = 0;
        }

       packet->frame.datagram.header.msg_begin.value = APM3001_FRAME_MSG_BEGIN;
       packet->frame.datagram.body.command.command   = command;
       packet->frame.datagram.body.command.reserved  = 0x00;
       packet->frame.datagram.body.command.option    = option;

       packet->frame.checksum.value = 0x00;
    }
}


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __APM3001_H__