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
 * @file mux4052a.h
 * @defgroup drivers mux4052a
 * @{
 *
 * ESP-IDF driver for mux4052a, a UART multiplexer based on the 74HC4052A analog multiplexer/demultiplexer.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MUX4052A_H__
#define __MUX4052A_H__

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
#include "mux4052a_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */
#define MUX4052A_PORT_NUM               UART_NUM_2          /*!<  */
#define MUX4052A_BAUD_RATE              (115200)            /*!<  */
#define MUX4052A_DATA_BITS              UART_DATA_8_BITS    /*!<  */
#define MUX4052A_PARITY                 UART_PARITY_DISABLE /*!<  */
#define MUX4052A_STOP_BITS              UART_STOP_BITS_1    /*!<  */
#define MUX4052A_TXD_IO_NUM             GPIO_NUM_17         /*!<  */
#define MUX4052A_RXD_IO_NUM             GPIO_NUM_16         /*!<  */
#define MUX4052A_INHBT_IN_IO_NUM        GPIO_NUM_18         /*!<  */
#define MUX4052A_CNTRL_A_IO_NUM         GPIO_NUM_4          /*!<  */
#define MUX4052A_CNTRL_B_IO_NUM         GPIO_NUM_13         /*!<  */


/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `mux4052a_config_t` to default configuration settings for the aht30 sensor type.
 */
#define MUX4052A_CONFIG_DEFAULT {              \
    .uart_port            = MUX4052A_PORT_NUM,         \
    .uart_baud_rate       = MUX4052A_BAUD_RATE,        \
    .uart_data_bits       = MUX4052A_DATA_BITS,        \
    .uart_parity          = MUX4052A_PARITY,           \
    .uart_stop_bits       = MUX4052A_STOP_BITS,        \
    .uart_tx_io_num       = MUX4052A_TXD_IO_NUM,       \
    .uart_rx_io_num       = MUX4052A_RXD_IO_NUM,       \
    .ch_cntrl_a_io_num    = MUX4052A_CNTRL_A_IO_NUM,   \
    .ch_cntrl_b_io_num    = MUX4052A_CNTRL_B_IO_NUM,   \
    .ch_inhbt_in_io_num   = MUX4052A_INHBT_IN_IO_NUM,  \
    .ch_input_disabled    = false }

/**
 * @brief MUX4052A channels enumerator.
 */
typedef enum mux4052a_channels_e {
    MUX4052A_CHANNEL_0 = 0b00,    /*!< mux4052a channel 0 (1Y0, 2Y0) */
    MUX4052A_CHANNEL_1 = 0b01,    /*!< mux4052a channel 1 (1Y1, 2Y1) */
    MUX4052A_CHANNEL_2 = 0b10,    /*!< mux4052a channel 2 (1Y2, 2Y2) */
    MUX4052A_CHANNEL_3 = 0b11,    /*!< mux4052a channel 3 (1Y3, 2Y3) */
} mux4052a_channels_t;

/**
 * @brief MUX4052A configuration structure definition.
 */
typedef struct mux4052a_config_s {
    uint16_t            uart_port;               /*!< mcu uart port number for mux4052a serial interface */
    int                 uart_baud_rate;          /*!< mcu uart baud rate for mux4052a serial interface */
    uart_word_length_t  uart_data_bits;          /*!< mcu uart data bits for mux4052a serial interface */
    uart_parity_t       uart_parity;             /*!< mcu uart parity bits for mux4052a serial interface */
    uart_stop_bits_t    uart_stop_bits;          /*!< mcu uart stop bits for mux4052a serial interface */
    gpio_num_t          uart_tx_io_num;          /*!< mcu uart transmit pin for mux4052a serial interface */
    gpio_num_t          uart_rx_io_num;          /*!< mcu uart receive pin for mux4052a serial interface */
    gpio_num_t          ch_cntrl_a_io_num;       /*!< mux4052a's channel control a pin */
    gpio_num_t          ch_cntrl_b_io_num;       /*!< mux4052a's channel control b pin */
    gpio_num_t          ch_inhbt_in_io_num;      /*!< mux4052a's channel inhibit input pin */
    bool                ch_input_disabled;       /*!< mux4052a's channel inhibit input pin initial state */
} mux4052a_config_t;


/**
 * @brief MUX4052A opaque handle structure definition.
 */
typedef void* mux4052a_handle_t;

/**
 * public function and subroutine declarations
 */



/**
 * @brief Initializes an MUX4052A device onto the I2C master bus.
 *
 * @param[in] mux4052a_config MUX4052A device configuration.
 * @param[out] mux4052a_handle MUX4052A device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mux4052a_init(const mux4052a_config_t *mux4052a_config, mux4052a_handle_t *const mux4052a_handle);

/**
 * @brief Sets the MUX4052A's channel.  Only one channel can be enabled at a time.
 * 
 * @param handle MUX4052A device handle.
 * @param channel MUX4052A's channel to configure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mux4052a_set_channel(mux4052a_handle_t handle, const mux4052a_channels_t channel);

/**
 * @brief Gets the MUX4052A's channel.
 * 
 * @param handle MUX4052A device handle.
 * @param channel MUX4052A's configured channel.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mux4052a_get_channel(mux4052a_handle_t handle, mux4052a_channels_t *const channel);

/**
 * @brief Enables the MUX4052A's channel control functionality and multiplexing.
 * 
 * @param handle MUX4052A device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mux4052a_enable(mux4052a_handle_t handle);

/**
 * @brief Disables the MUX4052A's channel control functionality and multiplexing.
 * 
 * @param handle MUX4052A device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mux4052a_disable(mux4052a_handle_t handle);

/**
 * @brief Removes the MUX4052A's uart and frees handle.
 * 
 * @param handle MUX4052A device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mux4052a_delete(mux4052a_handle_t handle);

/**
 * @brief Converts MUX4052A firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* MUX4052A firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* mux4052a_get_fw_version(void);

/**
 * @brief Converts MUX4052A firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t MUX4052A firmware version number.
 */
int32_t mux4052a_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __MUX4052A_H__
