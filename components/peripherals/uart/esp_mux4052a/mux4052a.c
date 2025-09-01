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
 * @file mux4052a.c
 *
 * ESP-IDF driver for MUX4052A uart port multiplexer. The 4052A denotes the multiplexer type
 * (SN74LV4052A) and number of uart ports supported. The MUX4052A, or SN74LV4052A, is a 
 * 4-channel analog multiplexer and demultiplexer with three control pins.
 * 
 * https://github.com/MikroElektronika/mikrosdk_click_v2/blob/master/clicks/mux4/lib_mux4/src/mux4.c
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

/**
 * dependency includes
 */

#include "include/mux4052a.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define MUX4052A_XFR_TIMEOUT_MS         (500)           //!< uart transaction timeout in milliseconds
#define MUX4052A_RX_BUFFER_SIZE         (512)           /*!< uart receive maximum buffer size */
#define MUX4052A_GPIO_LEVEL_HI          UINT8_C(1)      /*!< gpio high level state */
#define MUX4052A_GPIO_LEVEL_LO          UINT8_C(0)      /*!< gpio low level state */

/**
 * macro definitions
 */

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief MUX4052A device descriptor structure definition.
 */
typedef struct mux4052a_device_s {
    mux4052a_config_t       config;             /*!< mux4052a device configuration */
    bool                    enabled;            /*!< mux4052a uart input state, input is enabled when true */
    mux4052a_channels_t     channel_state;      /*!< mux4052a channel number */
} mux4052a_device_t;

/**
 * static constant declarations
 */

static const char* TAG = "mux4052a";

/**
 * @brief Sets the uart channel for the MUX4052A device.
 * 
 * @param device MUX4052A device descriptor.
 * @param channel MUX4052A uart channel to set.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mux4052a_set_serial_port(mux4052a_device_t *const device, const mux4052a_channels_t channel) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    if(device->channel_state == channel) {
        return ESP_OK;
    }

    switch(channel) {
        case MUX4052A_CHANNEL_0:
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_a_io_num, MUX4052A_GPIO_LEVEL_LO), TAG, "set channel control a gpio low failed" );
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_b_io_num, MUX4052A_GPIO_LEVEL_LO), TAG, "set channel control b gpio low failed" );
            device->channel_state = MUX4052A_CHANNEL_0;
            break;
        case MUX4052A_CHANNEL_1:
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_a_io_num, MUX4052A_GPIO_LEVEL_HI), TAG, "set channel control a gpio high failed" );
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_b_io_num, MUX4052A_GPIO_LEVEL_LO), TAG, "set channel control b gpio low failed" );
            device->channel_state = MUX4052A_CHANNEL_1;
            break;
        case MUX4052A_CHANNEL_2:
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_a_io_num, MUX4052A_GPIO_LEVEL_LO), TAG, "set channel control a gpio low failed" );
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_b_io_num, MUX4052A_GPIO_LEVEL_HI), TAG, "set channel control b gpio high failed" );
            device->channel_state = MUX4052A_CHANNEL_2;
            break;
        case MUX4052A_CHANNEL_3:
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_a_io_num, MUX4052A_GPIO_LEVEL_HI), TAG, "set channel control a gpio high failed" );
            ESP_RETURN_ON_ERROR( gpio_set_level(device->config.ch_cntrl_b_io_num, MUX4052A_GPIO_LEVEL_HI), TAG, "set channel control b gpio high failed" );
            device->channel_state = MUX4052A_CHANNEL_3;
            break;
        default:
            ESP_RETURN_ON_ERROR( ESP_ERR_INVALID_ARG, TAG, "invalid mux uart port number" );
    }
    return ESP_OK;
}

/**
 * @brief Initializes MUX4052A GPIO pins and levels.
 * 
 * @param device MUX4052A device descriptor.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t mux4052a_gpio_init(mux4052a_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* mask bits for gpio pins */
    const uint64_t pin_bit_mask = ((1ULL << device->config.ch_inhbt_in_io_num) | 
                                (1ULL << device->config.ch_cntrl_a_io_num) |
                                (1ULL << device->config.ch_cntrl_b_io_num));

    /* set gpio configuration */
    const gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = pin_bit_mask,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE
    };
    ESP_RETURN_ON_ERROR( gpio_config(&io_conf), TAG, "Unable to configure gpio, gpio init failed" );

    /* set input gpio levels */
    if(device->config.ch_input_disabled == true) {
        /* initialize MUX4052A gpio levels */
        ESP_RETURN_ON_ERROR( mux4052a_disable((mux4052a_handle_t)device), TAG, "Unable to set inhibit input gpio level high, gpio init failed" );
    } else {
        /* initialize MUX4052A gpio levels */
        ESP_RETURN_ON_ERROR( mux4052a_enable((mux4052a_handle_t)device), TAG, "Unable to set inhibit input gpio level low, gpio init failed" );
    }

    //gpio_dump_io_configuration(stdout, pin_bit_mask);

    return ESP_OK;
}

/**
 * @brief Initializes MUX4052A UART.
 * 
 * @param device MUX4052A device descriptor.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t mux4052a_uart_init(mux4052a_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    // set serial port
    uart_config_t uart_config = (uart_config_t) { 
        .baud_rate  = device->config.uart_baud_rate,
        .data_bits  = device->config.uart_data_bits,
        .parity     = device->config.uart_parity,
        .stop_bits  = device->config.uart_stop_bits,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    /* configure uart */
    ESP_RETURN_ON_ERROR( uart_driver_install(device->config.uart_port, MUX4052A_RX_BUFFER_SIZE * 2, 0, 0, NULL, 0), TAG, "unable to install uart drive, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_param_config(device->config.uart_port, &uart_config), TAG, "unable to configure uart parameters, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_set_pin(device->config.uart_port, device->config.uart_tx_io_num, device->config.uart_rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "unable to set uart pins, uart enable failed");

    return ESP_OK;
}

esp_err_t mux4052a_init(const mux4052a_config_t *mux4052a_config, mux4052a_handle_t *const mux4052a_handle) {
    esp_err_t ret = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( mux4052a_config );

    /* validate memory availability for handle */
    mux4052a_device_t* dev = (mux4052a_device_t*)calloc(1, sizeof(mux4052a_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for MUX4052A handle, init failed");

    /* copy configuration */
    dev->config = *mux4052a_config;

    /* attempt to initialize gpio pins and levels */
    ESP_GOTO_ON_ERROR( mux4052a_gpio_init(dev), err, TAG, "unable to init gpio, init failed");

    /* attempt to initialize uart */
    ESP_GOTO_ON_ERROR( mux4052a_uart_init(dev), err, TAG, "unable to init uart, init failed");

    /* set output parameter */
    *mux4052a_handle = (mux4052a_handle_t)dev;

    return ESP_OK;
    err:
        return ret;
}

esp_err_t mux4052a_set_channel(mux4052a_handle_t handle, const mux4052a_channels_t channel) {
    mux4052a_device_t* dev = (mux4052a_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* set serial port */
    ESP_RETURN_ON_ERROR( mux4052a_set_serial_port(dev, channel), TAG, "Unable to set channel, set channel failed" );

    /* set port state */
    dev->channel_state = channel;

    return ESP_OK;
}

esp_err_t mux4052a_get_channel(mux4052a_handle_t handle, mux4052a_channels_t *const channel) {
    mux4052a_device_t* dev = (mux4052a_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* set port state */
    *channel = dev->channel_state;

    return ESP_OK;
}

esp_err_t mux4052a_enable(mux4052a_handle_t handle) {
    mux4052a_device_t* dev = (mux4052a_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* set inhibit input pin to high */
    ESP_RETURN_ON_ERROR( gpio_set_level(dev->config.ch_inhbt_in_io_num, MUX4052A_GPIO_LEVEL_LO), TAG, "Unable to set inhibit input gpio low, enable input failed" );

    /* set input state */
    dev->enabled = true;

    return ESP_OK;
}

esp_err_t mux4052a_disable(mux4052a_handle_t handle) {
    mux4052a_device_t* dev = (mux4052a_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* set inhibit input pin to low */
    ESP_RETURN_ON_ERROR( gpio_set_level(dev->config.ch_inhbt_in_io_num, MUX4052A_GPIO_LEVEL_HI), TAG, "Unable to set inhibit input gpio high, enable input failed" );

    /* set input state */
    dev->enabled = false;

    return ESP_OK;
}

esp_err_t mux4052a_delete(mux4052a_handle_t handle) {
    mux4052a_device_t* dev = (mux4052a_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* remove device from uart bus */
    ESP_RETURN_ON_ERROR( uart_driver_delete(dev->config.uart_port), TAG, "unable to remove device from uart bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* mux4052a_get_fw_version(void) {
    return (const char*)MUX4052A_FW_VERSION_STR;
}

int32_t mux4052a_get_fw_version_number(void) {
    return (int32_t)MUX4052A_FW_VERSION_INT32;
}