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
 * @file lin_master.c
 *
 * ESP-IDF driver for LIN bus.
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/lin_master.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


typedef struct lin_master_context_s {
    lin_master_config_t     config;       /*!<  configuration */
    QueueHandle_t           uart_queue_handle;
    SemaphoreHandle_t       data_mutex_handle;
    TaskHandle_t            uart_task_handle;
    uint8_t                 uart_rx_len;
    uint8_t                 uart_rx_buffer[10];
    uint8_t                 uart_tx_len;
    uint8_t                 uart_tx_buffer[8];
} lin_master_context_t;



/**
 * static constant declarations
 */

static const char* TAG = "lin_master";


static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t data;

    static int current_pid = -1;
    static size_t rxd_len = 0;

    lin_master_context_t* context = (lin_master_context_t*)pvParameters;
    if (context == NULL) {
        ESP_LOGE(TAG, "LIN master context is NULL");
        //assert(context);
        return;
    }

    for (;;) {
        // Waiting for UART event.
        if (xQueueReceive(context->uart_queue_handle, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            // Event of UART receiving data.
            // This will contain the byte immediately following the 13 bit break since we cleared the RX buffer.
            case UART_DATA:;
                uart_read_bytes(context->config.uart_port, &data, 1, 0);

                if (rxd_len == 0) {
                    // This is the PID. It determines what action we can take.
                    switch (data) {
                    case 0x80:
                        // Ref1 position PID -> motor will answer
                        current_pid = 0x80;
                        break;
                    case 0x64:;
                        // Request power -> always respond
                        // Uncomment the below if using the original circuit from stevew817.
                        // static const uint8_t frame[2] = {0x9A, 0x01};
                        // uart_tx_chars(EX_UART_NUM, (char *)&frame, 2);
                        break;
                    case 0xE7:
                        // Safety sequence -> respond in sequence for as long as there is an active command
                        current_pid = -1; // Discard - do not need to make thread aware of PRBS

                        if (xSemaphoreTake(context->data_mutex_handle, (TickType_t)10) == pdTRUE) {
                            if (context->uart_tx_len > 0) {
                                //uart_tx_chars(context->config.uart_port, (char *)&prbs_sequence[prbs_seq], 2);
                                //prbs_seq = (prbs_seq + 1) % 9;
                            }
                            xSemaphoreGive(context->data_mutex_handle);
                        }
                        break;
                    case 0xA8: // ID40
                        if (context->uart_tx_len > 0) {
                            //uart_tx_chars(context->config.uart_port, (char *)&prbs_sequence[prbs_seq], 2);
                            //prbs_seq = (prbs_seq + 1) % 9;
                        }
                        break;
                    case 0xCA:
                        // Ref1 input -> respond with command if there is one
                        if (xSemaphoreTake(context->data_mutex_handle, (TickType_t)10) == pdTRUE) {
                            if (context->uart_tx_len == 4 && context->uart_tx_buffer[0] == 0xCA) {
                                uint8_t frame[3] = {context->uart_tx_buffer[1], context->uart_tx_buffer[2], context->uart_tx_buffer[3]};
                                uart_tx_chars(context->config.uart_port, (char *)&frame, 3);
                            }
                            xSemaphoreGive(context->data_mutex_handle);
                        }
                        break;
                    default:
                        current_pid = -1; // Set PID to discard this message
                        break;
                    }
                }
                else if (rxd_len < sizeof(context->uart_rx_buffer)) {
                    if (current_pid >= 0) {
                        context->uart_rx_buffer[rxd_len] = data;
                    }
                }
                rxd_len += 1;
                break;

            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                //  If fifo overflow happened, you should consider adding flow control for your application.
                //  The ISR has already reset the rx FIFO,
                //  As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(context->config.uart_port);
                xQueueReset(context->uart_queue_handle);
                break;

            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                //  If buffer full happened, you should consider increasing your buffer size
                //  As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(context->config.uart_port);
                xQueueReset(context->uart_queue_handle);
                break;

            // Event of UART RX break detected.
            // Flush the RX buffer and reset the LIN message. First byte of the next DATA event should be the PID.
            case UART_BREAK:
                uart_wait_tx_done(context->config.uart_port, 100);
                uart_flush_input(context->config.uart_port);

                if (current_pid >= 0) {
                    context->uart_rx_buffer[0] = current_pid;
                    context->uart_rx_len = rxd_len;
                    //xSemaphoreGive(gDeskTaskSemaphore);
                }
                rxd_len = 0;
                current_pid = -1;
                break;

            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;

            // Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;

            // Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }

    vTaskDelete(NULL);
}



static inline esp_err_t lin_master_uart_init(lin_master_context_t *const context) {
    /* validate arguments */
    ESP_ARG_CHECK( context );

    // set serial port
    const uart_config_t uart_config = (uart_config_t) { 
        .baud_rate  = LIN_UART_BAUD_RATE,
        .data_bits  = LIN_UART_DATA_BITS,
        .parity     = LIN_UART_PARITY,
        .stop_bits  = LIN_UART_STOP_BITS,
        .flow_ctrl  = LIN_UART_FLOW_CTRL,
        .source_clk = UART_SCLK_DEFAULT
    };

    /* configure uart */
    ESP_RETURN_ON_ERROR( uart_driver_install(context->config.uart_port, LIN_UART_TX_BUF_SIZE * 2, 0, 10, &context->uart_queue_handle, 0), TAG, "unable to install uart drive, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_param_config(context->config.uart_port, &uart_config), TAG, "unable to configure uart parameters, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_set_pin(context->config.uart_port, context->config.uart_tx_io_num, context->config.uart_rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "unable to set uart pins, uart enable failed");

    // Set the RX full threshold as low as possible while still allowing breaks to be detected.
    // Also set the timeout for interrupts as low as possible.
    // This seems to lower the delay between receiving bits and being able to react on them, allowing us to reply to LIN packets in time.
    ESP_RETURN_ON_ERROR( uart_set_rx_full_threshold(context->config.uart_port, 1), TAG, "unable to configure rx threshold, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_set_rx_timeout(context->config.uart_port, 1), TAG, "unable to configure rx timeout, uart enable failed");

    uart_set_always_rx_timeout(context->config.uart_port, true);

    ESP_RETURN_ON_ERROR( uart_enable_rx_intr(context->config.uart_port), TAG, "unable to enable rx interrupt, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_disable_tx_intr(context->config.uart_port), TAG, "unable to disable tx interrupt, uart enable failed");

    return ESP_OK;
}

esp_err_t lin_master_init(const lin_master_config_t *lin_master_config, lin_master_handle_t *const lin_master_handle) {
    esp_err_t ret = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( lin_master_config );

    /* validate memory availability for handle */
    lin_master_context_t* context = (lin_master_context_t*)calloc(1, sizeof(lin_master_context_t));
    ESP_GOTO_ON_FALSE(context, ESP_ERR_NO_MEM, err, TAG, "no memory for LIN context handle, init failed");

    /* copy configuration */
    context->config = *lin_master_config;

    /* create mutex */
    context->data_mutex_handle = xSemaphoreCreateMutex();
    ESP_GOTO_ON_FALSE(context->data_mutex_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for LIN data mutex, init failed");

    /* attempt to initialize gpio pins and levels */
    //ESP_GOTO_ON_ERROR( mux4052a_gpio_init(context), err, TAG, "unable to init gpio, init failed");

    /* attempt to initialize uart */
    ESP_GOTO_ON_ERROR( lin_master_uart_init(context), err, TAG, "unable to init uart, init failed");

    /* set output parameter */
    *lin_master_handle = (lin_master_handle_t)context;

    // Create UART event task
    xTaskCreate(uart_event_task, "uart_event_task", 2048, &context, 3, NULL);

    return ESP_OK;
    err:
        return ret;
}