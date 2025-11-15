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
 * @file app_config.h
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <onewire_bus.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TSK_MINIMAL_STACK_SIZE         (1024)

#define I2C0_MASTER_PORT               I2C_NUM_0
#define I2C0_MASTER_SDA_IO             GPIO_NUM_45 // blue
#define I2C0_MASTER_SCL_IO             GPIO_NUM_48 // yellow
//
#define I2C0_TASK_SAMPLING_RATE        (10) // seconds
#define I2C0_TASK_STACK_SIZE           (TSK_MINIMAL_STACK_SIZE * 10)
#define I2C0_TASK_PRIORITY             (tskIDLE_PRIORITY + 2)

#define OWB0_MASTER_DEVICE_MAX         (8)
#define OWB0_MASTER_DQ_IO              GPIO_NUM_47

#define OWB0_TASK_SAMPLING_RATE        (10) // seconds
#define OWB0_TASK_STACK_SIZE           (TSK_MINIMAL_STACK_SIZE * 8)
#define OWB0_TASK_PRIORITY             (tskIDLE_PRIORITY + 2)

#define SPI1_MASTER_HOST               SPI1_HOST
#define SPI1_MASTER_SCLK_IO            GPIO_NUM_35 //
#define SPI1_MASTER_MISO_IO            GPIO_NUM_36 //
#define SPI1_MASTER_MOSI_IO            GPIO_NUM_37 //

#define SPI1_TASK_SAMPLING_RATE        (10) // seconds
#define SPI1_TASK_STACK_SIZE           (TSK_MINIMAL_STACK_SIZE * 8)
#define SPI1_TASK_PRIORITY             (tskIDLE_PRIORITY + 2)

#define UTILS_TASK_SAMPLING_RATE       (30) // seconds
#define UTILS_TASK_STACK_SIZE          (TSK_MINIMAL_STACK_SIZE * 8)
#define UTILS_TASK_PRIORITY            (tskIDLE_PRIORITY + 2)

#define SCH_TASK_SAMPLING_RATE         (30) // seconds
#define SCH_TASK_STACK_SIZE            (TSK_MINIMAL_STACK_SIZE * 8)
#define SCH_TASK_PRIORITY              (tskIDLE_PRIORITY + 2)

#define APP_TAG                         "ESP-IDF COMPONENTS [APP]"

// macros

#define I2C0_MASTER_CONFIG_DEFAULT {                                \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = I2C0_MASTER_PORT,         \
        .scl_io_num                     = I2C0_MASTER_SCL_IO,       \
        .sda_io_num                     = I2C0_MASTER_SDA_IO,       \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

#define OW0_RMT_CONFIG_DEFAULT { .max_rx_bytes = 10 } // 1-byte ROM command + 8-byte ROM number + 1-byte device command

#define OW0_MASTER_CONFIG_DEFAULT { .bus_gpio_num = OWB0_MASTER_DQ_IO }

#define SPI1_MASTER_CONFIG_DEFAULT {                            \
        .sclk_io_num     = SPI1_MASTER_SCLK_IO,                 \
        .miso_io_num     = SPI1_MASTER_MISO_IO,                 \
        .mosi_io_num     = SPI1_MASTER_MOSI_IO,                 \
        .quadwp_io_num   = -1,                                  \
        .quadhd_io_num   = -1,                                  \
        .max_transfer_sz = 32}

extern i2c_master_bus_config_t  i2c0_bus_cfg;
extern i2c_master_bus_handle_t  i2c0_bus_hdl;

extern onewire_bus_rmt_config_t owb0_rmt_cfg;
extern onewire_bus_config_t     owb0_bus_cfg;
extern onewire_bus_handle_t     owb0_bus_hdl;

extern spi_bus_config_t         spi1_bus_cfg;
extern spi_device_handle_t      spi1_dev_hdl;

static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __APP_CONFIG_H__
