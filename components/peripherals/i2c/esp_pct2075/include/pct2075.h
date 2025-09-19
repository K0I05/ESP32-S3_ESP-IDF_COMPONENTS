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
 * @file pct2075.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for pct2075 sensor types
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __PCT2075_H__
#define __PCT2075_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "pct2075_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_PCT2075_DEV_CLK_SPD             UINT32_C(100000)    /*!< pct2075 i2c device scl clock frequency (100KHz) */
#define I2C_PCT2075_DEV_ADDR                UINT8_C(0x37)       /*!< pct2075 i2c device address */

/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `pct2075_config_t` to default configuration settings.
 */
#define I2C_PCT2075_CONFIG_DEFAULT {                  \
    .i2c_address     = I2C_PCT2075_DEV_ADDR,          \
    .i2c_clock_speed = I2C_PCT2075_DEV_CLK_SPD }



/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief PCT2075 OS output polarities enumerator.
 */
typedef enum pct2075_os_polarities_e {
    PCT2075_OS_POL_ACTIVE_LOW  = 0, /*!< sets the active state to OS output to low (default) */
    PCT2075_OS_POL_ACTIVE_HIGH = 1  /*!< sets the active state to OS output to high */
} pct2075_os_polarities_t;

/**
 * @brief PCT2075 OS operation modes enumerator.
 */
typedef enum pct2075_os_operation_modes_e {
    PCT2075_OS_OP_MODE_COMPARATOR = 0,  /*< selects the OS comparator mode (default) */
    PCT2075_OS_OP_MODE_INTERRUPT  = 1   /*< selects the OS interrupt mode */
} pct2075_os_operation_modes_t;

/**
 * @brief PCT2075 OS fault queue programming enumerator.
 */
typedef enum pct2075_os_fault_queues_e {
    PCT2075_OS_FAULT_QUEUE_1 = (0b00),  /*< fault queue data 00 with a fault queue value of 1 (default) */
    PCT2075_OS_FAULT_QUEUE_2 = (0b10),  /*< fault queue data 10 with a fault queue value of 2 */
    PCT2075_OS_FAULT_QUEUE_4 = (0b01),  /*< fault queue data 01 with a fault queue value of 4 */
    PCT2075_OS_FAULT_QUEUE_6 = (0b11)   /*< fault queue data 11 with a fault queue value of 6 */
} pct2075_os_fault_queues_t;


/**
 * @brief PCT2075 configuration structure definition.
 */
typedef struct pct2075_config_s {
    uint16_t                        i2c_address;        /*!< pct2075 i2c device address */
    uint32_t                        i2c_clock_speed;    /*!< pct2075 i2c device scl clock speed in hz */
    bool                            shutdown_enabled;   /*!< pct2075 is shutdown when enabled (true), shutdown is disabled by default */
    pct2075_os_operation_modes_t    operation_mode;     /*!< pct2075 os operation mode, the OS comparator mode is selected by default */
    pct2075_os_polarities_t         polarity;           /*!< pct2075 os polarity, the active state of OS output polarity is low by default */
    pct2075_os_fault_queues_t       fault_queue;        /*!< pct2075 os fault queue programming, fault queue data 00 with a fault queue value of 1 are set by default */
    bool                            configure_setpoints;/*!< pct2075 configures set-point temperatures (Tots & Thys) when true, factory set-point temperatures are set by default */
    float                           ots_temperature;    /*!< pct2075 overtemperature shutdown set-point temperature in degree Celsius (range is -55 to 125 degree Celsius), a temperature of 80.0 °C is set by default */
    float                           hys_temperature;    /*!< pct2075 hysteresis set-point temperature in degree Celsius (range is -55 to 125 degree Celsius), a temperature of 75.0 °C is set by default */
    bool                            configure_sampling; /*!< pct2075 sampling period is set when true, factory sampling period is used by default */
    uint16_t                        sampling_period;    /*!< pct2075 sampling period in milliseconds (range is 100 to 3,100 milliseconds), 100 ms is the sampling period by default */
} pct2075_config_t;


/**
 * @brief PCT2075 opaque handle structure definition.
 */
typedef void* pct2075_handle_t;

/**
 * public function and subroutine declarations
 */


/**
 * @brief Initializes an PCT2075 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] pct2075_config PCT2075 device configuration.
 * @param[out] pct2075_handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_init(const i2c_master_bus_handle_t master_handle, const pct2075_config_t *pct2075_config, pct2075_handle_t *const pct2075_handle);

/**
 * @brief Reads temperature measurement from PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_temperature(pct2075_handle_t handle, float *const temperature);

/**
 * @brief Reads overtemperature shutdown set-point temperature from PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_ots_temperature(pct2075_handle_t handle, float *const temperature);

/**
 * @brief Writes overtemperature shutdown set-point temperature to PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[in] temperature Temperature in degree Celsius (range is -55 to 125 degree Celsius).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_ots_temperature(pct2075_handle_t handle, const float temperature);

/**
 * @brief Reads hysteresis set-point temperature from PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_hys_temperature(pct2075_handle_t handle, float *const temperature);

/**
 * @brief Writes hysteresis set-point temperature to PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[in] temperature Temperature in degree Celsius (range is -55 to 125 degree Celsius).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_hys_temperature(pct2075_handle_t handle, const float temperature);

/**
 * @brief Reads sampling period from PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[out] period Sampling period in milliseconds.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_sampling_period(pct2075_handle_t handle, uint16_t *const period);

/**
 * @brief Writes sampling period to PCT2075.
 *
 * @param[in] handle PCT2075 device handle.
 * @param[in] period Sampling period in milliseconds (range is 100 to 3,100 milliseconds).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_sampling_period(pct2075_handle_t handle, const uint16_t period);

/**
 * @brief Reads operation mode from PCT2075.
 * 
 * @param handle PCT2075 device handle.
 * @param operation_mode Operation mode.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_operation_mode(pct2075_handle_t handle, pct2075_os_operation_modes_t *const operation_mode);

/**
 * @brief Writes operation mode to PCT2075.
 * 
 * @param handle PCT2075 device handle.
 * @param operation_mode Operation mode.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_operation_mode(pct2075_handle_t handle, const pct2075_os_operation_modes_t operation_mode);

/**
 * @brief Reads output polarity from PCT2075.
 * 
 * @param handle PCT2075 device handle.
 * @param polarity Output polarity.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_polarity(pct2075_handle_t handle, pct2075_os_polarities_t *const polarity);

/**
 * @brief Writes output polarity to PCT2075.
 * 
 * @param handle PCT2075 device handle.
 * @param polarity Output polarity.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_polarity(pct2075_handle_t handle, const pct2075_os_polarities_t polarity);

/**
 * @brief Reads fault queue from PCT2075.
 * 
 * @param handle PCT2075 device handle.
 * @param fault_queue Fault queue.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_get_fault_queue(pct2075_handle_t handle, pct2075_os_fault_queues_t *const fault_queue);

/**
 * @brief Writes fault queue to PCT2075.
 * @param handle PCT2075 device handle.
 * @param fault_queue Fault queue.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_set_fault_queue(pct2075_handle_t handle, const pct2075_os_fault_queues_t fault_queue);

/**
 * @brief Disables PCT2075 device (i.e. shutdown).
 *
 * @param[in] handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_disable(pct2075_handle_t handle);

/**
 * @brief Enables PCT2075 device (i.e. wakeup).
 *
 * @param[in] handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_enable(pct2075_handle_t handle);

/**
 * @brief Removes an PCT2075 device from master bus.
 *
 * @param[in] handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_remove(pct2075_handle_t handle);

/**
 * @brief Removes an PCT2075 device from master bus and frees handle.
 * 
 * @param handle PCT2075 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t pct2075_delete(pct2075_handle_t handle);

/**
 * @brief Converts PCT2075 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* PCT2075 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* pct2075_get_fw_version(void);

/**
 * @brief Converts PCT2075 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t PCT2075 firmware version number.
 */
int32_t pct2075_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __PCT2075_H__
