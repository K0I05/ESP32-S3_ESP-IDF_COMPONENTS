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
 * @file ahtxx.h
 * @defgroup drivers ahtxx
 * @{
 *
 * ESP-IDF driver for ahtxx sensor types
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AHTXX_H__
#define __AHTXX_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <hal_master.h>
#include <type_utils.h>
#include "ahtxx_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_AHTXX_DEV_CLK_SPD   UINT32_C(100000) /*!< ahtxx i2c device scl clock frequency (100KHz) */
#define I2C_AHTXX_DEV_ADDR      UINT8_C(0x38)    /*!< ahtxx i2c device address */


/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings that implements the I2C bus interface for the aht30 sensor type.
 */
#define AHT30_CONFIG_DEFAULT {                  \
    .hal_bif        = HAL_MASTER_BIF_I2C, \
    .hal_config     = (void*)&(i2c_device_config_t){ \
        .device_address = I2C_AHTXX_DEV_ADDR,       \
        .scl_speed_hz   = I2C_AHTXX_DEV_CLK_SPD     \
    },                                              \
    .sensor_type     = AHTXX_AHT30 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings that implements the I2C bus interface for the aht25 sensor type.
 */
#define AHT25_CONFIG_DEFAULT {                  \
    .hal_bif        = HAL_MASTER_BIF_I2C, \
    .hal_config     = (void*)&(i2c_device_config_t){ \
        .device_address = I2C_AHTXX_DEV_ADDR,       \
        .scl_speed_hz   = I2C_AHTXX_DEV_CLK_SPD     \
    },                                              \
    .sensor_type     = AHTXX_AHT25 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings that implements the I2C bus interface for the aht21 sensor type.
 */
#define AHT21_CONFIG_DEFAULT {                  \
    .hal_bif        = HAL_MASTER_BIF_I2C, \
    .hal_config     = (void*)&(i2c_device_config_t){ \
        .device_address = I2C_AHTXX_DEV_ADDR,       \
        .scl_speed_hz   = I2C_AHTXX_DEV_CLK_SPD     \
    },                                              \
    .sensor_type     = AHTXX_AHT21 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings that implements the I2C bus interface for the aht20 sensor type.
 */
#define AHT20_CONFIG_DEFAULT {                  \
    .hal_bif        = HAL_MASTER_BIF_I2C, \
    .hal_config     = (void*)&(i2c_device_config_t){ \
        .device_address = I2C_AHTXX_DEV_ADDR,       \
        .scl_speed_hz   = I2C_AHTXX_DEV_CLK_SPD     \
    },                                              \
    .sensor_type     = AHTXX_AHT20 }

/**
 * @brief Macro that initializes `ahtxx_config_t` to default configuration settings that implements the I2C bus interface for the aht10 sensor type.
 */
#define AHT10_CONFIG_DEFAULT {                  \
    .hal_bif        = HAL_MASTER_BIF_I2C, \
    .hal_config     = (void*)&(i2c_device_config_t){ \
        .device_address = I2C_AHTXX_DEV_ADDR,       \
        .scl_speed_hz   = I2C_AHTXX_DEV_CLK_SPD     \
    },                                              \
    .sensor_type     = AHTXX_AHT10 }



/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief AHTXX sensor types enumerator definition.
 * 
 * @note AHTXX types vary slightly with respect to setup and initialization according to available documentation.
 * The AHT10 and AHT20 are setup through the initialization command.  The AHT21, AHT25 and AHT30 are setup by resetting
 * 0x1b, 0x1c, and 0x1e initializing registers.
 */
typedef enum ahtxx_sensor_types_e {
    AHTXX_AHT10,    /*!< */
    AHTXX_AHT20,
    AHTXX_AHT21,
    AHTXX_AHT25,
    AHTXX_AHT30
} ahtxx_sensor_types_t;

/**
 * @brief AHTXX configuration structure definition.
 */
typedef struct ahtxx_config_s {
    hal_master_interfaces_t hal_bif;     /*!< HAL master bus interface type */
    void                   *hal_config;  /*!< HAL master bus interface configuration, the type (i2c_device_config_t) depends on the bus interface */
    ahtxx_sensor_types_t    sensor_type; /*!< aht sensor type, see `ahtxx_sensor_types_t` enumerator for support sensor types */
} ahtxx_config_t;

/**
 * @brief AHTXX opaque handle structure definition.
 */
typedef void* ahtxx_handle_t;


/**
 * public function and subroutine declarations
 */

/**
 * @brief Initializes an AHTXX device onto the I2C master bus.
 *
 * @param[in] master_handle HAL master bus handle.  The AHTXX device supports I2C bus interface only.
 * The bus handle must be of type `i2c_master_bus_handle_t` and initialized.
 * @param[in] ahtxx_config AHTXX device configuration.
 * @param[out] ahtxx_handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_init(const void* master_handle, const ahtxx_config_t *ahtxx_config, ahtxx_handle_t *const ahtxx_handle);

/**
 * @brief Reads temperature and relative humidity from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param temperature Temperature in degree Celsius.
 * @param humidity Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_measurement(ahtxx_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Similar to the `ahtxx_get_measurement` function but it includes dew-point and wet-bulb temperatures in the results.
 *
 * @param[in] handle AHTXX device handle.
 * @param[out] drybulb Dry-bulb temperature in degree Celsius.
 * @param[out] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dew-point temperature in degree Celsius.
 * @param[out] wetbulb Calculated wet-bulb temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_measurements(ahtxx_handle_t handle, float *const drybulb, float *const humidity, float *const dewpoint, float *const wetbulb);

/**
 * @brief Reads busy status flag from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_busy_status(ahtxx_handle_t handle, bool *const busy);

/**
 * @brief Reads calibration status flag from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_calibration_status(ahtxx_handle_t handle, bool *const calibrated);

/**
 * @brief Reads busy and calibrated status flags from AHTXX.
 *
 * @param handle AHTXX device handle.
 * @param[out] busy AHTXX is busy when true.
 * @param[out] calibrated AHTXX is calibrated when true.  See `i2c_ahtxx_setup` and datasheet for details.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_get_status(ahtxx_handle_t handle, bool *const busy, bool *const calibrated);

/**
 * @brief Issues soft-reset and initializes AHTXX.  See datasheet for details.
 *
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_reset(ahtxx_handle_t handle);

/**
 * @brief Removes an AHTXX device from master bus.
 *
 * @param[in] handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_remove(ahtxx_handle_t handle);

/**
 * @brief Removes an AHTXX device from master bus and frees handle.
 * 
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ahtxx_delete(ahtxx_handle_t handle);

/**
 * @brief Converts AHTXX firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* AHTXX firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ahtxx_get_fw_version(void);

/**
 * @brief Converts AHTXX firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t AHTXX firmware version number.
 */
int32_t ahtxx_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __AHTXX_H__
