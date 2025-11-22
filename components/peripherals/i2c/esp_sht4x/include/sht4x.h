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
 * @file sht4x.h
 * @defgroup drivers sht4x
 * @{
 *
 * ESP-IDF driver for sht4x sensor
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SHT4X_H__
#define __SHT4X_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "sht4x_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_SHT4X_DEV_CLK_SPD           UINT32_C(100000)    //!< sht4x i2c default scl clock frequency (100KHz)

#define I2C_SHT4X_DEV_ADDR_LO           UINT8_C(0x44)       //!< sht4x i2c address when ADDR pin floating/low
#define I2C_SHT4X_DEV_ADDR_HI           UINT8_C(0x45)       //!< sht4x i2c address when ADDR pin high

/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `i2c_sht4x_config_t` to default configuration settings.
 */
#define SHT4X_CONFIG_DEFAULT {                      \
        .i2c_address    = I2C_SHT4X_DEV_ADDR_LO,        \
        .i2c_clock_speed= I2C_SHT4X_DEV_CLK_SPD,        \
        .repeat_mode    = SHT4X_REPEAT_HIGH,            \
        .heater_mode    = SHT4X_HEATER_OFF }

/**
 * public enumerator, union, and structure definitions
 */

/* sht4x i2c measurement response packet prototyping */
typedef union {
    uint8_t  bytes[6];
    struct temperature_tag {
        union temperature_data_tag {
            uint8_t  bytes[3];
            uint16_t value;
            uint8_t  crc;
        } temperature_data;
    } temperature;
    struct humidity_tag {
        union humidity_data_tag {
            uint8_t  bytes[3];
            uint16_t value;
            uint8_t  crc;
        } humidity_data;
    } humidity;
} i2c_sht4x_data___t; 

/** 
 * @brief SHT4X measurement heater modes enumerator definition.
*/
typedef enum sht4x_heater_modes_e {
    SHT4X_HEATER_OFF = 0,      /*!< heater is off, default */
    SHT4X_HEATER_HIGH_LONG,    /*!< high power (~200mW), 1 second pulse */
    SHT4X_HEATER_HIGH_SHORT,   /*!< high power (~200mW), 0.1 second pulse */
    SHT4X_HEATER_MEDIUM_LONG,  /*!< medium power (~110mW), 1 second pulse */
    SHT4X_HEATER_MEDIUM_SHORT, /*!< medium power (~110mW), 0.1 second pulse */
    SHT4X_HEATER_LOW_LONG,     /*!< low power (~20mW), 1 second pulse */
    SHT4X_HEATER_LOW_SHORT,    /*!< low power (~20mW), 0.1 second pulse */
} sht4x_heater_modes_t;

/**
  * @brief SHT4X measurement repeatability modes enumerator definition.
 */
typedef enum sht4x_repeat_modes_e {
    SHT4X_REPEAT_HIGH = 0,     /*!< high repeatability (high resolution) */
    SHT4X_REPEAT_MEDIUM,       /*!< medium repeatability (medium resolution) */
    SHT4X_REPEAT_LOW           /*!< low repeatability (low resolution) */
} sht4x_repeat_modes_t;

/**
 * @brief SHT4X configuration structure definition.
 */
typedef struct sht4x_config_s {
    uint16_t             i2c_address;       /*!< sht4x i2c device address */
    uint32_t             i2c_clock_speed;   /*!< sht4x i2c device scl clock speed  */
    sht4x_repeat_modes_t repeat_mode;       /*!< sht4x measurement repeatability mode setting */
    sht4x_heater_modes_t heater_mode;       /*!< sht4x measurement heater mode setting */
} sht4x_config_t;

/**
 * @brief SHT4X data record structure definition.
 */
typedef struct sht4x_data_record_s {
    float drybulb;        /*!< dry-bulb temperature in degree Celsius */
    float humidity;       /*!< relative humidity in percentage */
    float dewpoint;       /*!< calculated dew-point temperature in degree Celsius */
    float wetbulb;        /*!< calculated wet-bulb temperature in degree Celsius */
} sht4x_data_record_t;

/**
 * @brief SHT4X opaque handle structure definition.
 */
typedef void* sht4x_handle_t;




/**
 * @brief Initializes an SHT4X device onto the HAL bus.
 *
 * @param[in] hal_handle HAL bus handle.
 * @param[in] sht4x_config SHT4X device configuration.
 * @param[out] sht4x_handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_init(const void* hal_handle, const sht4x_config_t *sht4x_config, sht4x_handle_t *const sht4x_handle);

/**
 * @brief Reads high-level measurements from SHT4X.  This is a blocking function.
 *
 * @note The function delays the calling task up to 1.1 s to wait for
 *       the measurement results. This might lead to problems when the function
 *       is called from a software timer callback function.
 *
 * @param[in] handle SHT4X device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] humidity Relative humidity in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_get_measurement(sht4x_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Similar to `sht4x_get_measurement` but it includes derived dew-point and wet-bulb temperatures in the results.
 *
 * @param[in] handle SHT4X device handle.
 * @param[out] drybulb Dry-bulb temperature in degree Celsius.
 * @param[out] humidity Relative humidity in percentage.
 * @param[out] dewpoint Calculated dew-point temperature in degree Celsius.
 * @param[out] wetbulb Calculated wet-bulb temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_get_measurements(sht4x_handle_t handle, float *const drybulb, float *const humidity, float *const dewpoint, float *const wetbulb);

/**
 * @brief Reads measurements from SHT4X and transforms them into a `sht4x_data_record_t` structure.  This is a blocking function.
 * 
 * @param handle SHT4X device handle.
 * @param data_record Data record structure that contains dry-bulb, humidity, dew-point, and wet-bulb measurements.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_get_measurement_record(sht4x_handle_t handle, sht4x_data_record_t *const data_record);

/**
 * @brief Reads measurement repeatability mode setting from SHT4X.
 * 
 * @param handle SHT4X device handle.
 * @param mode Repeatability mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_get_repeat_mode(sht4x_handle_t handle, sht4x_repeat_modes_t *const mode);

/**
 * @brief Writes measurement repeatability mode setting to SHT4X.
 * 
 * @param handle SHT4X device handle.
 * @param mode Repeatability mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_set_repeat_mode(sht4x_handle_t handle, const sht4x_repeat_modes_t mode);

/**
 * @brief Reads measurement heater mode setting from SHT4X.
 * 
 * @param handle SHT4X device handle.
 * @param mode Heater mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_get_heater_mode(sht4x_handle_t handle, sht4x_heater_modes_t *const mode);

/**
 * @brief Writes measurement heater mode setting to SHT4X.
 * 
 * @param handle SHT4X device handle.
 * @param mode Heater mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_set_heater_mode(sht4x_handle_t handle, const sht4x_heater_modes_t mode);

/**
 * @brief Issues soft-reset to SHT4X.
 *
 * @param[in] handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_reset(sht4x_handle_t handle);

/**
 * @brief Removes an SHT4X device from master I2C bus.
 *
 * @param[in] handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_remove(sht4x_handle_t handle);

/**
 * @brief Removes an SHT4X device from master I2C bus and delete the handle.
 * 
 * @param handle SHT4X device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sht4x_delete(sht4x_handle_t handle);

/**
 * @brief Converts SHT4X firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* SHT4X firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* sht4x_get_fw_version(void);

/**
 * @brief Converts SHT4X firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t SHT4X firmware version number.
 */
int32_t sht4x_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SHT4X_H__
