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
 * @file bmp280.h
 * @defgroup drivers bmp280
 * @{
 *
 * ESP-IDF driver for bmp280 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP280_H__
#define __BMP280_H__

/**
 * dependency includes
 */

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "bmp280_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * public constant definitions
 */

#define I2C_BMP280_DEV_CLK_SPD      UINT32_C(100000) //!< bmp280 I2C default clock frequency (100KHz)

#define I2C_BMP280_DEV_ADDR_LO      UINT8_C(0x76) //!< bmp280 I2C address when ADDR pin floating/low
#define I2C_BMP280_DEV_ADDR_HI      UINT8_C(0x77) //!< bmp280 I2C address when ADDR pin high


/**
 * public macro definitions
 */

/**
 * @brief Macro that initializes `bmp280_config_t` to default configuration settings.
 */
#define BMP280_CONFIG_DEFAULT {                                          \
        .i2c_address                = I2C_BMP280_DEV_ADDR_HI,                \
        .i2c_clock_speed            = I2C_BMP280_DEV_CLK_SPD,                \
        .power_mode                 = BMP280_POWER_MODE_NORMAL,              \
        .iir_filter                 = BMP280_IIR_FILTER_OFF,                 \
        .pressure_oversampling      = BMP280_PRESSURE_OVERSAMPLING_4X,       \
        .temperature_oversampling   = BMP280_TEMPERATURE_OVERSAMPLING_4X,    \
        .standby_time               = BMP280_STANDBY_TIME_250MS }


/**
 * public enumerator, union, and structure definitions
 */

/**
 * @brief BMP280 IIR filters coefficient enumerator definition.
 */
typedef enum bmp280_iir_filters_e {
    BMP280_IIR_FILTER_OFF = (0b000),
    BMP280_IIR_FILTER_2   = (0b001),
    BMP280_IIR_FILTER_4   = (0b010),
    BMP280_IIR_FILTER_8   = (0b011),
    BMP280_IIR_FILTER_16  = (0b100)
} bmp280_iir_filters_t;

/**
 * @brief BMP280 stand-by times enumerator definition.
 */
typedef enum bmp280_standby_times_e {
    BMP280_STANDBY_TIME_0_5MS  = (0b000),  //!< stand by time 0.5ms
    BMP280_STANDBY_TIME_62_5MS = (0b001),  //!< stand by time 62.5ms
    BMP280_STANDBY_TIME_125MS  = (0b010),  //!< stand by time 125ms
    BMP280_STANDBY_TIME_250MS  = (0b011),  //!< stand by time 250ms
    BMP280_STANDBY_TIME_500MS  = (0b100),  //!< stand by time 500ms
    BMP280_STANDBY_TIME_1000MS = (0b101),  //!< stand by time 1s
    BMP280_STANDBY_TIME_2000MS = (0b110),  //!< stand by time 2s BMP280, 10ms BME280
    BMP280_STANDBY_TIME_4000MS = (0b111)   //!< stand by time 4s BMP280, 20ms BME280
} bmp280_standby_times_t;

/**
 * @brief BMP280 power modes enumerator definition.
 */
typedef enum bmp280_power_modes_e {
    BMP280_POWER_MODE_SLEEP   = (0b00), //!< sleep mode, default after power-up
    BMP280_POWER_MODE_FORCED  = (0b01), //!< measurement is initiated by user. The value 0b10 is also valid for forced mode.
    BMP280_POWER_MODE_NORMAL  = (0b11)  //!< continuously cycles between active measurement and inactive (standby-time) periods
} bmp280_power_modes_t;

/**
 * @brief BMP280 pressure oversampling enumerator definition.
 */
typedef enum bmp280_pressure_oversampling_e {
    BMP280_PRESSURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BMP280_PRESSURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    BMP280_PRESSURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    BMP280_PRESSURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    BMP280_PRESSURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    BMP280_PRESSURE_OVERSAMPLING_16X        = (0b101)   //!< ultra high resolution
} bmp280_pressure_oversampling_t;

/**
 * @brief BMP280 temperature oversampling enumerator definition.
 */
typedef enum bmp280_temperature_oversampling_e {
    BMP280_TEMPERATURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BMP280_TEMPERATURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    BMP280_TEMPERATURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    BMP280_TEMPERATURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    BMP280_TEMPERATURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    BMP280_TEMPERATURE_OVERSAMPLING_16X        = (0b101)   //!< ultra high resolution. The values 0b110 and 0b111 are also valid for x16 oversampling.
} bmp280_temperature_oversampling_t;

/**
 * @brief BMP280 configuration structure definition.
 */
typedef struct bmp280_config_s {
    uint16_t                                i2c_address;                /*!< bmp280 i2c device address */
    uint32_t                                i2c_clock_speed;            /*!< bmp280 i2c device scl clock speed  */
    bmp280_power_modes_t                    power_mode;                 /*!< bmp280 power mode setting */
    bmp280_iir_filters_t                    iir_filter;                 /*!< bmp280 IIR filter setting */
    bmp280_pressure_oversampling_t          pressure_oversampling;      /*!< bmp280 pressure oversampling setting */
    bmp280_temperature_oversampling_t       temperature_oversampling;   /*!< bmp280 temperature oversampling setting */
    bmp280_standby_times_t                  standby_time;               /*!< bmp280 stand-by time setting */
} bmp280_config_t;


/**
 * @brief BMP280 opaque handle structure definition.
 */
typedef void* bmp280_handle_t;

/**
 * public function and subroutine declarations
 */

/**
 * @brief Initializes an BMP280 device onto the master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] bmp280_config BMP280 device configuration.
 * @param[out] bmp280_handle BMP280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_init(i2c_master_bus_handle_t master_handle, const bmp280_config_t *bmp280_config, bmp280_handle_t *bmp280_handle);

/**
 * @brief Reads temperature and pressure measurements from BMP280.
 *
 * @param[in] handle BMP280 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] pressure Pressure in pascal.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_measurements(bmp280_handle_t handle, float *const temperature, float *const pressure);

/**
 * @brief Reads data status from BMP280.
 * 
 * @param[in] handle bmp280 device handle.
 * @param[out] ready data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_data_status(bmp280_handle_t handle, bool *const ready);

/**
 * @brief Reads power mode setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] power_mode Power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_power_mode(bmp280_handle_t handle, bmp280_power_modes_t *const power_mode);

/**
 * @brief Writes power mode setting to the BMP280.  See datasheet, section 3.6, table 10.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] power_mode Power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_set_power_mode(bmp280_handle_t handle, const bmp280_power_modes_t power_mode);

/**
 * @brief Reads pressure oversampling setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] oversampling Pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_pressure_oversampling(bmp280_handle_t handle, bmp280_pressure_oversampling_t *const oversampling);

/**
 * @brief Writes pressure oversampling setting to BMP280.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] oversampling Pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_set_pressure_oversampling(bmp280_handle_t handle, const bmp280_pressure_oversampling_t oversampling);

/**
 * @brief Reads temperature oversampling setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] oversampling Temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_temperature_oversampling(bmp280_handle_t handle, bmp280_temperature_oversampling_t *const oversampling);

/**
 * @brief Writes temperature oversampling setting to BMP280.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] oversampling Temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_set_temperature_oversampling(bmp280_handle_t handle, const bmp280_temperature_oversampling_t oversampling);

/**
 * @brief Reads stand-by time setting from BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] standby_time Stand-by time setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_standby_time(bmp280_handle_t handle, bmp280_standby_times_t *const standby_time);

/**
 * @brief Writes stand-by time setting to BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] standby_time Stand-by time setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_set_standby_time(bmp280_handle_t handle, const bmp280_standby_times_t standby_time);

/**
 * @brief Reads IIR filter setting to BMP280.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[out] iir_filter IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_get_iir_filter(bmp280_handle_t handle, bmp280_iir_filters_t *const iir_filter);

/**
 * @brief Writes IIR filter setting to BMP280.  See datasheet, section 3.4, table 7.
 * 
 * @param[in] handle BMP280 device handle.
 * @param[in] iir_filter IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_set_iir_filter(bmp280_handle_t handle, const bmp280_iir_filters_t iir_filter);

/**
 * @brief Issues soft-reset sensor and initializes BMP280.
 *
 * @param[in] handle BMP280 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_reset(bmp280_handle_t handle);

/**
 * @brief Removes an BMP280 device from master bus.
 *
 * @param[in] handle BMP280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_remove(bmp280_handle_t handle);

/**
 * @brief Removes an BMP280 device from master bus and frees handle.
 *
 * @param[in] handle BMP280 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp280_delete(bmp280_handle_t handle);

/**
 * @brief Converts BMP280 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* BMP280 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* bmp280_get_fw_version(void);

/**
 * @brief Converts BMP280 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t BMP280 firmware version number.
 */
int32_t bmp280_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP280_H__
