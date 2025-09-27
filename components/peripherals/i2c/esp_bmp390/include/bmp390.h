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
 * @file bmp390.h
 * @defgroup drivers bmp390
 * @{
 *
 * ESP-IDF driver for bmp390 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BMP390_H__
#define __BMP390_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "bmp390_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BMP390 definitions
*/
#define I2C_BMP390_DEV_CLK_SPD      UINT32_C(100000) //!< bmp390 I2C default clock frequency (100KHz)

/*
 * supported device addresses
*/
#define I2C_BMP390_DEV_ADDR_LO      UINT8_C(0x76) //!< bmp390 I2C address when ADDR pin low
#define I2C_BMP390_DEV_ADDR_HI      UINT8_C(0x77) //!< bmp390 I2C address when ADDR pin high

/*
 * BMP390 macros
*/
#define BMP390_CONFIG_DEFAULT {                                          \
        .i2c_address                = I2C_BMP390_DEV_ADDR_HI,                \
        .i2c_clock_speed            = I2C_BMP390_DEV_CLK_SPD,                \
        .power_mode                 = BMP390_POWER_MODE_FORCED,              \
        .iir_filter                 = BMP390_IIR_FILTER_OFF,                 \
        .pressure_oversampling      = BMP390_PRESSURE_OVERSAMPLING_8X,       \
        .temperature_oversampling   = BMP390_TEMPERATURE_OVERSAMPLING_8X,    \
        .output_data_rate           = BMP390_ODR_40MS }

/*
 * BMP390 enumerator and structure declarations
*/

/**
 * @brief BMP390 IIR filters coefficient enumerator.
 */
typedef enum bmp390_iir_filters_e {
    BMP390_IIR_FILTER_OFF   = (0b000),
    BMP390_IIR_FILTER_1     = (0b001),
    BMP390_IIR_FILTER_3     = (0b010),
    BMP390_IIR_FILTER_7     = (0b011),
    BMP390_IIR_FILTER_15    = (0b100),
	BMP390_IIR_FILTER_31    = (0b101),
	BMP390_IIR_FILTER_63    = (0b110),
	BMP390_IIR_FILTER_127   = (0b111)
} bmp390_iir_filters_t;

/**
 * @brief BMP390 output data rates enumerator.
 */
typedef enum bmp390_output_data_rates_e {
    BMP390_ODR_5MS          = (0x00),  //!< sampling period 5ms
    BMP390_ODR_10MS         = (0x01),  //!< sampling period 10ms
    BMP390_ODR_20MS         = (0x02),  //!< sampling period 20ms
    BMP390_ODR_40MS         = (0x03),  //!< sampling period 40ms
    BMP390_ODR_80MS         = (0x04),  //!< sampling period 80ms
    BMP390_ODR_160MS        = (0x05),  //!< sampling period 160ms
    BMP390_ODR_320MS        = (0x06),  //!< sampling period 320ms
    BMP390_ODR_640MS        = (0x07)   //!< sampling period 640ms
    // TODO: ODR 1.280s to 655.36s
} bmp390_output_data_rates_t;

/**
 * @brief BMP390 power modes enumerator.
 */
typedef enum bmp390_power_modes_e {
    BMP390_POWER_MODE_SLEEP   = (0b00), //!< sleep mode, default after power-up
    BMP390_POWER_MODE_FORCED  = (0b01), //!< measurement is initiated by user
    BMP390_POWER_MODE_FORCED1 = (0b10), //!< measurement is initiated by user
    BMP390_POWER_MODE_NORMAL  = (0b11)  //!< continuously cycles between active measurement and inactive (standby-time) periods
} bmp390_power_modes_t;

/**
 * @brief BMP390 pressure oversampling enumerator.
 */
typedef enum bmp390_pressure_oversampling_e {
    BMP390_PRESSURE_OVERSAMPLING_SKIPPED        = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BMP390_PRESSURE_OVERSAMPLING_2X             = (0b001),  //!< ultra low power
    BMP390_PRESSURE_OVERSAMPLING_4X             = (0b010),  //!< low power
    BMP390_PRESSURE_OVERSAMPLING_8X             = (0b011),  //!< standard
    BMP390_PRESSURE_OVERSAMPLING_16X            = (0b100),  //!< high resolution
    BMP390_PRESSURE_OVERSAMPLING_32X            = (0b101)   //!< ultra high resolution
} bmp390_pressure_oversampling_t;

/**
 * @brief BMP390 temperature oversampling enumerator.
 */
typedef enum bmp390_temperature_oversampling_e {
    BMP390_TEMPERATURE_OVERSAMPLING_SKIPPED     = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BMP390_TEMPERATURE_OVERSAMPLING_2X          = (0b001),  //!< ultra low power
    BMP390_TEMPERATURE_OVERSAMPLING_4X          = (0b010),  //!< low power
    BMP390_TEMPERATURE_OVERSAMPLING_8X          = (0b011),  //!< standard
    BMP390_TEMPERATURE_OVERSAMPLING_16X         = (0b100),  //!< high resolution
    BMP390_TEMPERATURE_OVERSAMPLING_32X         = (0b101),  //!< ultra high resolution
} bmp390_temperature_oversampling_t;


/**
 * @brief BMP390 configuration structure.
 */
typedef struct bmp390_config_s {
    uint16_t                                i2c_address;                /*!< bmp390 i2c device address */
    uint32_t                                i2c_clock_speed;            /*!< bmp390 i2c device scl clock speed  */
    bmp390_iir_filters_t                    iir_filter;                 /*!< bmp390 IIR filter setting */
    bmp390_pressure_oversampling_t          pressure_oversampling;      /*!< bmp390 pressure oversampling setting */
    bmp390_temperature_oversampling_t       temperature_oversampling;   /*!< bmp390 temperature oversampling setting */
    bmp390_output_data_rates_t              output_data_rate;           /*!< bmp390 output data rate setting */
    bmp390_power_modes_t	                power_mode;                 /*!< bmp390 power mode setting */
} bmp390_config_t;

/**
 * @brief BMP390 opaque handle structure definition.
 */
typedef void* bmp390_handle_t;



/**
 * @brief Initializes an BMP390 device onto the master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] bmp390_config BMP390 device configuration.
 * @param[out] bmp390_handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_init(i2c_master_bus_handle_t master_handle, const bmp390_config_t *bmp390_config, bmp390_handle_t *bmp280_handle);

/**
 * @brief Reads high-level measurements (temperature & pressure) from BMP390.
 *
 * @param[in] handle BMP390 device handle.
 * @param[out] temperature Temperature in degree Celsius.
 * @param[out] pressure Pressure in pascal.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_measurements(bmp390_handle_t handle, float *const temperature, float *const pressure);

/**
 * @brief Reads status of the BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] temperature_ready Temperature data is ready when asserted to true.
 * @param[out] pressure_ready Pressure data is ready when asserted to true.
 * @param[out] command_ready Command is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_status(bmp390_handle_t handle, bool *const temperature_ready, bool *const pressure_ready, bool *const command_ready);

/**
 * @brief Reads data status of the BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] temperature_ready Temperature data is ready when asserted to true.
 * @param[out] pressure_ready Pressure data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_data_status(bmp390_handle_t handle, bool *const temperature_ready, bool *const pressure_ready);

/**
 * @brief Reads power mode setting from the BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] power_mode BMP390 power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_power_mode(bmp390_handle_t handle, bmp390_power_modes_t *const power_mode);

/**
 * @brief Writes power mode setting to the BMP390.  See datasheet, section 3.6, table 10.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[in] power_mode BMP390 power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_set_power_mode(bmp390_handle_t handle, const bmp390_power_modes_t power_mode);

/**
 * @brief Reads pressure oversampling setting from BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] oversampling BMP390 pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_pressure_oversampling(bmp390_handle_t handle, bmp390_pressure_oversampling_t *const oversampling);

/**
 * @brief Writes pressure oversampling setting to BMP390.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[in] oversampling BMP390 pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_set_pressure_oversampling(bmp390_handle_t handle, const bmp390_pressure_oversampling_t oversampling);

/**
 * @brief Reads temperature oversampling setting from BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] oversampling BMP390 temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_temperature_oversampling(bmp390_handle_t handle, bmp390_temperature_oversampling_t *const oversampling);

/**
 * @brief Writes temperature oversampling setting to BMP390.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[in] oversampling BMP390 temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_set_temperature_oversampling(bmp390_handle_t handle, const bmp390_temperature_oversampling_t oversampling);

/**
 * @brief Reads output data rate setting from BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] output_data_rate BMP390 output data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_output_data_rate(bmp390_handle_t handle, bmp390_output_data_rates_t *const output_data_rate);

/**
 * @brief writes output data rate setting to bmp390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[in] output_data_rate BMP390 output data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_set_output_data_rate(bmp390_handle_t handle, const bmp390_output_data_rates_t output_data_rate);

/**
 * @brief Reads IIR filter setting to BMP390.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[out] iir_filter BMP390 IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_get_iir_filter(bmp390_handle_t handle, bmp390_iir_filters_t *const iir_filter);

/**
 * @brief Writes IIR filter setting from BMP390.  See datasheet, section 3.4, table 7.
 * 
 * @param[in] handle BMP390 device handle.
 * @param[in] iir_filter BMP390 IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_set_iir_filter(bmp390_handle_t handle, const bmp390_iir_filters_t iir_filter);

/**
 * @brief Issues soft-reset sensor and initializes registers for BMP390.
 *
 * @param[in] handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_reset(bmp390_handle_t handle);

/**
 * @brief removes an BMP390 device from master bus.
 *
 * @param[in] handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_remove(bmp390_handle_t handle);

/**
 * @brief Removes an BMP390 device from master I2C bus and delete the handle.
 * 
 * @param[in] handle BMP390 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bmp390_delete(bmp390_handle_t handle);

/**
 * @brief Converts BMP390 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* BMP390 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* bmp390_get_fw_version(void);

/**
 * @brief Converts BMP390 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t BMP390 firmware version number.
 */
int32_t bmp390_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BMP390_H__
