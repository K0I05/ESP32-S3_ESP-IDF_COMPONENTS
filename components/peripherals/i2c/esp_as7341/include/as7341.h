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
 * @file as7341.h
 * @defgroup drivers as7341
 * @{
 *
 * ESP-IDF driver for as7341 11-channel spectrometer
 * 
 * https://github.com/DFRobot/DFRobot_AS7341/blob/master/DFRobot_AS7341.cpp
 * https://github.com/adafruit/Adafruit_AS7341/blob/master/Adafruit_AS7341.cpp
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AS7341_H__
#define __AS7341_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "as7341_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * AS7341 definitions
*/
#define I2C_AS7341_DEV_CLK_SPD           UINT32_C(100000)  //!< as7341 I2C default clock frequency (100KHz)

#define I2C_AS7341_DEV_ADDR              UINT8_C(0x39)     //!< as7341 I2C address


/*
 * AS7341 macro definitions
*/
#define AS7341_CONFIG_DEFAULT {                      \
    .i2c_address        = I2C_AS7341_DEV_ADDR,           \
    .i2c_clock_speed    = I2C_AS7341_DEV_CLK_SPD,        \
    .spectral_gain      = AS7341_SPECTRAL_GAIN_32X,      \
    .power_enabled      = true,                          \
    .atime              = 29,                            \
    .astep              = 599 }

/*
 * AS7341 enumerator and structure declarations
*/

/**
 * @brief AS7341 ambient light sensing mode enumerator.
 */
typedef enum as7341_als_modes_e {
    AS7341_ALS_SPM_MODE         = (0), /*!< as7341 spectral measurement, normal mode */
    AS7341_ALS_SYNS_MODE        = (1), /*!< as7341 SYNS mode */
    AS7341_ALS_RESERVED_MODE    = (2), /*!< as7341 reserved, do not use */
    AS7341_ALS_SYND_MODE        = (3)  /*!< as7341 SYND mode, use spectra data registers 0x60 to 0x6F in this mode */
} as7341_als_modes_t;

/**
 * @brief AS7341 led driving strengths enumerator.
 */
typedef enum as7341_led_drive_strengths_e {
    AS7341_LED_DRIVE_STRENGTH_4MA         = (0b0000000), /*!< as7341  */
    AS7341_LED_DRIVE_STRENGTH_6MA         = (0b0000001), /*!< as7341  */
    AS7341_LED_DRIVE_STRENGTH_8MA         = (0b0000010), /*!< as7341  */
    AS7341_LED_DRIVE_STRENGTH_10MA        = (0b0000011), /*!< as7341  */
    AS7341_LED_DRIVE_STRENGTH_12MA        = (0b0000100), /*!< as7341 (default) */

    AS7341_LED_DRIVE_STRENGTH_256MA       = (0b1111110), /*!< as7341  */
    AS7341_LED_DRIVE_STRENGTH_258MA       = (0b1111111), /*!< as7341  */
} as7341_led_drive_strengths_t;

/**
 * @brief AS7341 allowable gain multipliers enumerator.
 */
typedef enum as7341_spectral_gains_e {
    AS7341_SPECTRAL_GAIN_0_5X = 0,
    AS7341_SPECTRAL_GAIN_1X,
    AS7341_SPECTRAL_GAIN_2X,
    AS7341_SPECTRAL_GAIN_4X,
    AS7341_SPECTRAL_GAIN_8X,
    AS7341_SPECTRAL_GAIN_16X,
    AS7341_SPECTRAL_GAIN_32X,
    AS7341_SPECTRAL_GAIN_64X,
    AS7341_SPECTRAL_GAIN_128X,
    AS7341_SPECTRAL_GAIN_256X,
    AS7341_SPECTRAL_GAIN_512X,
} as7341_spectral_gains_t;

/**
 * @brief AS7341 allowable flicker detection gain multipliers enumerator.
 */
typedef enum as7341_flicker_detection_gains_e {
    AS7341_FLICKER_DETECTION_GAIN_0_5X = 0,
    AS7341_FLICKER_DETECTION_GAIN_1X,
    AS7341_FLICKER_DETECTION_GAIN_2X,
    AS7341_FLICKER_DETECTION_GAIN_4X,
    AS7341_FLICKER_DETECTION_GAIN_8X,
    AS7341_FLICKER_DETECTION_GAIN_16X,
    AS7341_FLICKER_DETECTION_GAIN_32X,
    AS7341_FLICKER_DETECTION_GAIN_64X,
    AS7341_FLICKER_DETECTION_GAIN_128X,
    AS7341_FLICKER_DETECTION_GAIN_256X,
    AS7341_FLICKER_DETECTION_GAIN_512X,
} as7341_flicker_detection_gains_t;

/**
 * @brief AS7341 flicker detection states enumerator.
 */
typedef enum as7341_flicker_detection_states_e {
    AS7341_FLICKER_DETECTION_INVALID = 0,   /*!< flicker detection is invalid */
    AS7341_FLICKER_DETECTION_UNKNOWN,       /*!< flicker detection valid but unknown */
    AS7341_FLICKER_DETECTION_SATURATED,     /*!< flicker detection is saturated */
    AS7341_FLICKER_DETECTION_100HZ,         /*!< flicker detection at 100 hz*/
    AS7341_FLICKER_DETECTION_120HZ          /*!< flicker detection at 120 hz*/
} as7341_flicker_detection_states_t;

/**
 * @brief AS7341 available SMUX commands enumerator.
 */
typedef enum as7341_smux_commands_e {
    AS7341_SMUX_CMD_ROM_RESET = 0,  ///< ROM code initialization of SMUX
    AS7341_SMUX_CMD_READ,       ///< Read SMUX configuration to RAM from SMUX chain
    AS7341_SMUX_CMD_WRITE,      ///< Write SMUX configuration from RAM to SMUX chain
} as7341_smux_commands_t;



/**
 * violet -> F1
 * indigo -> F2
 * blue   -> F3
 * cyan   -> F4
 * green  -> F5
 * yellow -> F6
 * orange -> F7
 * red    -> F8
 */



/**
 * @brief AS7341 channels basic counts data structure.
 */
typedef struct as7341_channels_basic_counts_data_s {
    float f1;       /*!< 405 to 425 nm */
    float f2;       /*!< 435 to 455 nm */
    float f3;       /*!< 470 to 490 nm */
    float f4;       /*!< 505 to 525 nm */
    float f5;       /*!< 545 to 565 nm */
    float f6;       /*!< 580 to 600 nm */
    float f7;       /*!< 620 to 640 nm */
    float f8;       /*!< 670 to 690 nm */
    float clear;    /*!< */
    float nir;      /*!< */
} as7341_channels_basic_counts_data_t;

/**
 * @brief AS7341 channels spectral data structure.
 */
typedef struct as7341_channels_spectral_data_s {
    uint16_t f1;    /*!< 405 to 425 nm */
    uint16_t f2;    /*!< 435 to 455 nm */
    uint16_t f3;    /*!< 470 to 490 nm */
    uint16_t f4;    /*!< 505 to 525 nm */
    uint16_t f5;    /*!< 545 to 565 nm */
    uint16_t f6;    /*!< 580 to 600 nm */
    uint16_t f7;    /*!< 620 to 640 nm */
    uint16_t f8;    /*!< 670 to 690 nm */
    uint16_t clear; /*!< */
    uint16_t nir;   /*!< */
} as7341_channels_spectral_data_t;

/**
 * @brief AS7341 configuration structure definition.
 */
typedef struct as7341_config_s {
    uint16_t                    i2c_address;          /*!< as7341 i2c device address */
    uint32_t                    i2c_clock_speed;      /*!< as7341 i2c device scl clock speed in hz */
    uint8_t                     atime;
    uint16_t                    astep;
    as7341_spectral_gains_t     spectral_gain;
    bool                        power_enabled;    /*!< enable or disable power to as7341  */
} as7341_config_t;



/**
 * @brief AS7341 opaque handle structure definition.
 */
typedef void* as7341_handle_t;



/**
 * @brief Initializes an AS7341 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] as7341_config AS7341 device configuration.
 * @param[out] as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_init(i2c_master_bus_handle_t master_handle, const as7341_config_t *as7341_config, as7341_handle_t *as7341_handle);

/**
 * @brief Reads spectral sensors measurements, F1 to F8, Clear and NIR, from AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] spectral_data Spectral sensors data from AS7341.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_spectral_measurements(as7341_handle_t handle, as7341_channels_spectral_data_t *const spectral_data);

/**
 * @brief Converts AS7341 spectral sensors measurements to basic counts.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[in] spectral_data Spectral sensors data from AS7341.
 * @param[out] basic_counts_data Converted basic counts data from spectral sensors data.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_basic_counts(as7341_handle_t handle, const as7341_channels_spectral_data_t spectral_data, as7341_channels_basic_counts_data_t *const basic_counts_data);

/**
 * @brief Reads flicker detection status from AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] state Flicker detection state, 100Hz, 120Hz or flicker saturation was detected.
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT if operation timed out.
 */
esp_err_t as7341_get_flicker_detection_status(as7341_handle_t handle, as7341_flicker_detection_states_t *const state);

/**
 * @brief Reads data status from AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] ready Data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_data_status(as7341_handle_t handle, bool *const ready);

/**
 * @brief Reads the number of integration steps for the ADC integration time from AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] atime Number of integration steps from 1 to 256, a value of 29 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_atime(as7341_handle_t handle, uint8_t *const atime);

/**
 * @brief Writes the number of integration steps for the ADC integration time to AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[in] atime Number of integration steps from 1 to 256, a value of 29 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_set_atime(as7341_handle_t handle, const uint8_t atime);

/**
 * @brief Reads the number of integration time steps for the ADC integration time from AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] astep Integration time step size.  Integration time step increment of 2.78us, a value of 599 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_astep(as7341_handle_t handle, uint16_t *const astep);

/**
 * @brief Writes the number of integration time steps for the ADC integration time to AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[in] astep Integration time step size.  Integration time step increment of 2.78us, a value of 599 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_set_astep(as7341_handle_t handle, const uint16_t astep);

/**
 * @brief Reads spectral gain setting from AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] gain AS7341 spectral gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_spectral_gain(as7341_handle_t handle, as7341_spectral_gains_t *const gain);

/**
 * @brief Writes spectral gain setting to AS7341.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[in] gain AS7341 spectral gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_set_spectral_gain(as7341_handle_t handle, const as7341_spectral_gains_t gain);

/**
 * @brief Reads ambient light sensing mode from AS7341.  SPM mode (spectral measurement), normal mode, is configured by default.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[out] mode AS7341 ambient light sensing mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_get_ambient_light_sensing_mode(as7341_handle_t handle, as7341_als_modes_t *const mode);

/**
 * @brief Writes ambient light sensing mode to AS7341.  SPM mode (spectral measurement), normal mode, is configured by default.
 * 
 * @param[in] handle AS7341 device handle.
 * @param[in] mode AS7341 ambient light sensing mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_set_ambient_light_sensing_mode(as7341_handle_t handle, const as7341_als_modes_t mode);

/**
 * @brief Enables AS7341 flicker detection.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_enable_flicker_detection(as7341_handle_t handle);

/**
 * @brief Disables AS7341 flicker detection.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_disable_flicker_detection(as7341_handle_t handle);

/**
 * @brief Enables AS7341 supper multiplier (SMUX) special interrupt.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_enable_smux(as7341_handle_t handle);

/**
 * @brief Enables AS7341 wait time between two consecutive spectral measurements.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_enable_wait_time(as7341_handle_t handle);

/**
 * @brief Disables AS7341 wait time between two consecutive spectral measurements.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_disable_wait_time(as7341_handle_t handle);

/**
 * @brief Enables AS7341 spectral measurement.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_enable_spectral_measurement(as7341_handle_t handle);

/**
 * @brief Disables AS7341 spectral measurement.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_disable_spectral_measurement(as7341_handle_t handle);

/**
 * @brief Enables AS7341 power.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_enable_power(as7341_handle_t handle);

/**
 * @brief Disables AS7341 power.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_disable_power(as7341_handle_t handle);

/**
 * @brief Enables AS7341 onboard LED.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_enable_led(as7341_handle_t handle);

/**
 * @brief Disables AS7341 onboard LED.
 * 
 * @param[in] handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_disable_led(as7341_handle_t handle);

/**
 * @brief Removes an AS7341 device from master bus.
 *
 * @param[in] handle AS7341 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_remove(as7341_handle_t handle);

/**
 * @brief Removes an AS7341 device from master bus and frees handle.
 * 
 * @param[in] handle AS7341 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as7341_delete(as7341_handle_t handle);

/**
 * @brief Converts AS7341 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* AS7341 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* as7341_get_fw_version(void);

/**
 * @brief Converts AS7341 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t AS7341 firmware version number.
 */
int32_t as7341_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __AS7341_H__
