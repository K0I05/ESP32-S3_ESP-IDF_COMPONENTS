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
 * @file hmc5883l.h
 * @defgroup drivers hmc5883l
 * @{
 *
 * ESP-IDF driver for hmc5883l sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __HMC5883L_H__
#define __HMC5883L_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "hmc5883l_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HMC5883L definitions
*/
#define I2C_HMC5883L_DEV_CLK_SPD                UINT32_C(100000)                //!< hmc5883l I2C default clock frequency (100KHz)

#define I2C_HMC5883L_DEV_ADDR                   UINT8_C(0x1e)           //!< hmc5883l I2C address when ADDR pin floating/low

/*
 * macro definitions
*/
#define HMC5883L_CONFIG_DEFAULT {                                   \
        .i2c_address                    = I2C_HMC5883L_DEV_ADDR,        \
        .i2c_clock_speed                = I2C_HMC5883L_DEV_CLK_SPD,     \
        .mode                           = HMC5883L_MODE_CONTINUOUS, \
        .sample                         = HMC5883L_SAMPLE_4,        \
        .rate                           = HMC5883L_DATA_RATE_15_00, \
        .gain                           = HMC5883L_GAIN_390,        \
        .bias                           = HMC5883L_BIAS_NORMAL,     \
        .declination                    = -16.0f }

/*
 * HMC5883L enumerator and structure declarations
*/

/**
 * HMC5883L possible calibration options
 */
typedef enum hmc5883l_calibration_options_e {
    HMC5883L_CAL_GAIN_DIFF = 1, /*!< calculates the difference in the gain of the each axis magnetometer axis */
    HMC5883L_CAL_AXES_MEAN = 2, /*!< calculates the mean of each axes magnetic field, when the Magnetometer is rotated 360 degree */
    HMC5883L_CAL_BOTH      = 3  /*!< do both */
} hmc5883l_calibration_options_t;

/**
 * HMC5883L possible operating modes
 */
typedef enum hmc5883l_modes_e {
    HMC5883L_MODE_CONTINUOUS = (0b00), //!< Continuous mode
    HMC5883L_MODE_SINGLE     = (0b01), //!< Single measurement mode, default
    HMC5883L_MODE_IDLE       = (0b10), //!< Idle mode
} hmc5883l_modes_t;

/**
 * HMC5883L number of samples averaged per measurement
 */
typedef enum hmc5883l_sample_averages_e {
    HMC5883L_SAMPLE_1 = (0b00), //!< 1 sample, default
    HMC5883L_SAMPLE_2 = (0b01), //!< 2 samples
    HMC5883L_SAMPLE_4 = (0b10), //!< 4 samples
    HMC5883L_SAMPLE_8 = (0b11)  //!< 8 samples
} hmc5883l_sample_averages_t;

/**
 * HMC5883L possible data output rate in continuous measurement mode
 */
typedef enum hmc5883l_data_rates_e {
    HMC5883L_DATA_RATE_00_75  = (0b000), //!< 0.75 Hz
    HMC5883L_DATA_RATE_01_50  = (0b001), //!< 1.5 Hz
    HMC5883L_DATA_RATE_03_00  = (0b010), //!< 3 Hz
    HMC5883L_DATA_RATE_07_50  = (0b011), //!< 7.5 Hz
    HMC5883L_DATA_RATE_15_00  = (0b100), //!< 15 Hz, default
    HMC5883L_DATA_RATE_30_00  = (0b101), //!< 30 Hz
    HMC5883L_DATA_RATE_75_00  = (0b110), //!< 75 Hz
//    HMC5883L_DATA_RATE_RESERVED = (0b111)  //!< 220 Hz, HMC5983 only
} hmc5883l_data_rates_t;

/**
 * HMC5883L possible measurement mode of the device (bias)
 */
typedef enum hmc5883l_biases_e {
    HMC5883L_BIAS_NORMAL   = (0b00), //!< Default flow, no bias
    HMC5883L_BIAS_POSITIVE = (0b01),   //!< Positive bias configuration all axes, used for self test (see datasheet)
    HMC5883L_BIAS_NEGATIVE = (0b10),    //!< Negative bias configuration all axes, used for self test (see datasheet)
//    HMC5883L_BIAS_RESERVED = (0b11)
} hmc5883l_biases_t;

/**
 * HMC5883L possible device gains
 */
typedef enum hmc5883l_gains_e {
    HMC5883L_GAIN_1370 = (0b000), //!< 0.73 mG/LSb, range -0.88..+0.88 G
    HMC5883L_GAIN_1090 = (0b001), //!< 0.92 mG/LSb, range -1.3..+1.3 G, default
    HMC5883L_GAIN_820  = (0b010), //!< 1.22 mG/LSb, range -1.9..+1.9 G
    HMC5883L_GAIN_660  = (0b011), //!< 1.52 mG/LSb, range -2.5..+2.5 G
    HMC5883L_GAIN_440  = (0b100), //!< 2.27 mG/LSb, range -4.0..+4.0 G
    HMC5883L_GAIN_390  = (0b101), //!< 2.56 mG/LSb, range -4.7..+4.7 G
    HMC5883L_GAIN_330  = (0b110), //!< 3.03 mG/LSb, range -5.6..+5.6 G
    HMC5883L_GAIN_230  = (0b111)  //!< 4.35 mG/LSb, range -8.1..+8.1 G
} hmc5883l_gains_t;

/**
 * HMC5883L raw measurement result
 */
typedef struct hmc5883l_axes_data_s {
    int16_t x_axis;
    int16_t y_axis;
    int16_t z_axis;
} hmc5883l_axes_data_t;

/**
 * HMC5883L measurement result, milligauss
 */
typedef struct hmc5883l_magnetic_axes_data_s {
    float x_axis;   /*!< x axis mG */
    float y_axis;   /*!< y axis mG */
    float z_axis;   /*!< z axis mG */
    float heading;  /*!< heading in degrees */
} hmc5883l_magnetic_axes_data_t;

typedef struct hmc5883l_offset_axes_data_s {
    float x_axis;   /*!< x axis */
    float y_axis;   /*!< y axis */
    float z_axis;   /*!< z axis */
} hmc5883l_offset_axes_data_t;

typedef struct hmc5883l_gain_error_axes_data_s {
    float x_axis;   /*!< x axis */
    float y_axis;   /*!< y axis */
    float z_axis;   /*!< z axis */
} hmc5883l_gain_error_axes_data_t;

/**
 * @brief HMC5883L device configuration structure definition.
 */
typedef struct hmc5883l_config_s {
    uint16_t                    i2c_address;    /*!< ens160 i2c device address */
    uint32_t                    i2c_clock_speed;/*!< ens160 i2c device scl clock speed in hz */
    hmc5883l_modes_t            mode;           /*!< operating mode */
    hmc5883l_sample_averages_t  sample;         /*!< number of samples averaged */
    hmc5883l_data_rates_t       rate;           /*!< data rate */
    hmc5883l_gains_t            gain;           /*!< used measurement mode */
    hmc5883l_biases_t           bias;           /*!< measurement mode (bias) */
    float                       declination;    /*!< magnetic declination angle http://www.magnetic-declination.com/ */
} hmc5883l_config_t;


/**
 * @brief HMC5883L opaque handle structure definition.
 */
typedef void* hmc5883l_handle_t;



/**
 * @brief Initializes an HMC5883L device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] hmc5883l_config HMC5883L device configuration.
 * @param[out] hmc5883l_handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_init(i2c_master_bus_handle_t master_handle, const hmc5883l_config_t *hmc5883l_config, hmc5883l_handle_t *hmc5883l_handle);

/**
 * @brief Reads uncompensated axes measurements from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param axes_data Uncompensated axes measurements (x, y, and z axes).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_fixed_magnetic_axes(hmc5883l_handle_t handle, hmc5883l_axes_data_t *const axes_data);

/**
 * @brief Reads compensated magnetic axes measurements from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param axes_data Compensated magnetic axes measurements (x, y, and z axes).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_magnetic_axes(hmc5883l_handle_t handle, hmc5883l_magnetic_axes_data_t *const axes_data);


/* under test */

esp_err_t hmc5883l_get_calibrated_offsets(hmc5883l_handle_t handle, const hmc5883l_calibration_options_t option);


/**
 * @brief Reads data status from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param ready HMC5883L data is ready.
 * @param locked HMC5883L data is locked.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_data_status(hmc5883l_handle_t handle, bool *const ready, bool *const locked);

/**
 * @brief Reads operating mode setting from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param mode HMC5883L operating mode setting.
 * @return esp_err_t ESP_OK on success.
 */

esp_err_t hmc5883l_get_mode(hmc5883l_handle_t handle, hmc5883l_modes_t *const mode);
/**
 * @brief Writes operating mode setting to HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param mode HMC5883L operating mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_set_mode(hmc5883l_handle_t handle, const hmc5883l_modes_t mode);

/**
 * @brief Reads samples averaged setting from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param sample HMC5883L samples averaged setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_samples_averaged(hmc5883l_handle_t handle, hmc5883l_sample_averages_t *const sample);

/**
 * @brief Writes samples averaged setting to HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param sample HMC5883L samples averaged setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_set_samples_averaged(hmc5883l_handle_t handle, const hmc5883l_sample_averages_t sample);

/**
 * @brief Reads data rate setting from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param rate HMC5883L data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_data_rate(hmc5883l_handle_t handle, hmc5883l_data_rates_t *const rate);

/**
 * @brief Writes data rate setting to HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param rate HMC5883L data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_set_data_rate(hmc5883l_handle_t handle, const hmc5883l_data_rates_t rate);

/**
 * @brief Reads measurement mode bias setting from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param bias HMC5883L measurement mode bias setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_bias(hmc5883l_handle_t handle, hmc5883l_biases_t *const bias);

/**
 * @brief Writes measurement mode bias setting to HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param bias HMC5883L measurement mode bias setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_set_bias(hmc5883l_handle_t handle, const hmc5883l_biases_t bias);

/**
 * @brief Reads gain setting from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param gain HMC5883L gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_gain(hmc5883l_handle_t handle, hmc5883l_gains_t *const gain);

/**
 * @brief Writes gain setting to HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param gain HMC5883L gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_set_gain(hmc5883l_handle_t handle, const hmc5883l_gains_t gain);

/**
 * @brief Reads gain sensitivity setting from HMC5883L.
 * 
 * @param handle HMC5883L device handle.
 * @param sensitivity HMC5883L gain sensitivity setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_get_gain_sensitivity(hmc5883l_handle_t handle, float *const sensitivity);

/**
 * @brief Removes an HMC5883L device from master bus.
 *
 * @param[in] handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_remove(hmc5883l_handle_t handle);

/**
 * @brief Removes an HMC5883L device from master bus and frees handle.
 * 
 * @param[in] handle HMC5883L device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hmc5883l_delete(hmc5883l_handle_t handle);

/**
 * @brief Converts HMC5883L firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* HMC5883L firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* hmc5883l_get_fw_version(void);

/**
 * @brief Converts HMC5883L firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t HMC5883L firmware version number.
 */
int32_t hmc5883l_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __HMC5883L_H__
