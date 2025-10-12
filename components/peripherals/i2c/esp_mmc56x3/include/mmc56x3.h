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
 * @file mmc56x3.h
 * @defgroup drivers mmc56x3
 * @{
 *
 * ESP-IDF driver for mmc56x3 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MMC56X3_H__
#define __MMC56X3_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "mmc56x3_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MMC56X3 definitions
*/
#define I2C_MMC56X3_DEV_CLK_SPD             UINT32_C(100000)  //!< mmc56x3 I2C default clock frequency (100KHz)

#define I2C_MMC56X3_DEV_ADDR                UINT8_C(0x30)     //!< mmc56x3 I2C address


/*
 * MMC56X3 macro definitions
*/

/**
 * @brief Macro that initializes `mmc56x3_config_t` to default configuration settings.
 */
#define I2C_MMC56X3_CONFIG_DEFAULT {                                 \
        .i2c_address                    = I2C_MMC56X3_DEV_ADDR,      \
        .i2c_clock_speed                = I2C_MMC56X3_DEV_CLK_SPD,   \
        .data_rate                      = 0,                         \
        .continuous_mode_enabled        = false,                     \
        .declination                    = -16.0f }

/*
 * MMC56X3 enumerator and structure declarations
*/

/**
 * @brief MMC56X3 measurement times enumerator.
 */
typedef enum mmc56x3_measurement_times_e {
    MMC56X3_MEAS_TIME_6_6MS = (0b00),
    MMC56X3_MEAS_TIME_3_5MS = (0b01),
    MMC56X3_MEAS_TIME_2MS   = (0b10),
    MMC56X3_MEAS_TIME_1_2MS = (0b11),
} mmc56x3_measurement_times_t;

/**
 * @brief MMC56X3 measurement samples enumerator.
 */
typedef enum mmc56x3_measurement_samples_e {
    MMC56X3_MEAS_SAMPLE_1    = (0b000),
    MMC56X3_MEAS_SAMPLE_25   = (0b001),
    MMC56X3_MEAS_SAMPLE_75   = (0b010),
    MMC56X3_MEAS_SAMPLE_100  = (0b011),
    MMC56X3_MEAS_SAMPLE_250  = (0b100),
    MMC56X3_MEAS_SAMPLE_500  = (0b101),
    MMC56X3_MEAS_SAMPLE_1000 = (0b110),
    MMC56X3_MEAS_SAMPLE_2000 = (0b111),
} mmc56x3_measurement_samples_t;

/**
 * @brief MMC56X3 status 1 register (0x18 Read) (POR 0x00) structure.
 */
typedef union __attribute__((packed)) mmc56x3_status_register_u {
    struct MMC56X3_STS_REG_BITS_TAG {
        uint8_t             reserved:4;       /*!< reserved and set to 0                                    (bit:0-3) */
        bool                otp_read_done:1;  /*!< true when otp memory read                                (bit:4)   */
        bool                selftest:1;       /*!< false when selftest pass                                 (bit:5)   */
        bool                data_ready_m:1;   /*!< true when magnetic measurement done and data is ready, reset when any magnetic data register is read (bit:6)   */
        bool                data_ready_t:1;   /*!< true when temperature measurement done and data is ready, reset when temperature data register is read (bit:7)   */
    } bits;            /*!< represents the 8-bit status 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status 1 register as `uint8_t` and has a reset-value of 0x00.   */
} mmc56x3_status_register_t;

/**
 * @brief MMC56X3 control 0 register (0x1B Write) (POR 0x00) structure.
 */
typedef union __attribute__((packed)) mmc56x3_control0_register_u {
    struct CTRL0_REG_BITS_TAG {
        bool                sample_m:1;     /*!< perform magnetic measurement when true and self clears at the end of the measurement  (bit:0) */
        bool                sample_t:1;     /*!< perform temperature measurement when true and self clears at the end of the measurement  (bit:1) */
        uint8_t             reserved:1;     /*!< reserved and set to 0                                  (bit:2) */
        bool                do_set:1;       /*!< perform set operation when true and self clears at the end of set operation (bit:3) */
        bool                do_reset:1;     /*!< perform reset operation when true and self clears at the end of reset operation (bit:4) */
        bool                auto_sr_enabled:1;   /*!< enable automatic periodic set/reset when true  (bit:5) */
        bool                auto_st_enabled:1;   /*!< enable automatic selftest when true and self clears at the end of the operation, set registers 0x1e, 0x1f, and 0x20 beforehand (bit:6) */
        bool                continuous_freq_enabled:1;  /*!< enable measurement period calculation when true and self clears once measurement period is calculated (bit:7) */
    } bits;            /*!< represents the 8-bit control 0 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control 0 register as `uint8_t` and has a reset-value of 0x00.   */
} mmc56x3_control0_register_t;

/**
 * @brief MMC56X3 control 1 register (0x1C Write) (POR 0x00) structure.
 */
typedef union __attribute__((packed)) mmc56x3_control1_register_u {
    struct CTRL1_REG_BITS_TAG {
        mmc56x3_measurement_times_t     bandwidth:2;     /*!< measurement time                           (bit:0-1) */
        bool                            x_disabled:1;    /*!< x channel disabled true                      (bit:2) */
        bool                            y_disabled:1;    /*!< y channel disabled true                      (bit:3) */
        bool                            z_disabled:1;    /*!< z channel disabled true                      (bit:4) */
        bool                            st_enp_enabled:1;/*!< bring selftest coil dc current when true   (bit:5) */
        bool                            st_enm_enabled:1;/*!< same as st_enp but opposite polarity       (bit:6) */
        bool                            sw_reset:1;      /*!< causes software-reset when true            (bit:7) */
    } bits;            /*!< represents the 8-bit control 1 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control 1 register as `uint8_t` and has a reset-value of 0x00.   */
} mmc56x3_control1_register_t;

/**
 * @brief MMC56X3 control 2 register (0x1D Write) (POR 0x00) structure.
 */
typedef union __attribute__((packed)) mmc56x3_control2_register_u {
    struct CTRL2_REG_BITS_TAG {
        mmc56x3_measurement_samples_t periodical_set_samples:3;  /*!< number of samples before set is executed, period set and auto set-reset must be enabled (bit:0-2) */
        bool                periodical_set_enabled:1;  /*!< perform periodical set when true               (bit:3) */
        bool                continuous_enabled:1;  /*!< continuous mode when true, data period (odr) must be non-zero and contineous frequency enabled beforehand (bit:4) */
        uint8_t             reserved:2;            /*!< reserved and set to 0                          (bit:5-6) */
        bool                h_power_enabled:1;     /*!< achieve 1000Hz odr when true                   (bit:7) */
    } bits;            /*!< represents the 8-bit control 2 register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit control 2 register as `uint8_t` and has a reset-value of 0x00.   */
} mmc56x3_control2_register_t;

/**
 * @brief MMC56X3 magnetic axes data structure.
 */
typedef struct mmc56x3_magnetic_axes_data_s {
    float x_axis;
    float y_axis;
    float z_axis;
} mmc56x3_magnetic_axes_data_t;

/**
 * @brief MMC56X3 self-test axes data structure.
 */
typedef struct mmc56x3_selftest_axes_data_s {
    uint8_t x_axis;
    uint8_t y_axis;
    uint8_t z_axis;
} mmc56x3_selftest_axes_data_t;

/**
 * @brief MMC56X3 configuration structure definition.
 */
typedef struct mmc56x3_config_s {
    uint16_t                        i2c_address;            /*!< mmc56x3 i2c device address */
    uint32_t                        i2c_clock_speed;        /*!< mmc56x3 i2c device scl clock speed  */
    uint16_t                        data_rate;              /*!< mmc56x3 device data rate (odr), 0-255 or 1000 for 1000Hz data rate configuration */
    bool                            continuous_mode_enabled;/*!< mmc56x3 device measurement mode configuration, data rate must be non-zero when enabled */
    mmc56x3_measurement_times_t     measurement_bandwidth;  /*!< mmc56x3 device measurement bandwith configuration */
    bool                            auto_sr_enabled;        /*!< mmc56x3 auto set-reset configuration */
    float                           declination;            /*!< magnetic declination angle http://www.magnetic-declination.com/ */
} mmc56x3_config_t;


/**
 * @brief MMC56X3 opaque handle structure definition.
 */
typedef void* mmc56x3_handle_t;



/**
 * @brief Initializes an MMC56X3 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] mmc56x3_config MMC56X3 device configuration.
 * @param[out] mmc56x3_handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_init(i2c_master_bus_handle_t master_handle, const mmc56x3_config_t *mmc56x3_config, mmc56x3_handle_t *mmc56x3_handle);

/**
 * @brief Reads temperature from MMC56X3. 
 * 
 * @param handle MMC56X3 device handle.
 * @param temperature Temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_get_temperature(mmc56x3_handle_t handle, float *const temperature);

/**
 * @brief Reads magnetic axes (x, y, z axes) from MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param axes_data Magnetic axes data (x, y, z axes) in mG.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_get_magnetic_axes(mmc56x3_handle_t handle, mmc56x3_magnetic_axes_data_t *const axes_data);

/**
 * @brief Reads temperature data status from MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param ready Temperature data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_get_temperature_data_status(mmc56x3_handle_t handle, bool *const ready);

/**
 * @brief Reads magnetic data status from MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param ready Magnetic data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_get_magnetic_data_status(mmc56x3_handle_t handle, bool *const ready);

/**
 * @brief Reads magnetic and temperature data status from MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param magnetic_ready Magnetic data is ready when true.
 * @param temperature_ready Temperature data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_get_data_status(mmc56x3_handle_t handle, bool *const magnetic_ready, bool *const temperature_ready);

/**
 * @brief Writes measurement mode to MMC56X3.  The data rate must be configured to a non-zero value before enabling continuous measurements.
 * 
 * @param handle MMC56X3 device handle.
 * @param continuous Measurement mode is continuous when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_set_measure_mode(mmc56x3_handle_t handle, const bool continuous);

/**
 * @brief Writes data rate to MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param rate MMC56X3 data rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_set_data_rate(mmc56x3_handle_t handle, const uint16_t rate);

/**
 * @brief Writes measurement bandwidth to MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param bandwith MMC56X3 measurement bandwidth setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_set_measure_bandwidth(mmc56x3_handle_t handle, const mmc56x3_measurement_times_t bandwidth);

/**
 * @brief Enables MMC56X3 periodical set when the number of samples threshold is met.
 * 
 * @param handle MMC56X3 device handle.
 * @param samples MMC56X3 measurement samples setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_enable_periodical_set(mmc56x3_handle_t handle, const mmc56x3_measurement_samples_t samples);

/**
 * @brief Disables MMC56X3 periodical set.
 * 
 * @param handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_disable_periodical_set(mmc56x3_handle_t handle);

/**
 * @brief Writes axes configuration to MMC56X3 to enable or disable axes (x, y, z), axes are enabled by default.
 * 
 * @param handle MMC56X3 device handle.
 * @param x_axis_disabled X-axis is enabled by default, set to true to disable x-axis.
 * @param y_axis_disabled Y-axis is enabled by default, set to true to disable y-axis.
 * @param z_axis_disabled Y-axis is enabled by default, set to true to disable z-axis.
 * @return esp_err_t ESP_OK on success.
 */
//esp_err_t mmc56x3_set_magnetic_axes(mmc56x3_handle_t handle, const bool x_axis_disabled, const bool y_axis_disabled, const bool z_axis_disabled);

/**
 * @brief Writes self-test axes data thresholds to MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param axes_data MMC56X3 axes data thresholds setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_set_selftest_thresholds(mmc56x3_handle_t handle, const mmc56x3_selftest_axes_data_t axes_data);

/**
 * @brief Reads self-test axes data set-values from MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param axes_data MMC56X3 axes data set-values setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_get_selftest_set_values(mmc56x3_handle_t handle, mmc56x3_selftest_axes_data_t *const axes_data);

/**
 * @brief Writes self-test axes data set-values to MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @param axes_data MMC56X3 axes data set-values setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_set_selftest_set_values(mmc56x3_handle_t handle, const mmc56x3_selftest_axes_data_t axes_data);

/**
 * @brief Pulses large currents through the sense coils to clear any offset.
 * 
 * @param[in] handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_magnetic_set_reset(mmc56x3_handle_t handle);

/**
 * @brief Issues soft-reset to MMC56X3.
 * 
 * @param handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_reset(mmc56x3_handle_t handle);

/**
 * @brief Removes an MMC56X3 device from master I2C bus.
 *
 * @param[in] handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_remove(mmc56x3_handle_t handle);

/**
 * @brief Removes an MMC56X3 device from master I2C bus and delete the handle.
 * 
 * @param[in] handle MMC56X3 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mmc56x3_delete(mmc56x3_handle_t handle);

/**
 * @brief Converts magnetic axes data to a heading.  See Honeywell application note AN-203 for details.
 * 
 * @param axes_data MMC56X3 magnetic axes data.
 * @return float Heading in degrees.
 */
float mmc56x3_convert_to_heading(const mmc56x3_magnetic_axes_data_t axes_data);

/**
 * @brief Converts magnetic axes data with magnetic declination to a true heading.  See Honeywell application note AN-203 for details.
 * 
 * @param declination MMC56X3 magnetic declination angle in degrees (http://www.magnetic-declination.com/) setting.
 * @param axes_data MMC56X3 magnetic axes data.
 * @return float True heading in degrees.
 */
float mmc56x3_convert_to_true_heading(const float declination, const mmc56x3_magnetic_axes_data_t axes_data);

/**
 * @brief Converts MMC56X3 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* MMC56X3 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* mmc56x3_get_fw_version(void);

/**
 * @brief Converts MMC56X3 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t MMC56X3 firmware version number.
 */
int32_t mmc56x3_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MMC56X3_H__
