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
 * @file bh1750.h
 * @defgroup drivers bh1750
 * @{
 *
 * ESP-IDF driver for bh1750 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BH1750_H__
#define __BH1750_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "bh1750_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BH1750 definitions
*/
#define I2C_BH1750_DEV_CLK_SPD          UINT32_C(100000)    //!< bh1750 I2C default clock frequency (100KHz)

#define I2C_BH1750_DEV_ADDR_LO          UINT8_C(0x23)       //!< bh1750 I2C address when ADDR pin floating/low
#define I2C_BH1750_DEV_ADDR_HI          UINT8_C(0x5C)       //!< bh1750 I2C address when ADDR pin high


/*
 * macros definitions
*/

#define I2C_BH1750_CONFIG_DEFAULT {                             \
        .i2c_address        = I2C_BH1750_DEV_ADDR_LO,           \
        .i2c_clock_speed    = I2C_BH1750_DEV_CLK_SPD,           \
        .mode               = BH1750_MODE_CM_HI_RESOLUTION,     \
        .power_enabled      = true,                             \
        .set_timespan       = false }

/*
 * BH1750 enumerator and structure declarations
*/

/**
 * @brief BH1750 measurement modes enumerator.
 * 
 */
typedef enum bh1750_measurement_modes_e {
    BH1750_MODE_OM_HI_RESOLUTION  = (0b00100000), //!< one time measurement high resolution (1 lx) mode, goes into power down mode after measurement
    BH1750_MODE_OM_HI2_RESOLUTION = (0b00100001), //!< one time measurement high resolution (0.5 lx) mode 2, goes into power down mode after measurement
    BH1750_MODE_OM_LO_RESOLUTION  = (0b00100011), //!< one time measurement low resolution (4 lx) mode, goes into power down mode after measurement
    BH1750_MODE_CM_HI_RESOLUTION  = (0b00010000), //!< continuous measurement high resolution (1 lx) mode
    BH1750_MODE_CM_HI2_RESOLUTION = (0b00010001), //!< continuous measurement high resolution (0.5 lx) mode 2
    BH1750_MODE_CM_LO_RESOLUTION  = (0b00010011)  //!< continuous measurement low resolution (4 lx) mode
} bh1750_measurement_modes_t;

/**
 * @brief BH1750 device configuration structure.
 */
typedef struct bh1750_config_s {
    uint16_t                        i2c_address;    /*!< bh1750 i2c device address */
    uint32_t                        i2c_clock_speed;/*!< bh1750 i2c device scl clock speed in hz */
    bh1750_measurement_modes_t      mode;           /*!< bh1750 measurement mode */
    uint8_t                         timespan;       /*!< bh1750 measurement time duration */
    bool                            set_timespan;   /*!< set bh1750 measurement timespan when true */
    bool                            power_enabled;  /*!< bh1750 powered up at initialization */
} bh1750_config_t;

/**
 * @brief BH1750 I2C device handle structure.
 */
struct bh1750_context_t {
    bh1750_config_t                 dev_config;     /*!< bh1750 device configuration */ 
    i2c_master_dev_handle_t         i2c_handle;     /*!< bh1750 I2C device handle */
};

/**
 * @brief BH1750 context structure definition.
 */
typedef struct bh1750_context_t bh1750_context_t;

/**
 * @brief BH1750 handle structure definition.
 */
typedef struct bh1750_context_t *bh1750_handle_t;



/**
 * @brief initializes an BH1750 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle
 * @param[in] bh1750_config configuration of BH1750 device
 * @param[out] bh1750_handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bh1750_init(i2c_master_bus_handle_t master_handle, const bh1750_config_t *bh1750_config, bh1750_handle_t *bh1750_handle);

/**
 * @brief measure BH1750 illuminance.  BH1750 goes into power-down mode after measurement when one-time measurements are configured.
 *
 * @param[in] handle BH1750 device handle
 * @param[out] ambient_light BH1750 illuminance measurement
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bh1750_get_ambient_light(bh1750_handle_t handle, float *const ambient_light);

/**
 * @brief Writes measurement mode to bh1750.
 *
 * @param[in] handle bh1750 device handle.
 * @param[in] mode bh1750 measurement mode.
 * @return ESP_OK on success.
 */
esp_err_t bh1750_set_measurement_mode(bh1750_handle_t handle, const bh1750_measurement_modes_t mode);

/**
 * @brief sets bh1750 sensor measurement time. see datasheet for details.
 *
 * @param[in] handle bh1750 device handle
 * @param[in] timespan bh1750 measurement time duration from 31 to 254 (typical 69)
 * @return ESP_OK on success.
 */
esp_err_t bh1750_set_measurement_time(bh1750_handle_t handle, const uint8_t timespan);

/**
 * @brief power-up BH1750 sensor.
 *
 * @param[in] handle BH1750 device handle
 * @return esp_err_t  ESP_OK on success.
 */
esp_err_t bh1750_enable_power(bh1750_handle_t handle);

/**
 * @brief power-down BH1750 sensor.
 *
 * @param[in] handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bh1750_disable_power(bh1750_handle_t handle);

/**
 * @brief soft-reset BH1750 sensor. Reset command is not acceptable in power-down mode.
 *
 * @param[in] handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bh1750_reset(bh1750_handle_t handle);

/**
 * @brief removes an BH1750 device from master bus.
 *
 * @param[in] handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bh1750_remove(bh1750_handle_t handle);

/**
 * @brief removes an BH1750 device from master bus and frees handle.
 *
 * @param[in] handle BH1750 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bh1750_delete(bh1750_handle_t handle);

/**
 * @brief Converts BH1750 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* BH1750 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* bh1750_get_fw_version(void);

/**
 * @brief Converts BH1750 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t BH1750 firmware version number.
 */
int32_t bh1750_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BH1750_H__
