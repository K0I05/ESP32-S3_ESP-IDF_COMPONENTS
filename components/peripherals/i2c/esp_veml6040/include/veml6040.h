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
 * @file veml6040.h
 * @defgroup drivers veml6040
 * @{
 *
 * ESP-IDF driver for veml6040 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __VEML6040_H__
#define __VEML6040_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "veml6040_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VEML6040 definitions
*/
#define I2C_VEML6040_DEV_CLK_SPD            UINT32_C(100000)    //!< veml6040 I2C default clock frequency (100KHz)

#define I2C_VEML6040_DEV_ADDR               UINT8_C(0x10)       //!< veml6040 I2C address

/*
 * VEML6040 macro definitions
*/

/**
 * @brief VEML6040 device configuration initialization default macro.
 */
#define I2C_VEML6040_CONFIG_DEFAULT {                                       \
            .i2c_address                = I2C_VEML6040_DEV_ADDR,            \
            .i2c_clock_speed            = I2C_VEML6040_DEV_CLK_SPD,         \
            .integration_time           = VEML6040_INTEGRATION_TIME_160MS,  \
            .shutdown_enabled           = false,                            \
            .mode                       = VEML6040_MODE_AUTO,               \
            .trigger_method             = VEML6040_TRIGGER_NONE,            \
        }

/*
 * VEML6040 enumerator and structure declarations
*/

/**
 * @brief VEML6040 channels enumerator.
 */
typedef enum {
    VEML6040_CHANNEL_RED   = 0,
    VEML6040_CHANNEL_GREEN = 1,
    VEML6040_CHANNEL_BLUE  = 2,
    VEML6040_CHANNEL_WHITE = 3,
} veml6040_channels_t;

/**
 * @brief VEML6040 integration times enumerator.
 */
typedef enum veml6040_integration_times_e {
    VEML6040_INTEGRATION_TIME_40MS   = (0b000),
    VEML6040_INTEGRATION_TIME_80MS   = (0b001),
    VEML6040_INTEGRATION_TIME_160MS  = (0b010),
    VEML6040_INTEGRATION_TIME_320MS  = (0b011),
    VEML6040_INTEGRATION_TIME_640MS  = (0b100),
    VEML6040_INTEGRATION_TIME_1280MS = (0b101),
} veml6040_integration_times_t;

/**
 * @brief VEML6040 triggers enumerator.
 */
typedef enum veml6040_triggers_e {
    VEML6040_TRIGGER_NONE = 0,     /*!< veml6040 no trigger */
    VEML6040_TRIGGER_ONE_TIME = 1  /*!< veml6040 trigger one time detect cycle */
} veml6040_triggers_t;

/**
 * @brief VEML6040 modes enumerator.
 */
typedef enum veml6040_modes_e {
    VEML6040_MODE_AUTO = 0,     /*!< veml6040 auto mode */
    VEML6040_MODE_MANUAL = 1    /*!< veml6040 manual force mode */
} veml6040_modes_t;

/**
 * @brief VEML6040 configuration register structure.
**/
typedef union __attribute__((packed)) veml6040_config_register_u {
    struct {
        bool                                shutdown_enabled:1;     /*!< shut-down when true                        (bit:0)     */
        veml6040_modes_t                    mode:1;                 /*!< mode, auto or manual                       (bit:1) */
        veml6040_triggers_t                 trigger:1;              /*!< trigger, none or one-time detect cycle     (bit:2)     */
        uint8_t                             reserved1:2;            /*!< reserved and set to 0                      (bit:3)   */
        veml6040_integration_times_t        integration_time:4;     /*!< time to measure                            (bit:6-4)   */
        uint8_t                             reserved2:1;            /*!< reserved and set to 0                      (bit:7)   */
        uint8_t                             reserved3:8;            /*!< reserved and set to 0 (high byte)          (bit:0-7)   */
    } bits;
    uint16_t reg;
} veml6040_config_register_t;

/**
 * @brief VEML6040 configuration structure.
 */
typedef struct veml6040_config_s {
    uint16_t                            i2c_address;            /*!< veml6040 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< veml6040 i2c device scl clock speed  */
    veml6040_integration_times_t        integration_time;       /*!< veml6040 integration time setting */
    bool                                shutdown_enabled;       /*!< veml6040 is shutdown when enabled */
    veml6040_modes_t                    mode;                   /*!< veml6040 mode */
    veml6040_triggers_t                 trigger_method;         /*!< veml6040 trigger method */
} veml6040_config_t;


/**
 * @brief VEML6040 opaque handle structure definition.
 */
typedef void* veml6040_handle_t;

/**
 * @brief Reads configuration register from VEML6040.
 *
 * @param[in] handle VEML6040 device handle.
 * @param[out] reg VEML6040 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_configuration_register(veml6040_handle_t handle, veml6040_config_register_t *const reg);

/**
 * @brief Writes configuration register to VEML6040.
 *
 * @param[in] handle VEML6040 device handle.
 * @param[in] reg VEML6040 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_set_configuration_register(veml6040_handle_t handle, const veml6040_config_register_t reg);

/**
 * @brief Initializes an VEML6040 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] veml6040_config VEML6040 device configuration.
 * @param[out] veml6040_handle VEML6040 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_init(i2c_master_bus_handle_t master_handle, const veml6040_config_t *veml6040_config, veml6040_handle_t *veml6040_handle);

/**
 * @brief Reads red illuminance channel from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param als VEML6040 red illuminance in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_red_als(veml6040_handle_t handle, float *const als);

/**
 * @brief Reads green illuminance channel from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param als VEML6040 green illuminance in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_green_als(veml6040_handle_t handle, float *const als);

/**
 * @brief Reads blue illuminance channel from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param als VEML6040 blue illuminance in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_blue_als(veml6040_handle_t handle, float *const als);

/**
 * @brief Reads white illuminance channel from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param als VEML6040 white illuminance in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_white_als(veml6040_handle_t handle, float *const als);

/**
 * @brief Reads red, green, blue, and white illuminance channels from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param red_als VEML6040 red illuminance in lux.
 * @param green_als VEML6040 green illuminance in lux.
 * @param blue_als VEML6040 blue illuminance in lux.
 * @param white_als VEML6040 white illuminance in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_als(veml6040_handle_t handle, float *const red_als, float *const green_als, float *const blue_als, float *const white_als);

/**
 * @brief Reads integration time from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param[out] integration_time VEML6040 integration time setting. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_integration_time(veml6040_handle_t handle, veml6040_integration_times_t *const integration_time);

/**
 * @brief Writes integration time to VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param[in] integration_time VEML6040 integration time setting. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_set_integration_time(veml6040_handle_t handle, const veml6040_integration_times_t integration_time);

/**
 * @brief Reads trigger method from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param[out] trigger_method VEML6040 trigger method seting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_trigger_method(veml6040_handle_t handle, veml6040_triggers_t *const trigger_method);

/**
 * @brief Write trigger method to VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param trigger_method VEML6040 trigger method seting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_set_trigger_method(veml6040_handle_t handle, const veml6040_triggers_t trigger_method);

/**
 * @brief Reads mode from VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param[out] mode VEML6040 mode seting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_get_mode(veml6040_handle_t handle, veml6040_modes_t *const mode);

/**
 * @brief Writes mode to VEML6040.
 * 
 * @param handle VEML6040 device handle.
 * @param[out] mode VEML6040 mode seting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_set_mode(veml6040_handle_t handle, const veml6040_modes_t mode);

/**
 * @brief Shuts down VEML6040 until woken.
 *
 * @param[in] handle VEML6040 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_disable(veml6040_handle_t handle);

/**
 * @brief Wakes up VEML6040 from shutdown.
 *
 * @param[in] handle VEML6040 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_enable(veml6040_handle_t handle);

/**
 * @brief Removes an VEML6040 device from master bus.
 *
 * @param[in] handle VEML6040 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_remove(veml6040_handle_t handle);

/**
 * @brief Removes an VEML6040 device from master I2C bus and delete the handle.
 * 
 * @param handle VEML6040 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml6040_delete(veml6040_handle_t handle);

/**
 * @brief Converts VEML6040 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* VEML6040 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* veml6040_get_fw_version(void);

/**
 * @brief Converts VEML6040 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t VEML6040 firmware version number.
 */
int32_t veml6040_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __VEML6040_H__
