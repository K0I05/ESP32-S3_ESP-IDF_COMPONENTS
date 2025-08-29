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
 * @file tbi2cxxx.h
 * @defgroup drivers tbi2cxxx
 * @{
 *
 * ESP-IDF driver for tbi2cxxx ir temperature sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TBI2CXXX_H__
#define __TBI2CXXX_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "tbi2cxxx_version.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * TBI2CXXX definitions
 */
#define I2C_TBI2CXXX_DEV_CLK_SPD           UINT32_C(100000) //!< tbi2cxxx I2C default clock frequency (100KHz)

#define I2C_TBI2CXXX_DEV_ADDR              UINT8_C(0x3a)   //!< tbi2cxxx I2C address

/*
 * TBI2CXXX macro definitions
 */
#define I2C_AHT2X_CONFIG_DEFAULT {                  \
    .i2c_address        = I2C_TBI2CXXX_DEV_ADDR,    \
    .i2c_clock_speed    = I2C_TBI2CXXX_DEV_CLK_SPD, \
    .tb_type            = TB_I2C_H04 }


/*
* TBI2CXXX enumerator and structure declarations
*/

/**
 * @brief TBI2CXXX types enumerators.
 */
typedef enum tbi2cxxx_types_e {
    TBI2CH04,   /*!< tb(p) series non-contact infrared temperature sensor with 3.814° FOV and heatsink(O) */
    TBI2CH08,   /*!< tb(p) series non-contact infrared temperature sensor with 7.16° FOV and heatsink(O) */
    TBI2CH70,   /*!< tb(p) series non-contact infrared temperature sensor with 70° FOV and heatsink(O) */
    TBI2CS70    /*!< tb(p) series non-contact infrared temperature sensor with 70° FOV and heatsink(X) */
} tbi2cxxx_types_t;


/**
 * @brief TBI2CXXX configuration structure.
 */
typedef struct tbi2cxxx_config_s {
    uint16_t            i2c_address;    /*!< tbi2cxxx i2c device address */
    uint32_t            i2c_clock_speed;/*!< tbi2cxxx i2c device scl clock speed in hz */
    tbi2cxxx_types_t    tb_type;
} tbi2cxxx_config_t;

/**
 * @brief TBI2CXXX context structure.
 */
struct tbi2cxxx_context_t {
    tbi2cxxx_config_t       dev_config;
    i2c_master_dev_handle_t i2c_handle; /*!< I2C device handle */
};

/**
 * @brief TBI2CXXX context structure definition.
 */
typedef struct tbi2cxxx_context_t tbi2cxxx_context_t;

/**
 * @brief TBI2CXXX handle structure definition.
 */
typedef struct tbi2cxxx_context_t *tbi2cxxx_handle_t;



/**
 * @brief Initializes an TBI2CXXX device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] tbi2cxxx_config TBI2CXXX device configuration.
 * @param[out] tbi2cxxx_handle TBI2CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_init(i2c_master_bus_handle_t master_handle, const tbi2cxxx_config_t *tbi2cxxx_config, tbi2cxxx_handle_t *tbi2cxxx_handle);

/**
 * @brief Reads ambient and object temperatures from TBI2CXXX.
 * 
 * @param handle TBI2CXXX device handle.
 * @param ambient_temperature Ambient temperature in degrees celsius.
 * @param object_temperature Object temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_get_temperatures(tbi2cxxx_handle_t handle, float *const ambient_temperature, float *const object_temperature);

/**
 * @brief Reads ambient temperature from TBI2CXXX.
 * 
 * @param handle TBI2CXXX device handle. 
 * @param temperature Ambient temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_get_ambient_temperature(tbi2cxxx_handle_t handle, float *const temperature);

/**
 * @brief Reads object temperature from TBI2CXXX.
 * 
 * @param handle TBI2CXXX device handle. 
 * @param temperature Object temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_get_object_temperature(tbi2cxxx_handle_t handle, float *const temperature);

/**
 * @brief Reads emissivity coefficient setting from TBI2CXXX.
 * 
 * @note Factory default emissivity coefficient setting is 0.97.
 * 
 * @param handle TBI2CXXX device handle.
 * @param coefficient TBI2CXXX emissivity coefficient setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_get_emissivity(tbi2cxxx_handle_t handle, float *const coefficient);

/**
 * @brief Writes emissivity coefficient setting to TBI2CXXX.
 * 
 * @note The manufacturer recommends restarting the sensor by cycling power off and on.
 * 
 * @param handle TBI2CXXX device handle.
 * @param coefficient TBI2CXXX emissivity coefficient setting, acceptable range is 0.1 to 1.0.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_set_emissivity(tbi2cxxx_handle_t handle, const float coefficient);

/**
 * @brief Removes an TBI2CXXX device from master bus.
 *
 * @param[in] handle TBI2CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_remove(tbi2cxxx_handle_t handle);

/**
 * @brief Removes an TBI2CXXX device from master bus and frees handle.
 * 
 * @param handle TBI2CXXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tbi2cxxx_delete(tbi2cxxx_handle_t handle);

/**
 * @brief Converts TBI2CXXX firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* TBI2CXXX firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* tbi2cxxx_get_fw_version(void);

/**
 * @brief Converts TBI2CXXX firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t TBI2CXXX firmware version number.
 */
int32_t tbi2cxxx_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __TBI2CXXX_H__
