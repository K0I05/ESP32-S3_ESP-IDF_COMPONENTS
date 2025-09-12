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
 * @file vl53l4cx.h
 * @defgroup drivers vl53l4cx
 * @{
 *
 * ESP-IDF driver for vl53l4cx ToF sensor
 * 
 * https://github.com/revk/ESP32-VL53L1X/blob/main/vl53l1x.c
 * 
 * https://github.com/RJRP44/VL53L8CX-Library/blob/master/examples/get_set_params/main/main.c
 * 
 * https://github.com/david-asher/VL53L1-ULD-ESP/blob/master/platform/vl53l1_platform.c
 * 
 * https://github.com/GrimbiXcode/VL53L0X-Register-Map
 * 
 * https://github.com/RJRP44/VL53L8CX-Library/blob/master/include/vl53l8cx_api.h
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __VL53L4CX_H__
#define __VL53L4CX_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "vl53l4cx_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VL53L4CX definitions
*/
#define I2C_VL53L4CX_DEV_CLK_SPD            UINT32_C(100000)    //!< vl53l4cx I2C default clock frequency (100KHz)

#define I2C_VL53L4CX_DEV_ADDR               UINT8_C(0x52)       //!< vl53l4cx I2C address


/*
 * VL53L4CX macro definitions
*/

/**
 * @brief VL53L4CX device configuration initialization default.
 */
#define I2C_VL53L4CX_CONFIG_DEFAULT {                                               \
            .i2c_address                = I2C_VL53L4CX_DEV_ADDR,                    \
            .i2c_clock_speed            = I2C_VL53L4CX_DEV_CLK_SPD, }

/*
 * VL53L4CX enumerator and structure declarations
*/



/**
 * @brief VL53L4CX device configuration structure definition.
 */
typedef struct vl53l4cx_config_s {
    uint16_t                            i2c_address;            /*!< veml7700 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< veml7700 i2c device scl clock speed  */
} vl53l4cx_config_t;


/**
 * @brief VL53L4CX opaque handle structure definition.
 */
typedef void* vl53l4cx_handle_t;




/**
 * @brief Initializes an VL53L4CX device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] vl53l4cx_config VL53L4CX device configuration.
 * @param[out] vl53l4cx_handle VL53L4CX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t vl53l4cx_init(i2c_master_bus_handle_t master_handle, const vl53l4cx_config_t *vl53l4cx_config, vl53l4cx_handle_t *vl53l4cx_handle);



/**
 * @brief Removes an VL53L4CX device from master bus.
 *
 * @param[in] handle VL53L4CX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t vl53l4cx_remove(vl53l4cx_handle_t handle);

/**
 * @brief Removes an VL53L4CX device from master I2C bus and delete the handle.
 * 
 * @param handle VL53L4CX device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t vl53l4cx_delete(vl53l4cx_handle_t handle);


/**
 * @brief Converts VL53L4CX firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* VL53L4CX firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* vl53l4cx_get_fw_version(void);

/**
 * @brief Converts VL53L4CX firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t VL53L4CX firmware version number.
 */
int32_t vl53l4cx_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __VL53L4CX_H__
