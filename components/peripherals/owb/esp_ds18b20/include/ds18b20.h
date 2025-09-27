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
 * @file ds18b20.h
 * @defgroup drivers ds18b20
 * @{
 *
 * ESP-IDF driver for ds18b20 temperature sensor
 * 
 * Source references:
 * https://github.com/espressif/esp-bsp/blob/master/components/ds18b20/src/ds18b20.c
 * https://github.com/DavidAntliff/esp32-ds18b20/blob/99eb5dd55536fd79b4bd5790f1430c218c51410f/ds18b20.c
 * https://github.com/adafruit/MAX31850_DallasTemp/blob/master/DallasTemperature.cpp
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <onewire_bus.h>
#include <onewire_device.h>
#include "ds18b20_version.h"


#ifdef __cplusplus
extern "C"
{
#endif

/*
 * DS18B20 definitions
 */


/*
 * DS18B20 macro definitions
 */

#define DS18B20_CONFIG_DEFAULT {                \
    .resolution      = DS18B20_RESOLUTION_10BIT,    \
    .trigger_enabled = false }


/**
 * @brief DS18B20 supported resolutions enumerator.
 */
typedef enum ds18b20_resolutions_e {
    DS18B20_RESOLUTION_9BIT  = 0b00,  /*!<  9bit, increment of 0.5°C, needs ~93.75ms convert time (Tconv/8)  */
    DS18B20_RESOLUTION_10BIT = 0b01,  /*!< 10bit, increment of 0.25°C, needs ~187.5ms convert time (Tconv/4) */
    DS18B20_RESOLUTION_11BIT = 0b10,  /*!< 11bit, increment of 0.125°C, needs ~375ms convert time (Tconv/2)  */
    DS18B20_RESOLUTION_12BIT = 0b11,  /*!< 12bit, increment of 0.0625°C, needs ~750ms convert time (Tconv)   */
} ds18b20_resolutions_t;

/**
 * @brief DS18B20 configuration register structure.  See datasheet for details, Figure 10 and Table 2.
 */
typedef union __attribute__((packed)) ds18b20_configuration_register_u {
    struct DS18B20_CFG_REG_BIT_TAG {
        uint8_t                     reserved1:5;  /*!< reserved and set 1      (bit:0-4) */
        ds18b20_resolutions_t       resolution:2; /*!< temperature resolution  (bit:5-6) */
        uint8_t                     reserved2:1;  /*!< reserved and set 0      (bit:7)   */
    } bits;            /*!< represents the 8-bit configuration register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit configuration register as `uint8_t`.    */
} ds18b20_configuration_register_t;

/**
 * @brief DS18B20 scratchpad structure.  Power-up state depends on value(s) stored in EEPROM.
 */
typedef struct __attribute__((packed)) ds18b20_scratchpad_s {
    uint8_t temp_lsb;       /*!< lsb of temperature */
    uint8_t temp_msb;       /*!< msb of temperature */
    uint8_t trigger_high;   /*!< high trigger alarm threshold */
    uint8_t trigger_low;    /*!< low trigger alarm threshold */
    uint8_t configuration;  /*!< configuration register */
    uint8_t reserved1;      /*!< reserved */
    uint8_t reserved2;      /*!< reserved */
    uint8_t reserved3;      /*!< reserved */
    uint8_t crc;            /*!< crc value of scratchpad data */
} ds18b20_scratchpad_t;

/**
 * @brief DS18B20 device configuration structure definition.
 */
typedef struct ds18b20_config_s {
    ds18b20_resolutions_t     resolution;       /*!< ds18b20 temperature resolution */
    bool                      trigger_enabled;  /*!< ds18b20 alarm trigger thresholds are configured when true */
    int8_t                    trigger_high;     /*!< ds18b20 high alarm trigger threshold */
    int8_t                    trigger_low;      /*!< ds18b20 low alarm trigger threshold */
} ds18b20_config_t;

/**
 * @brief DS18B20 opaque handle structure definition.
 */
typedef void* ds18b20_handle_t;


/**
 * @brief Checks 1-wire device address against the ROM ID (i.e. family code of DS18B20 is 0x28) to determine if the device is a DS18B20.
 * 
 * @param address 1-wire device address.
 * @return bool 1-wire device is a DS18B20 when true;
 */
bool ods18b20_validate_address(const onewire_device_address_t address);

/**
 * @brief Detects up to 10 DS18B20 devices on the 1-wire bus.
 * 
 * @param[in] owb_handle 1-wire bus handle.
 * @param[out] devices Array of DS18B20 devices detected on the 1-wire bus.
 * @param[in] device_size Size of DS18B20 devices array.  The maximum number of detectable DS18B20 devices is 10.
 * @param[out] device_count Number of DS18B20 devices detected.  The maximum number of detectable DS18B20 devices is 10.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_detect(onewire_bus_handle_t owb_handle, onewire_device_t *const devices, const uint8_t device_size, uint8_t *const device_count);

/**
 * @brief Checks if the DS18B20 is connected to the 1-wire bus.
 * 
 * @param handle DS18B20 device handle.
 * @param connected DS18B20 is connected when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_connected(ds18b20_handle_t handle, bool *const connected);

/**
 * @brief Initializes an DS18B20 device onto the 1-wire master bus.
 * 
 * @note Parasitic-powered devices are not supported at this time.
 *
 * @param[in] device 1-wire device.
 * @param[in] ds18b20_config DS18B20 device configuration.
 * @param[out] ds18b20_handle DS18B20 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_init(onewire_device_t *device, const ds18b20_config_t *ds18b20_config, ds18b20_handle_t *ds18b20_handle);

/**
 * @brief Triggers temperature conversion and reads temperature from DS18B20.
 * 
 * @param handle DS18B20 device handle.
 * @param temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_get_temperature(ds18b20_handle_t handle, float *const temperature);

/**
 * @brief Reads temperature from DS18B20.
 * 
 * @note The trigger temperature conversion function must be called before reading the temperature.
 *
 * @param handle DS18B20 device handle.
 * @param temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_get_measurement(ds18b20_handle_t handle, float *const temperature);

/**
 * @brief Triggers DS18B20 temperature conversion.  This function must be called before reading the temperature from DS18B20.
 * 
 * @param handle DS18B20 device handle. 
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_trigger_temperature_conversion(ds18b20_handle_t handle);

/**
 * @brief Reads temperature conversion resolution from DS18B20.
 * 
 * @param handle DS18B20 device handle. 
 * @param resolution DS18B20 temperature conversion resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_get_resolution(ds18b20_handle_t handle, ds18b20_resolutions_t *const resolution);

/**
 * @brief Writes temperature conversion resolution to DS18B20.
 * 
 * @param handle DS18B20 device handle. 
 * @param resolution DS18B20 temperature conversion resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_set_resolution(ds18b20_handle_t handle, const ds18b20_resolutions_t resolution);

/**
 * @brief Reads high and low temperature alarm thresholds from DS18B20.
 * 
 * @param handle DS18B20 device handle. 
 * @param high DS18B20 high temperature alarm threshold setting (-55 to 125 degrees Celsius).
 * @param low DS18B20 low temperature alarm threshold setting (-55 to 125 degrees Celsius).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_get_alarm_thresholds(ds18b20_handle_t handle, int8_t *const high, int8_t *const low);

/**
 * @brief Writes high and low temperature alarm thresholds to DS18B20.
 * 
 * @param handle DS18B20 device handle. 
 * @param high DS18B20 high temperature alarm threshold setting (-55 to 125 degrees Celsius).
 * @param low DS18B20 low temperature alarm threshold setting (-55 to 125 degrees Celsius).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_set_alarm_thresholds(ds18b20_handle_t handle, const int8_t high, const int8_t low);

/**
 * @brief Check if the DS18B20 temperature alarm triggered.
 * 
 * @param handle DS18B20 device handle.
 * @param alarm DS18B20 alarm triggered when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_alarm_triggered(ds18b20_handle_t handle, bool *const alarm);

/**
 * @brief Reads power supply mode setting from DS18B20.  Parasitic-powered devices are powered by the data pin (DQ)
 * and will pull the bus low during read time slot.  Parasitic-powered devices are not supported at this time.
 * 
 * @param handle DS18B20 device handle. 
 * @param parasitic DS18B20 power supply mode setting is parasitic when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_get_power_supply_mode(ds18b20_handle_t handle, bool *const parasitic);

/**
 * @brief Removes an DS18B20 device from master bus and frees handle.
 * 
 * @param handle DS18B20 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ds18b20_delete(ds18b20_handle_t handle);

/**
 * @brief Converts DS18B20 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* DS18B20 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ds18b20_get_fw_version(void);

/**
 * @brief Converts DS18B20 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t DS18B20 firmware version number.
 */
int32_t ds18b20_get_fw_version_number(void);




#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __DS18B20_H__
