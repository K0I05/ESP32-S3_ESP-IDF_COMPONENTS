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
 * @file hdc1080.h
 * @defgroup drivers hdc1080
 * @{
 *
 * ESP-IDF driver for hdc1080 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __HDC1080_H__
#define __HDC1080_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "hdc1080_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * definitions
*/
#define I2C_HDC1080_DEV_CLK_SPD             UINT32_C(100000)   //!< hdc1080 I2C default clock frequency (100KHz)

#define I2C_HDC1080_DEV_ADDR_0              UINT8_C(0x40) //!< hdc1080 I2C address when ADR1 = 0, ADR0 = 0
#define I2C_HDC1080_DEV_ADDR_1              UINT8_C(0x41) //!< hdc1080 I2C address when ADR1 = 0, ADR0 = 1
#define I2C_HDC1080_DEV_ADDR_2              UINT8_C(0x42) //!< hdc1080 I2C address when ADR1 = 1, ADR0 = 0
#define I2C_HDC1080_DEV_ADDR_3              UINT8_C(0x43) //!< hdc1080 I2C address when ADR1 = 1, ADR0 = 1

/*
 * macro definitions
*/

/**
 * @brief Macro that initializes `i2c_hdc1080_config_t` to default configuration settings.
 */
#define I2C_HDC1080_CONFIG_DEFAULT {                                            \
        .i2c_address                = I2C_HDC1080_DEV_ADDR_0,                   \
        .i2c_clock_speed            = I2C_HDC1080_DEV_CLK_SPD,                  \
        .temperature_resolution     = HDC1080_TEMPERATURE_RESOLUTION_14BIT,     \
        .humidity_resolution        = HDC1080_HUMIDITY_RESOLUTION_14BIT,    }


/*
 * enumerator and structure declarations
*/


/**
 * @brief HDC1080 acquisition modes enumerator definition.
 */
typedef enum hdc1080_acquisition_modes_e {
    HDC1080_ACQUISITION_SINGLE     = 0, /*!< acquisition in single mode for temperature or humidity */
    HDC1080_ACQUISITION_SEQUENCED  = 1  /*!< acquisition in sequenced mode for both temperature and humidity */
} hdc1080_acquisition_modes_t;

/**
 * @brief HDC1080 battery states enumerator definition.
 */
typedef enum hdc1080_battery_states_e {
    HDC1080_BATT_VOLT_OVER_2_8V   = 0, /*!< battery voltage is over 2.8 volts  */
    HDC1080_BATT_VOLT_UNDER_2_8V  = 1  /*!< battery voltage is under 2.8 volts */
} hdc1080_battery_states_t;

/**
 * @brief HDC1080 temperature measurement resolutions enumerator definition.
 */
typedef enum hdc1080_temperature_resolutions_e {
    HDC1080_TEMPERATURE_RESOLUTION_14BIT = 0, /*!< temperature measurement 14-bit resolution */
    HDC1080_TEMPERATURE_RESOLUTION_11BIT = 1  /*!< temperature measurement 11-bit resolution */
} hdc1080_temperature_resolutions_t;

/**
 * @brief HDC1080 humidity measurement resolutions enumerator definition.
 */
typedef enum hdc1080_humidity_resolutions_e {
    HDC1080_HUMIDITY_RESOLUTION_14BIT = (0b00), /*!< humidity measurement 14-bit resolution */
    HDC1080_HUMIDITY_RESOLUTION_11BIT = (0b01), /*!< humidity measurement 11-bit resolution */
    HDC1080_HUMIDITY_RESOLUTION_8BIT  = (0b10)  /*!< humidity measurement 8-bit resolution  */
} hdc1080_humidity_resolutions_t;

/**
 * @brief HDC1080 device configuration register structure definition.
 */
typedef union __attribute__((packed)) hdc1080_configuration_register_u {
    struct REG_CFG_BITS_TAG {
        uint8_t                           reserved1:8;               /*!< reserved and set to 0              (bit:0-7) */
        hdc1080_humidity_resolutions_t    humidity_resolution:2;     /*!< humidity measurement resolution    (bit:8-9) */
        hdc1080_temperature_resolutions_t temperature_resolution:1;  /*!< temperature measurement resolution (bit:10) */
        hdc1080_battery_states_t          battery_state:1;           /*!< battery status                     (bit:11) */
        hdc1080_acquisition_modes_t       acquisition_mode:1;        /*!< acquisition mode                   (bit:12) */
        bool                              heater_enabled:1;          /*!< heater enabled when true           (bit:13) */
        uint8_t                           reserved2:1;               /*!< reserved and set to 0              (bit:14) */
        bool                              reset_enabled:1;           /*!< software reset when true           (bit:15) */
    } bits;          /*!< represents the 16-bit configuration register parts in bits. */
    uint16_t reg;   /*!< represents the 16-bit configuration register as `uint16_t` */
} hdc1080_configuration_register_t;

/**
 * @brief HDC1080 temperature or humidity measurement register structure definition.
 */
typedef union __attribute__((packed)) hdc1080_measurement_register_u {
    struct REG_MEAS_BITS_TAG {
        uint8_t        reserved:2;  /*!< reserved and set to 0        (bit:0-1) */
        uint16_t       value:14;    /*!< measurement value            (bit:2-14) */
    } bits;         /*!< represents the 16-bit measurement register parts in bits. */
    uint16_t reg;   /*!< represents the 16-bit measurement register as `uint16_t` */
} hdc1080_measurement_register_t;

/**
 * @brief HDC1080 serial number register structure definition.
 */
typedef union __attribute__((packed)) hdc1080_serial_number_register_u {
    struct REG_SN_BITS_TAG {
        uint8_t        reserved:7;      /*!< reserved and set to 0        (bit:0-6)   */
        uint16_t       serial_id_0:9;   /*!< serial id 0                  (bit:7-15)  */
        uint16_t       serial_id_1:16;  /*!< serial id 1                  (bit:16-31) */
        uint16_t       serial_id_2:16;  /*!< serial id 2                  (bit:32-47) */
    } bits;          /*!< represents the 64-bit measurement register parts in bits. */
    uint64_t reg;   /*!< represents the 64-bit measurement register as `uint64_t` */
} hdc1080_serial_number_register_t;

/**
 * @brief HDC1080 configuration structure definition.
 */
typedef struct hdc1080_config_s {
    uint16_t                            i2c_address;            /*!< hdc1080 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< hdc1080 i2c device scl clock speed  */
    hdc1080_temperature_resolutions_t   temperature_resolution; /*!< hdc1080 device temperature resolution */
    hdc1080_humidity_resolutions_t      humidity_resolution;    /*!< hdc1080 device humidity resolution */
} hdc1080_config_t;

/**
 * @brief HDC1080 context structure.
 */
struct hdc1080_context_t {
    hdc1080_config_t                    dev_config;             /*!< hdc1080 device configuration */ 
    i2c_master_dev_handle_t             i2c_handle;             /*!< hdc1080 i2c device handle */
    uint64_t                            serial_number;          /*!< hdc1080 device serial number */
    uint16_t                            manufacturer_id;        /*!< hdc1080 device manufacturer identifier */
    uint16_t                            device_id;              /*!< hdc1080 device device identifier */
};

/**
 * @brief HDC1080 context structure definition.
 */
typedef struct hdc1080_context_t hdc1080_context_t;

/**
 * @brief HDC1080 handle structure definition.
 */
typedef struct hdc1080_context_t* hdc1080_handle_t;


/**
 * @brief Reads unique serial number register from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 serial number register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_serial_number_register(hdc1080_handle_t handle, uint64_t *const reg);

/**
 * @brief Reads manufacturer identifier register from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 manufacturer identifier register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_manufacturer_id_register(hdc1080_handle_t handle, uint16_t *const reg);

/**
 * @brief Reads device identifier register from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 device identifier register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_device_id_register(hdc1080_handle_t handle, uint16_t *const reg);

/**
 * @brief Reads configuration register from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] reg HDC1080 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_configuration_register(hdc1080_handle_t handle, hdc1080_configuration_register_t *const reg);

/**
 * @brief Writes configuration register to HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[in] reg HDC1080 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_set_configuration_register(hdc1080_handle_t handle, const hdc1080_configuration_register_t reg);

/**
 * @brief Initializes an HDC1080 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] hdc1080_config HDC1080 device configuration.
 * @param[out] hdc1080_handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_init(i2c_master_bus_handle_t master_handle, const hdc1080_config_t *hdc1080_config, hdc1080_handle_t *hdc1080_handle);

/**
 * @brief Reads temperature and relative humidity from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] temperature Temperature measurement in degrees Celsius.
 * @param[out] humidity Relative humidity measurement in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_measurement(hdc1080_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Reads temperature, relative humidity, and dew-point from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] temperature Temperature measurement in degrees Celsius.
 * @param[out] humidity Relative humidity measurement in percentage.
 * @param[out] dewpoint Calculated dewpoint in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_measurements(hdc1080_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint);

/**
 * @brief Enables HDC1080 heater.
 * 
 * @param[in] handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_enable_heater(hdc1080_handle_t handle);

/**
 * @brief Disables HDC1080 heater.
 * 
 * @param[in] handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_disable_heater(hdc1080_handle_t handle);

/**
 * @brief Reads temperature measurement resolution from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] resolution HDC1080 temperature measurement resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_temperature_resolution(hdc1080_handle_t handle, hdc1080_temperature_resolutions_t *const resolution);

/**
 * @brief Writes temperature measurement resolution to HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[in] resolution HDC1080 temperature measurement resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_set_temperature_resolution(hdc1080_handle_t handle, const hdc1080_temperature_resolutions_t resolution);

/**
 * @brief Reads relative humidity measurement resolution from HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[out] resolution HDC1080 relative humidity measurement resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_get_humidity_resolution(hdc1080_handle_t handle, hdc1080_humidity_resolutions_t *const resolution);

/**
 * @brief Writes relative humidity measurement resolution to HDC1080.
 * 
 * @param[in] handle HDC1080 device handle.
 * @param[in] resolution HDC1080 relative humidity measurement resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_set_humidity_resolution(hdc1080_handle_t handle, const hdc1080_humidity_resolutions_t resolution);

/**
 * @brief Issues soft-reset to HDC1080.
 *
 * @param[in] handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_reset(hdc1080_handle_t handle);

/**
 * @brief Removes an HDC1080 device from master I2C bus.
 *
 * @param[in] handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_remove(hdc1080_handle_t handle);

/**
 * @brief Removes an HDC1080 device from master bus and frees handle.
 * 
 * @param[in] handle HDC1080 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t hdc1080_delete(hdc1080_handle_t handle);

/**
 * @brief Converts HDC1080 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* HDC1080 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* hdc1080_get_fw_version(void);

/**
 * @brief Converts HDC1080 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t HDC1080 firmware version number.
 */
int32_t hdc1080_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __HDC1080_H__
