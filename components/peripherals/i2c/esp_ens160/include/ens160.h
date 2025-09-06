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
 * @file ens160.h
 * @defgroup drivers ens160
 * @{
 *
 * ESP-IDF driver for ens160 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __ENS160_H__
#define __ENS160_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "ens160_version.h"


#ifdef __cplusplus
extern "C" {
#endif

/*
 * ENS160 definitions
*/
#define I2C_ENS160_DEV_CLK_SPD              UINT32_C(100000)  //!< ens160 I2C default clock frequency (100KHz)

#define I2C_ENS160_DEV_ADDR_LO              UINT8_C(0x52)   //!< ens160 I2C address ADDR pin low
#define I2C_ENS160_DEV_ADDR_HI              UINT8_C(0x53)   //!< ens160 I2C address ADDR pin high


#define ENS160_TVOC_MIN                 UINT16_C(0)         /*!< ens160 tvoc minimum in ppb (section 5.1) */
#define ENS160_TVOC_MAX                 UINT16_C(65000)     /*!< ens160 tvoc maximum in ppb (section 5.1) */
#define ENS160_ECO2_MIN                 UINT16_C(400)       /*!< ens160 equivalent co2 minimum in ppm (table 5) */
#define ENS160_ECO2_MAX                 UINT16_C(65000)     /*!< ens160 equivalent co2 maximum in ppm (table 5) */
#define ENS160_AQI_MIN                  UINT16_C(1)         /*!< ens160 air quality index UBA minimum (table 6) */
#define ENS160_AQI_MAX                  UINT16_C(5)         /*!< ens160 air quality index UBA maximum (table 6) */


#define ENS160_ERROR_MSG_SIZE          (80)   //!< ens160 I2C error message size
#define ENS160_ERROR_MSG_TABLE_SIZE    (7)    //!< ens160 I2C error message table size

/**
 * Parameter Range
 * TVOC     0..65,000 ppb (Total Volatile Organic Compounds)
 * eCO2    400..65,00 ppm (Equivalent Carbon Dioxide)
 * AQI-UBA   1..5         (UBA Air Quality Index)
 */

/*
 * ENS160 macro definitions
*/


/**
 * @brief Macro that initializes `i2c_ens160_config_t` to default configuration settings.
 */
#define I2C_ENS160_CONFIG_DEFAULT {                                             \
        .i2c_address                = I2C_ENS160_DEV_ADDR_HI,                   \
        .i2c_clock_speed            = I2C_ENS160_DEV_CLK_SPD,                   \
        .irq_enabled                = false,                                    \
        .irq_data_enabled           = false,                                    \
        .irq_gpr_enabled            = false,                                    \
        .irq_pin_driver             = ENS160_INT_PIN_DRIVE_OPEN_DRAIN,          \
        .irq_pin_polarity           = ENS160_INT_PIN_POLARITY_ACTIVE_LO }

/*
 * ENS160 enumerator and structure declarations
*/

/**
 * @brief ENS160 air quality index of the uba enumerator.
 */
typedef enum ens160_aqi_uba_indexes_e {
    ENS160_AQI_UBA_INDEX_UNKNOWN    = 0, /*!< uba air quality index is unknown */
    ENS160_AQI_UBA_INDEX_1          = 1, /*!< uba air quality index of 1 is excellent */
    ENS160_AQI_UBA_INDEX_2          = 2, /*!< uba air quality index of 2 is good */
    ENS160_AQI_UBA_INDEX_3          = 3, /*!< uba air quality index of 3 is moderate */
    ENS160_AQI_UBA_INDEX_4          = 4, /*!< uba air quality index of 4 is poor */
    ENS160_AQI_UBA_INDEX_5          = 5  /*!< uba air quality index of 5 is unhealthy */
} ens160_aqi_uba_indexes_t;

/**
 * @brief ENS160 interrupt pin polarities enumerator.
 */
typedef enum ens160_interrupt_pin_polarities_e {
    ENS160_INT_PIN_POLARITY_ACTIVE_LO  = 0, /*!< ens160 interrupt pin polarity active low (default) */
    ENS160_INT_PIN_POLARITY_ACTIVE_HI  = 1  /*!< ens160 interrupt pin polarity active high  */
} ens160_interrupt_pin_polarities_t;

/**
 * @brief ENS160 interrupt pin drivers enumerator.
 */
typedef enum ens160_interrupt_pin_drivers_e {
    ENS160_INT_PIN_DRIVE_OPEN_DRAIN  = 0, /*!< ens160 interrupt pin drive open drain */
    ENS160_INT_PIN_DRIVE_PUSH_PULL   = 1, /*!< ens160 interrupt pin drive push/pull  */
} ens160_interrupt_pin_drivers_t;

/**
 * @brief ENS160 operating modes enumerator.
 */
typedef enum ens160_operating_modes_e {
    ENS160_OPMODE_DEEP_SLEEP    = 0x00, /*!< ens160 deep sleep mode (low-power standby) */
    ENS160_OPMODE_IDLE          = 0x01, /*!< ens160 idle mode (low-power) (default) */
    ENS160_OPMODE_STANDARD      = 0x02, /*!< ens160 standard gas sensing mode */
    ENS160_OPMODE_RESET         = 0xf0  /*!< ens160 reset mode */
} ens160_operating_modes_t;

/**
 * @brief ENS160 commands enumerator.
 */
typedef enum ens160_commands_e {
    ENS160_CMD_NORMAL         = 0x00, /*!< ens160 normal operation command (default) */
    ENS160_CMD_GET_FW_APPVER  = 0x0e, /*!< ens160 get firmware version command */
    ENS160_CMD_CLEAR_GPR      = 0xcc  /*!< ens160 clear general purpose read registers command */
} ens160_commands_t;

/**
 * @brief ENS160 validity flags enumerator.
 */
typedef enum ens160_validity_flags_e {
    ENS160_VALFLAG_NORMAL           = 0x00, /*!< ens160 normal operation validity flag */
    ENS160_VALFLAG_WARMUP           = 0x01, /*!< ens160 warm-up phase validity flag (first 3-minutes after power-on) */
    ENS160_VALFLAG_INITIAL_STARTUP  = 0x02, /*!< ens160 initial start-up phase validity flag (first full hour of operation after power-on, once in the sensor's lifetime) */
    ENS160_VALFLAG_INVALID_OUTPUT   = 0x03  /*!< ens160 invalid output validity flag */
} ens160_validity_flags_t;

/**
 * @brief ENS160 status register structure.
 */
typedef union __attribute__((packed)) ens160_status_register_u {
    struct STS_REG_BIT_TAG {
        bool                        new_gpr_data:1;   /*!< true indicates new data is available in `GPR_READ` registers (bit:0)   */
        bool                        new_data:1;       /*!< true indicates new data is available in `DATA_x` registers   (bit:1)   */
        ens160_validity_flags_t     state:2;          /*!< device status                                                (bit:2-3) */
        uint8_t                     reserved:2;       /*!< reserved and set 0                                           (bit:4-5) */
        bool                        error:1;          /*!< true indicates an error is detected                          (bit:6)   */
        bool                        mode:1;           /*!< true indicates an operating mode is running                  (bit:7)   */
    } bits;            /*!< represents the 8-bit status register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit status register as `uint8_t`.   */
} ens160_status_register_t;

/**
 * @brief ENS160 interrupt configuration register structure.
 */
typedef union __attribute__((packed)) ens160_interrupt_config_register_u {
    struct CFG_REG_BIT_TAG {
        bool                        irq_enabled:1;       /*!< true indicates interrupt pin is enabled                       (bit:0)   */
        bool                        irq_data_enabled:1;  /*!< true indicates interrupt pin is asserted when new data is available in `DATA_XXX` registers  (bit:1)   */
        uint8_t                     reserved1:1;         /*!< reserved and set to 0                                         (bit:2)   */
        bool                        irq_gpr_enabled:1;   /*!< true indicates interrupt pin is asserted when new data is available in general purpose registers (bit:3) */
        uint8_t                     reserved2:1;         /*!< reserved and set to 0                                         (bit:4)   */
        ens160_interrupt_pin_drivers_t irq_pin_driver:1; /*!< interrupt pin driver configuration                        (bit:5)   */
        ens160_interrupt_pin_polarities_t irq_pin_polarity:1; /*!< interrupt pin polarity configuration                 (bit:6)   */
        uint8_t                     reserved3:1;         /*!< reserved and set to 0                                         (bit:7)   */
    } bits;            /*!< represents the 8-bit interrupt configuration register parts in bits.   */
    uint8_t reg;       /*!< represents the 8-bit interrupt configuration register as `uint8_t`.   */
} ens160_interrupt_config_register_t;

/**
 * @brief ENS160 application version register structure.
 */
typedef union ens160_app_version_u {
    uint8_t major;      /*!< ens160 major firmware version */
    uint8_t minor;      /*!< ens160 minor firmware version */
    uint8_t release;    /*!< ens160 firmware version release */
    uint8_t bytes[3];   /*!< represents app version as a byte array. */
} ens160_app_version_t;

/**
 * @brief ENS160 calculated air quality index (aqi) data register structure.  See datasheet for AQI-UBA details.
 */
typedef union __attribute__((packed)) ens160_caqi_data_register_u {
    struct CAL_AQI_REG_BITS_TAG {
        uint8_t          aqi_uba:3;     /*!< air quality index per uba[1..5] (default: 0x01)  (bit:0-2)  */
        uint8_t          reserved:5;    /*!< reserved and set to 0                            (bit:3-7)  */
    } bits;        /*!< represents the 8-bit calculated air quality index data register parts in bits.  */
    uint8_t value; /*!< represents the 8-bit calculated air quality index data register as `uint8_t`.   */
} ens160_caqi_data_register_t;

/**
 * @brief ENS160 air quality data structure.
 */
typedef struct ens160_air_quality_data_s {
    ens160_aqi_uba_indexes_t        uba_aqi;    /*!< ENS160 air quality index according to the UBA */
    uint16_t                        tvoc;       /*!< ENS160 calculated tvoc in ppb */
    uint16_t                        etoh;       /*!< ENS160 calculated etoh in ppb */
    uint16_t                        eco2;       /*!< ENS160 calculated equivalent co2 concentration in ppm */
} ens160_air_quality_data_t;

/**
 * @brief ENS160 air quality raw data structure.
 */
typedef struct ens160_air_quality_raw_data_s {
    uint32_t                        hp0_ri;     /*!< */
    uint32_t                        hp1_ri;     /*!< */
    uint32_t                        hp2_ri;     /*!< */
    uint32_t                        hp3_ri;     /*!< */
    uint32_t                        hp0_bl;     /*!< */
    uint32_t                        hp1_bl;     /*!< */
    uint32_t                        hp2_bl;     /*!< */
    uint32_t                        hp3_bl;     /*!< */
} ens160_air_quality_raw_data_t;

/**
 * @brief ENS160 air quality index of the UBA row definition structure.
 */
typedef struct ens160_aqi_uba_row_s {
    ens160_aqi_uba_indexes_t        index;            /*!< AQI-UBA index rating */
    const char*                     rating;           /*!< AQI-UBA rating category */
    const char*                     hygienic_rating;  /*!< AQI-UBA hygienic rating guidance */
    const char*                     recommendation;   /*!< AQI-UBA recommendation */
    const char*                     exposure_limit;   /*!< AQI-UBA exposure limit guidance */
} ens160_aqi_uba_row_t;


/**
 * @brief ENS160 configuration structure.
 */
typedef struct ens160_config_s {
    uint16_t                            i2c_address;            /*!< ens160 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< ens160 i2c device scl clock speed in hz */
    bool                                irq_enabled;            /*!< true indicates interrupt pin is enabled  */
    bool                                irq_data_enabled;       /*!< true indicates interrupt pin is asserted when new data is available in `DATA_XXX` registers   */
    bool                                irq_gpr_enabled;        /*!< true indicates interrupt pin is asserted when new data is available in general purpose registers  */
    ens160_interrupt_pin_drivers_t      irq_pin_driver;         /*!< interrupt pin driver configuration   */
    ens160_interrupt_pin_polarities_t   irq_pin_polarity;       /*!< interrupt pin polarity configuration  */
} ens160_config_t;



/**
 * @brief ENS160 opaque handle structure definition.
 */
typedef void* ens160_handle_t;


/**
 * @brief Reads interrupt configuration register from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] reg ENS160 interrupt configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_interrupt_config_register(ens160_handle_t handle, ens160_interrupt_config_register_t *const reg);

/**
 * @brief Writes interrupt configuration register to ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[in] reg ENS160 interrupt configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_set_interrupt_config_register(ens160_handle_t handle, const ens160_interrupt_config_register_t reg);

/**
 * @brief Reads status register from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] reg ENS160 status configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_status_register(ens160_handle_t handle, ens160_status_register_t *const reg);

/**
 * @brief Resets command to operate normal and clears general purpose registers on ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_clear_command_register(ens160_handle_t handle);

/**
 * @brief Reads temperature and humidity compensation registers from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] temperature temperature compensation in degree Celsius.
 * @param[out] humidity humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_compensation_registers(ens160_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Writes temperature and humidity compensation registers to ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[in] temperature temperature compensation in degree Celsius.
 * @param[in] humidity humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_set_compensation_registers(ens160_handle_t handle, const float temperature, const float humidity);

/**
 * @brief Reads part identifier register from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] reg Part id register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_part_id_register(ens160_handle_t handle, uint16_t *const reg);

/**
 * @brief Initializes an ENS160 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] ens160_config ENS160 device configuration.
 * @param[out] ens160_handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_init(i2c_master_bus_handle_t master_handle, const ens160_config_t *ens160_config, ens160_handle_t *ens160_handle);

/**
 * @brief Reads calculated air quality measurements from ENS160.
 * 
 * @param[in] handle ENS160 device handle.
 * @param[out] data ENS160 air quality data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_measurement(ens160_handle_t handle, ens160_air_quality_data_t *const data);

/**
 * @brief Reads raw air quality measurements from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] data ENS160 air quality raw data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_raw_measurement(ens160_handle_t handle, ens160_air_quality_raw_data_t *const data);

/**
 * @brief Reads data ready status from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] ready ENS160 data ready status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_data_status(ens160_handle_t handle, bool *const ready);

/**
 * @brief Reads general purpose registers data ready status from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] ready ENS160 general purpose registers data ready status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_gpr_data_status(ens160_handle_t handle, bool *const ready);

/**
 * @brief Read validity flag status, device status and signal rating, from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] state ENS160 validity flag status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_validity_status(ens160_handle_t handle, ens160_validity_flags_t *const state);

/**
 * @brief Read error status from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] error ENS160 error status, true indicates an error.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_error_status(ens160_handle_t handle, bool *const error);

/**
 * @brief Read operating mode status from ENS160.
 *
 * @param[in] handle ENS160 device handle.
 * @param[out] mode ENS160 operating mode status, true indicates an operating mode is running.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_mode_status(ens160_handle_t handle, bool *const mode);

/**
 * @brief Reads data ready, general purpose registers data ready, validity flag, and error status from ENS160.
 * 
 * @param handle ENS160 device handle.
 * @param[out] data_ready ENS160 data ready status.
 * @param[out] gpr_data_ready ENS160 general purpose registers data ready status.
 * @param[out] state ENS160 validity flag status.
 * @param[out] error ENS160 error status.
 * @param[out] mode ENS160 operating mode status.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_status(ens160_handle_t handle, bool *const data_ready, bool *const gpr_data_ready, ens160_validity_flags_t *const state, bool *const error, bool *const mode);

/**
 * @brief Reads temperature and humidity compensation factors from ENS160.
 * 
 * @param handle ENS160 device handle.
 * @param temperature ENS160 temperature compensation in degrees celsius.
 * @param humidity ENS160 humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_get_compensation_factors(ens160_handle_t handle, float *const temperature, float *const humidity);

/**
 * @brief Writes temperature and humidity compensation factors to ENS160.
 * 
 * @param handle ENS160 device handle.
 * @param temperature ENS160 temperature compensation in degrees celsius.
 * @param humidity ENS160 humidity compensation in percentage.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_set_compensation_factors(ens160_handle_t handle, const float temperature, const float humidity);

/**
 * @brief Enables standard operating mode to ENS160 to operate as a gas sensor and respond to commands.
 *
 * @param[in] handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_enable_standard_mode(ens160_handle_t handle);

/**
 * @brief Enables idle operating mode to ENS160 to respond to commands.
 *
 * @param[in] handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_enable_idle_mode(ens160_handle_t handle);

/**
 * @brief Enables deep sleep operating mode to ENS160.
 * 
 * @note The ENS160 will not respond to commands unless it is placed in idle or operational mode.
 *
 * @param[in] handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_enable_deep_sleep_mode(ens160_handle_t handle);

/**
 * @brief Issues soft-reset and initializes ENS160 to idle mode.
 * 
 * @note ENS160 I2C device handle registers are initialized.
 *
 * @param[in] handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_reset(ens160_handle_t handle);

/**
 * @brief Removes an ENS160 device from master I2C bus.
 *
 * @param[in] handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_remove(ens160_handle_t handle);

/**
 * @brief Removes an ENS160 device from master bus and frees handle.
 * 
 * @param handle ENS160 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ens160_delete(ens160_handle_t handle);

/**
 * @brief Decodes ENS160 air quality index to a uba definition row.
 * 
 * @param[in] index ENS160 air quality index of the uba.
 * @return i2c_ens160_aqi_uba_row_t air quality index of the uba definition row on success.
 */
ens160_aqi_uba_row_t ens160_aqi_index_to_definition(const ens160_aqi_uba_indexes_t index);

/**
 * @brief Converts ENS160 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* ENS160 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ens160_get_fw_version(void);

/**
 * @brief Converts ENS160 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t ENS160 firmware version number.
 */
int32_t ens160_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __ENS160_H__
