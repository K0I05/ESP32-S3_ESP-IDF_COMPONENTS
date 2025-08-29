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
 * @file veml7700.h
 * @defgroup drivers veml7700
 * @{
 *
 * ESP-IDF driver for veml7700 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __VEML7700_H__
#define __VEML7700_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "veml7700_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VEML7700 definitions
*/
#define I2C_VEML7700_DEV_CLK_SPD            UINT32_C(100000)    //!< veml7700 I2C default clock frequency (100KHz)

#define I2C_VEML7700_DEV_ADDR               UINT8_C(0x10)       //!< veml7700 I2C address


/*
 * VEML7700 macro definitions
*/

/**
 * @brief VEML7700 device configuration initialization default.
 */
#define I2C_VEML7700_CONFIG_DEFAULT {                                               \
            .i2c_address                = I2C_VEML7700_DEV_ADDR,                    \
            .i2c_clock_speed            = I2C_VEML7700_DEV_CLK_SPD,                 \
            .gain                       = VEML7700_GAIN_DIV_4,                      \
            .integration_time           = VEML7700_INTEGRATION_TIME_400MS,          \
            .persistence_protect        = VEML7700_PERSISTENCE_PROTECTION_4,        \
            .irq_enabled                = true,                                     \
            .power_disabled             = false,                                    \
            .power_saving_enabled       = false,                                    \
            .power_saving_mode          = VEML7700_POWER_SAVING_MODE_1 }

/*
 * VEML7700 enumerator and structure declarations
*/

/**
 * @brief VEML7700 gains enumerator.
 */
typedef enum veml7700_gains_e {
    VEML7700_GAIN_1     = (0b00),   /*!< ALS gain x 1 */
    VEML7700_GAIN_2     = (0b01),   /*!< ALS gain x 2 */
    VEML7700_GAIN_DIV_8 = (0b10),   /*!< ALS gain x (1/8) */
    VEML7700_GAIN_DIV_4 = (0b11),   /*!< ALS gain x (1/4) */
} veml7700_gains_t;

/**
 * @brief VEML7700 integration times enumerator.
 */
typedef enum veml7700_integration_times_e {
    VEML7700_INTEGRATION_TIME_25MS  = (0b1100),
    VEML7700_INTEGRATION_TIME_50MS  = (0b1000),
    VEML7700_INTEGRATION_TIME_100MS = (0b0000),
    VEML7700_INTEGRATION_TIME_200MS = (0b0001),
    VEML7700_INTEGRATION_TIME_400MS = (0b0010),
    VEML7700_INTEGRATION_TIME_800MS = (0b0011),
} veml7700_integration_times_t;

/**
 * @brief VEML7700 persistence protections enumerator.
 */
typedef enum veml7700_persistence_protections_e {
    VEML7700_PERSISTENCE_PROTECTION_1 = (0b00),
    VEML7700_PERSISTENCE_PROTECTION_2 = (0b01),
    VEML7700_PERSISTENCE_PROTECTION_4 = (0b10),
    VEML7700_PERSISTENCE_PROTECTION_8 = (0b11),
} veml7700_persistence_protections_t;

/**
 * @brief VEML7700 power saving modes enumerator.
 */
typedef enum veml7700_power_saving_modes_e {
    VEML7700_POWER_SAVING_MODE_1 = (0b00),
    VEML7700_POWER_SAVING_MODE_2 = (0b01),
    VEML7700_POWER_SAVING_MODE_3 = (0b10),
    VEML7700_POWER_SAVING_MODE_4 = (0b11),
} veml7700_power_saving_modes_t;

/**
 * @brief VEML7700 configuration register structure.
 * 
 *
 * Table 1 - Configuration Register #0 (see datasheet pg. 7)
 * REGISTER NAME    BIT
 *  Reserved       15:13
 *  ALS_GAIN       12:11
 *  Reserved        10
 *  ALS_IT          9:6
 *  ALS_PERS        5:4
 *  Reserved        3:2
 *  ALS_INT_END     1
 *  ALS_SD          0
**/
typedef union __attribute__((packed)) veml7700_configuration_register_u {
    struct CFG_REG_BITS_TAG {
        bool                                shutdown:1;             /*!< als shut-down when true                    (bit:0)     */
        bool                                irq_enabled:1;          /*!< als interrupt enable when true             (bit:1)     */
        uint8_t                             reserved1:2;            /*!< reserved and set to 0                      (bit:2-3)   */
        veml7700_persistence_protections_t  persistence_protect:2;  /*!< sample count before the interrupt triggers (bit:4-5)   */
        veml7700_integration_times_t        integration_time:4;     /*!< time to measure                            (bit:6-9)   */
        uint8_t                             reserved2:1;            /*!< reserved and set to 0                      (bit:10)    */
        veml7700_gains_t                    gain:2;                 /*!< control the sensitivity                    (bit:11-12) */
        uint8_t                             reserved3:3;            /*!< reserved and set to 0                      (bit:13-15) */
    } bits;
    uint16_t reg;
} veml7700_configuration_register_t;

/**
 * @brief VEML7700 power saving mode register structure.
 * 
 *
 * Table 4 - Power Saving Modes (see datasheet pg. 8)
 * REGISTER NAME    BIT
 *  Reserved       15:3
 *  PSM             2:1
 *  PSM_EN          0
**/
typedef union __attribute__((packed)) veml7700_power_saving_mode_register_u {
    struct PSM_REG_BITS_TAG {
        bool                                power_saving_enabled:1; /*!< power saving enabeld when true     (bit:0)    */
        veml7700_power_saving_modes_t       power_saving_mode:2;    /*!< power saving mode                  (bit:1-2)  */
        uint16_t                            reserved:13;            /*!< reserved and set to 0              (bit:3-15) */
    } bits;
    uint16_t reg;
} veml7700_power_saving_mode_register_t;

/**
 * @brief VEML7700 interrupt status register structure.
 * 
 *
 * Table 7 - Interrupt Status (see datasheet pg. 9)
 * REGISTER NAME    BIT

**/
typedef union __attribute__((packed)) veml7700_interrupt_status_register_u {
    struct IRQ_STS_REG_BITS_TAG {
        uint16_t                                reserved:14;              /*!< reserved and set to 0                    (bit:0-13) */
        bool                                    hi_threshold_exceeded:1;  /*!< normal unless hi threshold is exceeded   (bit:14)   */
        bool                                    lo_threshold_exceeded:1;  /*!< normal unless lo threshold is exceeded   (bit:15)   */
    } bits;
    uint16_t reg;
} veml7700_interrupt_status_register_t;

/**
 * @brief VEML7700 identifier register structure.
 */
typedef union __attribute__((packed)) veml7700_identifier_register_u {
    struct ID_REG_BITS_TAG {
        uint8_t                                device_id_code:8;       /*!< device id code (fixed 0x81)  (bit:0-7)    */
        uint8_t                                slave_option_code:8;    /*!< slave address option code    (bit:8-15)   */
    } bits;
    uint16_t reg;
} veml7700_identifier_register_t;

/**
 * @brief VEML7700 device configuration structure.
 */
typedef struct veml7700_config_s {
    uint16_t                            i2c_address;            /*!< veml7700 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< veml7700 i2c device scl clock speed  */
    veml7700_gains_t                    gain;                   /*!< veml7700 gain sensitivity */
    veml7700_integration_times_t        integration_time;       /*!< veml7700 integration time to measure */
    veml7700_persistence_protections_t  persistence_protect;    /*!< veml7700 persistence protection */
    bool                                irq_enabled;            /*!< veml7700 interrupt enabled when true */
    bool                                power_disabled;         /*!< veml7700 power disabled when true */
    bool                                power_saving_enabled;   /*!< veml7700 power saving enabled when true */
    veml7700_power_saving_modes_t       power_saving_mode;      /*!< veml7700 power mode register */
    bool                                set_thresholds;         /*!< veml7700 configures interrupt thresholds */
    uint16_t                            hi_threshold;           /*!< veml7700 high threshold register for the interrupt */
    uint16_t                            lo_threshold;           /*!< veml7700 low threshold register for the interrupt */
} veml7700_config_t;

/**
 * @brief VEML7700 context structure.
 */
struct veml7700_context_t {
    veml7700_config_t                       dev_config;             /*!< veml7700 device configuration */
    i2c_master_dev_handle_t                 i2c_handle;             /*!< veml7700 i2c device handle */
    //float                                 resolution;			    /*!< Current resolution and multiplier */
    //uint32_t                              maximum_lux;		    /*!< Current maximum lux limit */
};

/**
 * @brief VEML7700 context structure definition.
 */
typedef struct veml7700_context_t veml7700_context_t;

/**
 * @brief VEML7700 handle structure definition.
 */
typedef struct veml7700_context_t *veml7700_handle_t;

/**
 * @brief Reads configuration register from VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] reg VEML7700 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_configuration_register(veml7700_handle_t handle, veml7700_configuration_register_t *const reg);

/**
 * @brief Writes configuration register to VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[in] reg VEML7700 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_configuration_register(veml7700_handle_t handle, const veml7700_configuration_register_t reg);

/**
 * @brief Reads high and low threshold registers from VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] hi_threshold VEML7700 high threshold register.
 * @param[out] lo_threshold VEML7700 lo threshold register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_threshold_registers(veml7700_handle_t handle, uint16_t *const hi_threshold, uint16_t *const lo_threshold);

/**
 * @brief Writes high and low threshold registers to VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[in] hi_threshold VEML7700 high threshold register.
 * @param[in] lo_threshold VEML7700 lo threshold register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_threshold_registers(veml7700_handle_t handle, const uint16_t hi_threshold, const uint16_t lo_threshold);

/**
 * @brief Reads power saving mode register from VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] reg VEML7700 power saving mode register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_power_saving_mode_register(veml7700_handle_t handle, veml7700_power_saving_mode_register_t *const reg);

/**
 * @brief Writes power saving mode register to VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[in] reg VEML7700 power saving mode register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_power_saving_mode_register(veml7700_handle_t handle, const veml7700_power_saving_mode_register_t reg);

/**
 * @brief Reads interrupt status register from VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] reg VEML7700 interrupt status register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_interrupt_status_register(veml7700_handle_t handle, veml7700_interrupt_status_register_t *const reg);

/**
 * @brief Reads identifier register from VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] reg VEML7700 identifier register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_identifier_register(veml7700_handle_t handle, veml7700_identifier_register_t *const reg);

/**
 * @brief Optimizes VEML7700 gain and integration time configuration.
 * 
 * @param handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_optimize_configuration(veml7700_handle_t handle);
esp_err_t veml7700_optimize_configuration___(veml7700_handle_t handle);


/**
 * @brief Initializes an VEML7700 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] veml7700_config VEML7700 device configuration.
 * @param[out] veml7700_handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_init(i2c_master_bus_handle_t master_handle, const veml7700_config_t *veml7700_config, veml7700_handle_t *veml7700_handle);

/**
 * @brief Reads ambient light counts from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param counts Ambient light counts.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_ambient_light_counts(veml7700_handle_t handle, uint16_t *const counts);

/**
 * @brief Reads ambient light (0 lux to 140 klux) from VEML7700.
 * 
 * @note This follows the official Vishay VEML7700 Application Note, rev. 17-Jan-2024.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] ambient_light Ambient light illumination in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_ambient_light(veml7700_handle_t handle, float *const ambient_light);

/**
 * @brief Reads optimal ambient light (0 lux to 140 klux) from VEML7700.
 * 
 * @note This follows the official Vishay VEML7700 Application Note, rev. 17-Jan-2024.
 * 
 * @note This doesn't seem to work all the time, results can vary, more testing required.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] ambient_light Ambient light illumination in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_ambient_light_auto(veml7700_handle_t handle, float *const ambient_light);

/**
 * @brief Reads white channel counts from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param counts White channel counts.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_white_channel_counts(veml7700_handle_t handle, uint16_t *const counts);

/**
 * @brief Reads white channel from VEML7700.
 * 
 * @note This follows the official Vishay VEML7700 Application Note, rev. 17-Jan-2024.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] white_light White channel illumination in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_white_channel(veml7700_handle_t handle, float *const white_light);

/**
 * @brief Reads optimal white channel from VEML7700.
 * 
 * @note This follows the official Vishay VEML7700 Application Note, rev. 17-Jan-2024.
 * 
 * @note This doesn't seem to work all the time, results can vary, more testing required.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] white_light White channel illumination in lux.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_white_channel_auto(veml7700_handle_t handle, float *const white_light);

/**
 * @brief Reads high and low als thresholds (lux) from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param hi_threshold VEML7700 high threshold setting.
 * @param lo_threshold VEML7700 low threshold setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_thresholds(veml7700_handle_t handle, uint16_t *const hi_threshold, uint16_t *const lo_threshold);

/**
 * @brief Writes high and low als thresholds (lux) to VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param hi_threshold VEML7700 high threshold setting.
 * @param lo_threshold VEML7700 low threshold setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_thresholds(veml7700_handle_t handle, const uint16_t hi_threshold, const uint16_t lo_threshold);

/**
 * @brief Reads als gain from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param gain VEML7700 gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_gain(veml7700_handle_t handle, veml7700_gains_t *const gain);

/**
 * @brief Writes als gain to VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param gain VEML7700 gain setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_gain(veml7700_handle_t handle, const veml7700_gains_t gain);

/**
 * @brief Reads als integration time from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param integration_time VEML7700 integration time setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_integration_time(veml7700_handle_t handle, veml7700_integration_times_t *const integration_time);

/**
 * @brief Writes als integration time to VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param integration_time VEML7700 integration time setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_integration_time(veml7700_handle_t handle, const veml7700_integration_times_t integration_time);

/**
 * @brief Reads als persistence protection from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param persistence_protection VEML7700 persistence protection setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_persistence_protection(veml7700_handle_t handle, veml7700_persistence_protections_t *const persistence_protection);

/**
 * @brief Writes als persistence protection to VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param persistence_protection VEML7700 persistence protection setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_persistence_protection(veml7700_handle_t handle, const veml7700_persistence_protections_t persistence_protection);

/**
 * @brief Reads power saving mode from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param power_saving_mode VEML7700 power saving mode setting.
 * @param power_saving_enabled VEML7700 power saving state setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_power_saving_mode(veml7700_handle_t handle, veml7700_power_saving_modes_t *const power_saving_mode, bool *const power_saving_enabled);

/**
 * @brief Reads power saving mode from VEML7700.
 * 
 * @param handle VEML7700 device handle.
 * @param power_saving_mode VEML7700 power saving mode setting.
 * @param power_saving_enabled VEML7700 power saving state setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_set_power_saving_mode(veml7700_handle_t handle, const veml7700_power_saving_modes_t power_saving_mode, const bool power_saving_enabled);

/**
 * @brief Enables interrupt assertion.
 * 
 * @param handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_enable_irq(veml7700_handle_t handle);

/**
 * @brief Disables interrupt assertion.
 * 
 * @param handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_disable_irq(veml7700_handle_t handle);

/**
 * @brief Reads interrupt status from VEML7700.
 *
 * @param[in] handle VEML7700 device handle.
 * @param[out] hi_threshold_exceeded true when high threshold is exceeded.
 * @param[out] lo_threshold_exceeded true when lo threshold is exceeded.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_get_interrupt_status(veml7700_handle_t handle, bool *const hi_threshold_exceeded, bool *const lo_threshold_exceeded);

/**
 * @brief Shuts down VEML7700 until woken.
 *
 * @param[in] handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_disable(veml7700_handle_t handle);

/**
 * @brief Wakes up VEML7700 from shut-down.
 *
 * @param[in] handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_enable(veml7700_handle_t handle);

/**
 * @brief Removes an VEML7700 device from master bus.
 *
 * @param[in] handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_remove(veml7700_handle_t handle);

/**
 * @brief Removes an VEML7700 device from master I2C bus and delete the handle.
 * 
 * @param handle VEML7700 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t veml7700_delete(veml7700_handle_t handle);


/**
 * @brief Converts VEML7700 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* VEML7700 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* veml7700_get_fw_version(void);

/**
 * @brief Converts VEML7700 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t VEML7700 firmware version number.
 */
int32_t veml7700_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __VEML7700_H__
