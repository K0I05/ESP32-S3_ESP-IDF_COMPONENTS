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
 * @file ccs811.h
 * @defgroup drivers ccs811
 * @{
 *
 * ESP-IDF driver for ccs811 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __CCS811_H__
#define __CCS811_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "ccs811_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * CCS811 definitions
*/
#define I2C_CCS811_DEV_CLK_SPD                  UINT32_C(100000) //!< ccs811 I2C default clock frequency (100KHz)

#define I2C_CCS811_DEV_ADDR_LO                  UINT8_C(0x5a)   //!< ccs811 I2C address when ADDR pin floating/low
#define I2C_CCS811_DEV_ADDR_HI                  UINT8_C(0x5b)   //!< ccs811 I2C address when ADDR pin high

#define CCS811_ERROR_TABLE_SIZE                 (6)             //!< ccs811 I2C error table size
#define CCS811_MEASURE_MODE_TABLE_SIZE          (5)             //!< ccs811 I2C measure mode table size


/*
 * CCS811 macro definitions
*/

#define I2C_CCS811_CONFIG_DEFAULT   {                                           \
        .i2c_address                = I2C_CCS811_DEV_ADDR_LO,                   \
        .i2c_clock_speed            = I2C_CCS811_DEV_CLK_SPD,                   \
        .wake_io_enabled            = false,                                    \
        .reset_io_enabled           = false,                                    \
        .irq_threshold_enabled      = false,                                    \
        .irq_data_ready_enabled     = false,                                    \
        .drive_mode                 = CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ,     \
        .set_environmental_data     = false }


/*
 * CCS811 enumerator and structure declarations
*/

/**
 * CCS811 drive modes enumerator.
 */
typedef enum ccs811_drive_modes_e {
    CCS811_DRIVE_MODE_IDLE                  = (0b000),   //!< idle (measurements are disabled in this mode)
    CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ    = (0b001),   //!< constant power mode, IAQ measurement every second
    CCS811_DRIVE_MODE_PULSE_HEATING_IAQ     = (0b010),   //!< pulse heating mode IAQ measurement every 10 seconds
    CCS811_DRIVE_MODE_LP_PULSE_HEATING_IAQ  = (0b011),   //!< low power pulse heating mode IAQ measurement every 60 seconds
    CCS811_DRIVE_MODE_CONSTANT_POWER        = (0b100)    //!< constant power mode, sensor measurement every 250ms
} ccs811_drive_modes_t;

/**
 * CCS811 firmware modes enumerator.
 */
typedef enum ccs811_firmware_modes_e {
    CCS811_FW_MODE_BOOT = 0,   //!< firmware is in boot mode, this allows new firmware to be loaded
    CCS811_FW_MODE_APP  = 1    //!< formware is in application mode, deivce is ready to take ADC measurements
} ccs811_firmware_modes_t;


/**
 * @brief CCS811 status register structure.
 */
typedef union __attribute__((packed)) ccs811_status_register_u {
    struct STS_REG_BITS_TAG {
        bool                            error:1;                /*!< error when true                        (bit:0)   */
        uint8_t                         reserved:2;             /*!< reserved and set to 0                  (bit:2-1) */
        bool                            data_ready:1;           /*!< data ready when true                   (bit:3)   */
        bool                            app_valid:1;            /*!< application firmware passed when true  (bit:4)   */
        bool                            app_verify_completed:1; /*!< application verify completed when true (bit:5)   */
        bool                            app_erase_completed:1;  /*!< application erase completed when true  (bit:6)   */
        ccs811_firmware_modes_t         firmware_mode:1;        /*!< firmware mode                          (bit:7)   */
    } bits;          /*!< represents the 8-bit register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit register as `uint8_t` */
} ccs811_status_register_t;

/**
 * @brief CCS811 measure mode and condition register structure.
 */
typedef union __attribute__((packed)) ccs811_measure_mode_register_u {
    struct MODE_REG_BITS_TAG {
        uint8_t                                  reserved1:2;            /*!< reserved and set to 0                     (bit:1-0) */
        bool                                     irq_threshold_enabled:1;/*!< threshold interrupt enabled when true     (bit:2)   */
        bool                                     irq_data_ready_enabled:1;/*!< data ready interrupt enabled when true    (bit:3)   */
        ccs811_drive_modes_t                     drive_mode:3;           /*!< drive mode                                (bit:6-4) */
        uint8_t                                  reserved2:1;            /*!< reserved and set to 0                     (bit:7)   */
    } bits;         /*!< represents the 8-bit register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit register as `uint8_t`   */
} ccs811_measure_mode_register_t;

/**
 * @brief CCS811 raw data register structure (review - not correct).
 */
typedef union __attribute__((packed)) ccs811_raw_data_register_u {
    struct RDAT_REG_BITS_TAG {
        uint16_t    raw_adc:10;    /*!< raw adc        (bit:) */
        uint16_t    current:6;     /*!< current        (bit:7)   */
    } bits;         /*!< represents the 16-bit register parts in bits. */
    uint16_t reg;   /*!< represents the 16-bit register as `uint16_t`  */
} ccs811_raw_data_register_t;

/**
 * @brief CCS811 threshold value structure (big endien).
 */
typedef union __attribute__((packed)) ccs811_threshold_value_u {
    struct THRVAL_REG_BITS_TAG {
        uint8_t    hi_byte:8;  /*!< low to medium threshold (1500ppm = 0x05DC default)  (bit:7-0)    */
        uint8_t    lo_byte:8;  /*!< low to medium threshold (1500ppm = 0x05DC default)  (bit:8-15)   */
    } bits;              /*!< represents the 16-bit register parts in bits. */
    uint16_t value;     /*!< represents the 16-bit threshold value register as `uint16_t`  */
} ccs811_threshold_value_t;

/**
 * @brief CCS811 thresholds register structure.
 */
typedef struct ccs811_thresholds_register_s {
    ccs811_threshold_value_t    low_to_med;  /*!< low to medium threshold (1500ppm = 0x05DC default)   */
    ccs811_threshold_value_t    med_to_high; /*!< medium to high threshold (2500ppm = 0x09C4 default)  */
    uint8_t                     hysteresis;  /*!< threshold hysteresis value (50ppm = 0x default)*/
} ccs811_thresholds_register_t;

/**
 * @brief CCS811 environmental data register structure.
 */
typedef struct ccs811_environmental_data_register_s {
    float temperature;  /*!< temperature (25 C = 0x64, 0x00 default)   */
    float humidity;     /*!< humidity (50% = 0x64, 0x00 default)  */
} ccs811_environmental_data_register_t;

/**
 * @brief CCS811 firmware version format (bootloader 0x23 and application 0x24) structure (review).
 */
typedef union __attribute__((packed)) ccs811_firmware_version_format_u {
    struct FWVF_PARTS_TAG {
        uint16_t    major:4;      /*!< major         (bit:3-0)  */
        uint16_t    minor:4;      /*!< minor         (bit:4-7)  */
        uint16_t    trivial:8;    /*!< trivial       (bit:15-8) */
    } parts;            /*!< represents the 16-bit version parts in bits. */
    uint16_t version;   /*!< represents the 16-bit version as `uint16_t`  */
} ccs811_firmware_version_format_t;

/**
 * @brief CCS811 error codes `ERROR_ID` register structure.
 */
typedef union __attribute__((packed)) ccs811_error_code_register_u {
    struct ERRC_REG_BITS_TAG {
        bool            write_register_invalid:1;   /*!< write register invalid when tru            (bit:0) */
        bool            read_register_invalid:1;    /*!< read register invalid when true            (bit:1) */
        bool            drive_mode_invalid:1;       /*!< measurement drive mode invalid when true   (bit:2) */
        bool            max_resistance_exceeded:1;  /*!< maximum resistance exceeded when true      (bit:3) */
        bool            heater_current_fault:1;     /*!< heater current supply fault when true      (bit:4) */
        bool            heater_voltage_fault:1;     /*!< heater voltage supply fault when true      (bit:5) */
        uint8_t         reserved1:1;                /*!< reserved and set to 0                      (bit:6) */
        uint8_t         reserved2:1;                /*!< reserved and set to 0                      (bit:7) */
    } bits;          /*!< represents the 8-bit register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit register as `uint8_t`   */
} ccs811_error_code_register_t;

/**
 * @brief CCS811 error row definition structure.
 */
typedef struct ccs811_error_row_s {
    const char *code;   /*!< error code */
    const char *msg;    /*!< error message */
} ccs811_error_row_t;

/**
 * @brief CCS811 measure mode row definition structure.
 */
typedef struct ccs811_measure_mode_row_s {
    ccs811_drive_modes_t mode;                                           /*!< drive mode */
    const char          *desc; /*!< drive mode description */
} ccs811_measure_mode_row_t;

/**
 * @brief CCS811 configuration structure definition.
 */
typedef struct {
    uint16_t                 i2c_address;               /*!< ccs811 i2c device address */
    uint32_t                 i2c_clock_speed;           /*!< ccs811 i2c device scl clock speed  */
    bool                     irq_data_ready_io_enabled; /*!< ccs811 flag to enable hardware interrupt */
    gpio_num_t               irq_data_ready_io_num;     /*!< mcu interrupt gpio number for ccs811 device */
    bool                     wake_io_enabled;           /*!< ccs811 flag to enable hardware wake */
    gpio_num_t               wake_io_num;               /*!< mcu wake gpio number for ccs811 device */
    bool                     reset_io_enabled;          /*!< ccs811 flag to enable hardware reset */
    gpio_num_t               reset_io_num;              /*!< mcu reset gpio number for ccs811 device */
    bool                     irq_threshold_enabled;      /*!< interrupt threshold asserted when enabled */
    bool                     irq_data_ready_enabled;     /*!< interrupt data ready asserted when enabled  */
    ccs811_drive_modes_t     drive_mode;                 /*!< drive mode */
    bool                     set_environmental_data;     /*!< flag to set user-defined environmental data */
    float                    temperature;                /*!< user-defined temperature environmental data */
    float                    humidity;                   /*!< user-defined humidity environmental data */
} ccs811_config_t;



/**
 * @brief CCS811 opaque handle structure definition.
 */
typedef void* ccs811_handle_t;


/**
 * @brief Reads status register from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] reg CCS811 status register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_status_register(ccs811_handle_t handle, ccs811_status_register_t *const reg);

/**
 * @brief Reads measure mode register from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] reg CCS811 measure mode register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_measure_mode_register(ccs811_handle_t handle, ccs811_measure_mode_register_t *const reg);

/**
 * @brief Writes measure mode register to CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] reg CCS811 measure mode register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_measure_mode_register(ccs811_handle_t handle, const ccs811_measure_mode_register_t reg);

/**
 * @brief Reads error register from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] reg CCS811 error register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_error_register(ccs811_handle_t handle, ccs811_error_code_register_t *const reg);

/**
 * @brief Writes environmental compensation factors data to CCS811 register.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] temperature temperature compensation in degrees Celsius (default: 25 C).
 * @param[in] humidity relative humidity compensation in percentage (default: 50 %).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_environmental_data_register(ccs811_handle_t handle, const float temperature, const float humidity);

/**
 * @brief Writes eCO2 thresholds to CCS811 register.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] low_to_med Low to medium threshold  within a range of 400 to 32768 ppm (1500 ppm).
 * @param[in] med_to_high Medium to high threshold within a range of 400 to 32768 ppm (2500 ppm).
 * @param[in] hysteresis Variance for thresholds in ppm (default: 50 ppm).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_thresholds_register(ccs811_handle_t handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis);

/**
 * @brief Reads encoded version of the current baseline register from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] reg CCS811 encoded version of the baseline register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_baseline_register(ccs811_handle_t handle, uint16_t *const reg);

/**
 * @brief Writes encoded version to the CCS811 baseline register.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] reg CCS811 encoded version of the baseline register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_baseline_register(ccs811_handle_t handle, const uint16_t reg);

/**
 * @brief Reads hardware identifier register from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] reg CCS811 hardware identifier register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_hardware_identifier_register(ccs811_handle_t handle, uint8_t *const reg);

/**
 * @brief Reads hardware version register from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] reg CCS811 hardware version register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_hardware_version_register(ccs811_handle_t handle, uint8_t *const reg);

/**
 * @brief Starts the CCS811 application.
 * 
 * @param[in] handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_start_application(ccs811_handle_t handle);

/**
 * @brief Initializes a CCS811 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] ccs811_config CCS811 device configuration.
 * @param[out] ccs811_handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_init(i2c_master_bus_handle_t master_handle, const ccs811_config_t *ccs811_config, ccs811_handle_t *ccs811_handle);

/**
 * @brief Reads air quality measurement from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] eco2 Equivalent CO2 in ppm (range is from 400 to 32768 ppm).
 * @param[out] etvoc Equivalent Total Volatile Oragnic Compound (TVOC) in ppb (range is from 0 to 29206 ppb).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_measurement(ccs811_handle_t handle, uint16_t *eco2, uint16_t *etvoc);

/**
 * @brief Writes environmental compensation factors data to CCS811 register.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] temperature Temperature compensation in degrees Celsius (default: 25 C).
 * @param[in] humidity Relative humidity compensation in percentage (default: 50 %).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_environmental_data(ccs811_handle_t handle, const float temperature, const float humidity);

/**
 * @brief Writes eCO2 thresholds to CCS811 register.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] low_to_med Low to medium threshold  within a range of 400 to 32768 ppm (1500 ppm).
 * @param[in] med_to_high Medium to high threshold within a range of 400 to 32768 ppm (2500 ppm).
 * @param[in] hysteresis Variance for thresholds in ppm (default: 50 ppm).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_thresholds(ccs811_handle_t handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis);

/**
 * @brief Reads drive mode from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] mode CCS811 measurement drive mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_drive_mode(ccs811_handle_t handle, ccs811_drive_modes_t *const mode);

/**
 * @brief Writes drive mode to CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[in] mode CCS811 measurement drive mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_set_drive_mode(ccs811_handle_t handle, const ccs811_drive_modes_t mode);

/**
 * @brief Reads firmware mode from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] mode CCS811 firmware mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_firmware_mode(ccs811_handle_t handle, ccs811_firmware_modes_t *const mode);

/**
 * @brief Reads NTC resistance connected to CCS811 per AMS application note AN000372.
 * 
 * @param handle CCS811 device handle.
 * @param r_ref CCS811 resistance reference value. 
 * @param resistance CCS811 NTC resistance value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_ntc_resistance(ccs811_handle_t handle, const uint32_t r_ref, uint32_t *const resistance);

/**
 * @brief Reads data ready status from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] ready CCS811 data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_data_status(ccs811_handle_t handle, bool *const ready);

/**
 * @brief Reads error status from CCS811.
 * 
 * @param[in] handle CCS811 device handle.
 * @param[out] error CCS811 error is present when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_get_error_status(ccs811_handle_t handle, bool *const error);

/**
 * @brief Issues soft-reset to CCS811.
 *
 * @param[in] handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_reset(ccs811_handle_t handle);

/**
 * @brief Wakes the CCS811 through the configured wake GPIO pin number.
 *
 * @param[in] handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_io_wake(ccs811_handle_t handle);

/**
 * @brief Puts the CCS811 a sleep through the configured wake GPIO pin number.
 *
 * @param[in] handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_io_sleep(ccs811_handle_t handle);

/**
 * @brief Resets the CCS811 through the configured reset GPIO pin number.
 *
 * @param[in] handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_io_reset(ccs811_handle_t handle);

/**
 * @brief Removes a CCS811 device from master bus.
 *
 * @param[in] handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_remove(ccs811_handle_t handle);

/**
 * @brief Removes an CCS811 device from master I2C bus and delete the handle.
 * 
 * @param handle CCS811 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ccs811_delete(ccs811_handle_t handle);

/**
 * @brief Decodes CCS811 device error to a textual message representation.
 * 
 * @param[in] code CCS811 I2C register error.
 * @return char textual representation of error message on success.
 */
const char *ccs811_err_to_message(const ccs811_error_code_register_t error_reg);

/**
 * @brief Decodes CCS811 device error to a textual code representation.
 * 
 * @param[in] code CCS811 I2C register error.
 * @return char textual representation of error code on success.
 */
const char *ccs811_err_to_code(const ccs811_error_code_register_t error_reg);

/**
 * @brief Decodes enumerated drive mode to a textual descriptive representation.
 * 
 * @param[in] mode measurement drive mode.
 * @return char textual representation of measure mode on success.
 */
const char *ccs811_measure_mode_description(const ccs811_drive_modes_t mode);

/**
 * @brief Converts CCS811 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* CCS811 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ccs811_get_fw_version(void);

/**
 * @brief Converts CCS811 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t CCS811 firmware version number.
 */
int32_t ccs811_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __CCS811_H__
