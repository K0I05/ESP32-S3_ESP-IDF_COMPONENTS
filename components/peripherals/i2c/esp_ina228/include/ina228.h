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
 * @file ina228.h
 * @defgroup drivers ina228
 * @{
 *
 * ESP-IDF driver for ina228 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __INA228_H__
#define __INA228_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "ina228_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * INA228 definitions
*/

#define I2C_INA228_DEV_CLK_SPD          UINT32_C(100000)    //!< ina228 I2C default clock frequency (100KHz)

#define I2C_INA228_ADDR_GND_GND         UINT8_C(0x40)       //!< ina228 I2C address, A1 pin - GND, A0 pin - GND
#define I2C_INA228_ADDR_GND_VS          UINT8_C(0x41)       //!< ina228 I2C address, A1 pin - GND, A0 pin - VS+
#define I2C_INA228_ADDR_GND_SDA         UINT8_C(0x42)       //!< ina228 I2C address, A1 pin - GND, A0 pin - SDA
#define I2C_INA228_ADDR_GND_SCL         UINT8_C(0x43)       //!< ina228 I2C address, A1 pin - GND, A0 pin - SCL
#define I2C_INA228_ADDR_VS_GND          UINT8_C(0x44)       //!< ina228 I2C address, A1 pin - VS+, A0 pin - GND
#define I2C_INA228_ADDR_VS_VS           UINT8_C(0x45)       //!< ina228 I2C address, A1 pin - VS+, A0 pin - VS+
#define I2C_INA228_ADDR_VS_SDA          UINT8_C(0x46)       //!< ina228 I2C address, A1 pin - VS+, A0 pin - SDA
#define I2C_INA228_ADDR_VS_SCL          UINT8_C(0x47)       //!< ina228 I2C address, A1 pin - VS+, A0 pin - SCL
#define I2C_INA228_ADDR_SDA_GND         UINT8_C(0x48)       //!< ina228 I2C address, A1 pin - SDA, A0 pin - GND
#define I2C_INA228_ADDR_SDA_VS          UINT8_C(0x49)       //!< ina228 I2C address, A1 pin - SDA, A0 pin - VS+
#define I2C_INA228_ADDR_SDA_SDA         UINT8_C(0x4a)       //!< ina228 I2C address, A1 pin - SDA, A0 pin - SDA
#define I2C_INA228_ADDR_SDA_SCL         UINT8_C(0x4b)       //!< ina228 I2C address, A1 pin - SDA, A0 pin - SCL
#define I2C_INA228_ADDR_SCL_GND         UINT8_C(0x4c)       //!< ina228 I2C address, A1 pin - SCL, A0 pin - GND
#define I2C_INA228_ADDR_SCL_VS          UINT8_C(0x4d)       //!< ina228 I2C address, A1 pin - SCL, A0 pin - VS+
#define I2C_INA228_ADDR_SCL_SDA         UINT8_C(0x4e)       //!< ina228 I2C address, A1 pin - SCL, A0 pin - SDA
#define I2C_INA228_ADDR_SCL_SCL         UINT8_C(0x4f)       //!< ina228 I2C address, A1 pin - SCL, A0 pin - SCL

/*

INA268 Wiring for Voltage (UPDATE)
- Source(+) to INA266 Voltage(+)
- Source(-) to INA266 Voltage(-)

INA266 Wiring for Current
- Source(+) to INA266 Current(+)
- INA266 Current(-) to Load(+)

INA266 Wiring for Voltage & Current
- Source(+) to INA266 Voltage(+)
- INA266 Voltage(+) to INA266 Current(+)
- INA266 Current(-) to Load(+)
- Source(-) to INA266 Voltage(-)
- INA266 Voltage(-) to Load(-)

*/

/*
 * INA228 macro definitions
*/
#define I2C_INA228_CONFIG_DEFAULT {                                         \
    .i2c_address                = I2C_INA228_ADDR_GND_GND,                  \
    .i2c_clock_speed            = I2C_INA228_DEV_CLK_SPD,                   \
    .averaging_mode             = INA228_AVG_MODE_1,                        \
    .shunt_voltage_conv_time    = INA228_CONV_TIME_50US,                    \
    .bus_voltage_conv_time      = INA228_CONV_TIME_50US,                    \
    .temperature_conv_time      = INA228_CONV_TIME_50US,                    \
    .operating_mode             = INA228_OP_MODE_CONT_BUS_SHUNT_VOLT_TEMP,  \
    .shunt_resistance           = 0.015,                                    \
    .max_current                = 0.5                                       \
    }

    // shunt resistor 0.002 ohms

/*
 * INA228 enumerator and structure declarations
*/


/**
 * ADC resolution/averaging
 */

/**
 * @brief Averaging modes enumerator for ADC resolution/averaging.
 */
typedef enum ina228_averaging_modes_e {
    INA228_AVG_MODE_1       = (0b000),  /*!< 1 sample averaged (default) */
    INA228_AVG_MODE_4       = (0b001),  /*!< 4 samples averaged */
    INA228_AVG_MODE_16      = (0b010),  /*!< 16 samples averaged */
    INA228_AVG_MODE_64      = (0b011),  /*!< 64 samples averaged */
    INA228_AVG_MODE_128     = (0b100),  /*!< 128 samples averaged */
    INA228_AVG_MODE_256     = (0b101),  /*!< 256 samples averaged */
    INA228_AVG_MODE_512     = (0b110),  /*!< 512 samples averaged */
    INA228_AVG_MODE_1024    = (0b111)   /*!< 1024 samples averaged */
} ina228_averaging_modes_t;

/**
 * @brief Voltage or temperature conversion times enumerator for ADC resolution/averaging.
 */
typedef enum ina228_conversion_times_e {
    INA228_CONV_TIME_50US     = (0b000),  /*!< 50 us voltage/temperature conversion time */
    INA228_CONV_TIME_84US     = (0b001),  /*!< 84 us voltage/temperature conversion time */
    INA228_CONV_TIME_150US    = (0b010),  /*!< 150 us voltage/temperature conversion time */
    INA228_CONV_TIME_280US    = (0b011),  /*!< 280 us voltage/temperature conversion time */
    INA228_CONV_TIME_540US    = (0b100),  /*!< 540 us voltage/temperature conversion time (default) */
    INA228_CONV_TIME_1052US   = (0b101),  /*!< 1052 us voltage/temperature conversion time */
    INA228_CONV_TIME_2074US   = (0b110),  /*!< 2074 us voltage/temperature conversion time */
    INA228_CONV_TIME_4120US   = (0b111)   /*!< 4120 us voltage/temperature conversion time */
} ina228_conversion_times_t;

/**
 * @brief Delay for initial ADC conversion in steps of 2 milliseconds.
 */
typedef enum ina228_adc_conversion_steps_e {
    INA228_ADC_CONV_STEP_0MS = 0,
    INA228_ADC_CONV_STEP_2MS = 1,
    INA228_ADC_CONV_STEP_4MS = 2,
    INA228_ADC_CONV_STEP_6MS = 3,
    INA228_ADC_CONV_STEP_8MS = 4,
    INA228_ADC_CONV_STEP_10MS = 5,
    INA228_ADC_CONV_STEP_12MS = 6,
    INA228_ADC_CONV_STEP_14MS = 7,
    INA228_ADC_CONV_STEP_16MS = 8,
    INA228_ADC_CONV_STEP_18MS = 9,
    INA228_ADC_CONV_STEP_20MS = 10,
    INA228_ADC_CONV_STEP_22MS = 11,
    INA228_ADC_CONV_STEP_24MS = 12,
    INA228_ADC_CONV_STEP_26MS = 13,
    INA228_ADC_CONV_STEP_28MS = 14,
    INA228_ADC_CONV_STEP_30MS = 15,
    INA228_ADC_CONV_STEP_32MS = 16,
    INA228_ADC_CONV_STEP_34MS = 17,
    // to do 
    INA228_ADC_CONV_STEP_510MS = 255
} ina228_adc_conversion_steps_t;

/**
 * @brief .
 */
typedef enum ina228_adc_range_e {
    INA228_ADC_RANGE_163_84MV = 0,  /* +/- 163.84 mV */
    INA228_ADC_RANGE_40_96MV        /* +/- 40.96 mV */
} ina228_adc_range_t;

/**
 * @brief Current conversion times enumerator for ADC resolution/averaging.
 */
typedef enum ina228_operating_modes_e {
    INA228_OP_MODE_SHUTDOWN                 = (0b0000),  /*!< device is powered down */
    INA228_OP_MODE_TRIG_BUS_VOLT            = (0b0001),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_TRIG_SHUNT_VOLT          = (0b0010),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_TRIG_SHUNT_BUS_VOLT      = (0b0011),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_TRIG_TEMP                = (0b0100),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_TRIG_TEMP_BUS_VOLT       = (0b0101),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_TRIG_TEMP_SHUNT_VOLT     = (0b0110),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_TRIG_BUS_SHUNT_VOLT_TEMP = (0b0111),  /*!< triggers single-shot conversion when set */
    INA228_OP_MODE_SHUTDOWN2                = (0b1000),  /*!< device is powered down */
    INA228_OP_MODE_CONT_BUS_VOLT            = (0b1001),  /*!< normal operating mode when set */
    INA228_OP_MODE_CONT_SHUNT_VOLT          = (0b1010),  /*!< normal operating mode when set */
    INA228_OP_MODE_CONT_SHUNT_BUS_VOLT      = (0b1011),  /*!< normal operating mode when set */
    INA228_OP_MODE_CONT_TEMP                = (0b1100),  /*!< normal operating mode when set */
    INA228_OP_MODE_CONT_TEMP_BUS_VOLT       = (0b1101),  /*!< normal operating mode when set */
    INA228_OP_MODE_CONT_TEMP_SHUNT_VOLT     = (0b1110),  /*!< normal operating mode when set */
    INA228_OP_MODE_CONT_BUS_SHUNT_VOLT_TEMP = (0b1111),  /*!< normal operating mode when set */
} ina228_operating_modes_t;

/**
 * @brief All-register reset, ADC range, temperature compensation
 * ADC conversion delay, accumulation registers (reset = 0h).
 */
typedef union __attribute__((packed)) ina228_config_register_u {
    struct {
        int8_t                        reserved:4;
        ina228_adc_range_t            adc_range:1;
        bool                          shunt_temperature_comp_enabled:1;
        ina228_adc_conversion_steps_t adc_conversion_delay:8;
        bool                          reset_accumulation_register:1;        
        bool                          reset_enabled:1;         /*!< reset state BIT15 */
    } bits;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
} ina228_config_register_t;

/**
 * @brief All-register reset, ADC range, temperature compensation
 * ADC conversion delay, accumulation registers (reset = FB68h).
 */
typedef union __attribute__((packed)) ina228_adc_config_register_u {
    struct {
        ina228_averaging_modes_t     averaging_mode:3;
        ina228_conversion_times_t    temperature_conv_time:3;
        ina228_conversion_times_t    shunt_voltage_conv_time:3;
        ina228_conversion_times_t    bus_voltage_conv_time:3;
        ina228_operating_modes_t     operating_mode:4;         /*!< reset state BIT15 */
    } bits;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
} ina228_adc_config_register_t;

typedef union __attribute__((packed)) ina228_shunt_calibration_register_u {
    struct {
        uint8_t     reserved:1;
        uint16_t    shunt_calibration:15;
    } bits;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
} ina228_shunt_calibration_register_t;

typedef union __attribute__((packed)) ina228_shunt_temperature_coefficient_register_u {
    struct {
        uint8_t     reserved:2;
        uint16_t    temperature_coefficient:14;
    } bits;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
} ina228_shunt_temperature_coefficient_register_t;

/**
 * @brief Alert configuration and Conversion Ready flag.
 */
typedef union __attribute__((packed)) ina228_diagnostic_alert_register_u {
    struct {
        bool        memory_checksum_error:1;    /*!<  (bit:0) */
        bool        conversion_ready_flag:1;    /*!<  (bit:1) */
        bool        power_over_limit:1;         /*!<  (bit:2) */
        bool        bus_volt_under_volt:1;      /*!<  (bit:3) */ 
        bool        bus_volt_over_volt:1;       /*!<  (bit:4) */
        bool        shunt_volt_under_volt:1;    /*!<  (bit:5) */ 
        bool        shunt_volt_over_volt:1;     /*!<  (bit:6) */
        bool        temperature_over:1;         /*!<  (bit:7) */
        uint8_t     reserved:1;                 /*!<  (bit:8) */
        bool        math_overflow_flag:1;       /*!<  (bit:9) */
        bool        charge_overflow_flag:1;     /*!<  (bit:10) */
        bool        energy_overflow_flag:1;     /*!<  (bit:11) */
        bool        alert_polarity_bit:1;       /*!<  (bit:12) */
        bool        alert_conv_ready_flag:1;    /*!<  (bit:13) */
        bool        alert_conv_ready_enabled:1; /*!<  (bit:14) */
        bool        alert_latch_enable:1;       /*!<  (bit:15) */
    } bits;                  /*!< represents the 16-bit mask/enable register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit mask/enable register as `uint16_t` */
} ina228_diagnostic_alert_register_t;

/**
 * @brief INA228 device configuration.
 */
typedef struct ina228_config_s {
    uint16_t                        i2c_address;                /*!< ina228 i2c device address */
    uint32_t                        i2c_clock_speed;            /*!< ina228 i2c device scl clock speed  */
    ina228_adc_range_t              adc_range;                  /*!< ina228 adc range */
    ina228_averaging_modes_t        averaging_mode;             /*!< ina228 averaging mode */
    ina228_conversion_times_t       shunt_voltage_conv_time;    /*!< ina228 shunt voltage conversion time */
    ina228_conversion_times_t       bus_voltage_conv_time;      /*!< ina228 bus voltage conversion time */
    ina228_conversion_times_t       temperature_conv_time;      /*!< ina228 temperature conversion time */
    ina228_operating_modes_t        operating_mode;             /*!< ina228 operating mode */
    float                           shunt_resistance;           /*!< ina228 shunt resistance, Ohm */
    float                           max_current;                /*!< ina228 maximum expected current, A */
} ina228_config_t;


/**
 * @brief INA228 context structure.
 */
struct ina228_context_t {
    ina228_config_t                 dev_config;       /*!< ina228 device configuration */
    i2c_master_dev_handle_t         i2c_handle;       /*!< ina228 I2C device handle */
    float                           current_lsb;      /*!< ina228 current LSB value, uA/bit, this is automatically configured */
};

/**
 * @brief INA228 context structure definition.
 */
typedef struct ina228_context_t ina228_context_t;

/**
 * @brief INA228 handle structure definition.
 */
typedef struct ina228_context_t *ina228_handle_t;

/**
 * @brief Reads the configuration register from the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_get_configuration_register(ina228_handle_t handle, ina228_config_register_t *const reg);

/**
 * @brief Writes the configuration register to the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_set_configuration_register(ina228_handle_t handle, const ina228_config_register_t reg);

/**
 * @brief Reads the ADC configuration register from the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 ADC configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_get_adc_configuration_register(ina228_handle_t handle, ina228_adc_config_register_t *const reg);

/**
 * @brief Writes the ADC configuration register to the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 ADC configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_set_adc_configuration_register(ina228_handle_t handle, const ina228_adc_config_register_t reg);

/**
 * @brief Reads the calibration register from the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 calibration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_get_shunt_calibration_register(ina228_handle_t handle, ina228_shunt_calibration_register_t *const reg);

/**
 * @brief Writes the calibration register to the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 calibration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_set_shunt_calibration_register(ina228_handle_t handle, const ina228_shunt_calibration_register_t reg);

/**
 * @brief Reads the mask/enable register from the INA228.
 * 
 * @param handle INA228 device handle.
 * @param reg INA228 mask/enable register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_get_diagnostic_alert_register(ina228_handle_t handle, ina228_diagnostic_alert_register_t *const reg);

/**
 * @brief initializes an INA228 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] ina226_config INA228 device configuration.
 * @param[out] ina226_handle INA228 device handle.
 * @return ESP_OK on success.
 */
esp_err_t ina228_init(i2c_master_bus_handle_t master_handle, const ina228_config_t *ina228_config, ina228_handle_t *ina228_handle);

/**
 * @brief Reads bus voltage (V) from INA228.
 *
 * @param[in] handle INA228 device handle.
 * @param[out] voltage INA228 bus voltage, V.
 * @return ESP_OK on success.
 */
esp_err_t ina228_get_bus_voltage(ina228_handle_t handle, float *const voltage);

/**
 * @brief Reads shunt voltage (V) from INA228.
 *
 * @param[in] handle INA228 device handle.
 * @param[out] voltage INA228 shunt voltage, V.
 * @return ESP_OK on success.
 */
esp_err_t ina228_get_shunt_voltage(ina228_handle_t handle, float *const voltage);

/**
 * @brief Reads current (A) from INA228.
 *
 * @note This function works properly only after calibration.
 *
 * @param[in] handle INA228 device handle.
 * @param[out] current INA228 current, A.
 * @return ESP_OK on success.
 */
esp_err_t ina228_get_current(ina228_handle_t handle, float *const current);

/**
 * @brief Reads power (W) from INA228.
 *
 * @note This function works properly only after calibration.
 *
 * @param[in] handle INA228 device handle.
 * @param[out] power INA228 power, W.
 * @return ESP_OK on success.
 */
esp_err_t ina228_get_power(ina228_handle_t handle, float *const power);

esp_err_t ina228_get_temperature(ina228_handle_t handle, float *const temperature);


/**
 * @brief Reads operating mode from the INA228.
 *
 * @param[in] handle INA228 device handle
 * @param[out] mode Operating mode setting.
 * @return ESP_OK on success.
 */
esp_err_t ina228_get_operating_mode(ina228_handle_t handle, ina228_operating_modes_t *const mode);

/**
 * @brief Writes operating mode to the INA228.
 *
 * @param[in] handle INA228 device handle
 * @param[out] mode Operating mode setting.
 * @return ESP_OK on success.
 */
esp_err_t ina228_set_operating_mode(ina228_handle_t handle, const ina228_operating_modes_t mode);

esp_err_t ina228_get_averaging_mode(ina228_handle_t handle, ina228_averaging_modes_t *const mode);

esp_err_t ina228_set_averaging_mode(ina228_handle_t handle, const ina228_averaging_modes_t mode);

esp_err_t ina228_get_bus_volt_conv_time(ina228_handle_t handle, ina228_conversion_times_t *const conv_time);

esp_err_t ina228_set_bus_volt_conv_time(ina228_handle_t handle, ina228_conversion_times_t *const conv_time);

esp_err_t ina228_get_shunt_volt_conv_time(ina228_handle_t handle, ina228_conversion_times_t *const conv_time);

esp_err_t ina228_set_shunt_volt_conv_time(ina228_handle_t handle, ina228_conversion_times_t *const conv_time);

/**
 * @brief Resets the INA228.
 *
 * Same as power-on reset. Resets all registers to default values.
 * Calibration is conducted automatically after reset.
 *
 * @param[in] handle INA228 device handle
 * @return ESP_OK on success.
 */
esp_err_t ina228_reset(ina228_handle_t handle);

/**
 * @brief Removes an INA228 device from master bus.
 *
 * @param[in] handle INA228 device handle
 * @return ESP_OK on success.
 */
esp_err_t ina228_remove(ina228_handle_t handle);

/**
 * @brief Removes an INA228 device from master bus and frees handle.
 * 
 * @param[in] handle INA228 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina228_delete(ina228_handle_t handle);

/**
 * @brief Converts INA228 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* INA228 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ina228_get_fw_version(void);

/**
 * @brief Converts INA228 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t INA228 firmware version number.
 */
int32_t ina228_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __INA228_H__
