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
 * @file ina226.h
 * @defgroup drivers ina226
 * @{
 *
 * ESP-IDF driver for ina226 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __INA226_H__
#define __INA226_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "ina226_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * INA226 definitions
*/

#define I2C_INA226_DEV_CLK_SPD          UINT32_C(100000)    //!< ina226 I2C default clock frequency (100KHz)

#define I2C_INA226_ADDR_GND_GND         UINT8_C(0x40)       //!< ina226 I2C address, A1 pin - GND, A0 pin - GND
#define I2C_INA226_ADDR_GND_VS          UINT8_C(0x41)       //!< ina226 I2C address, A1 pin - GND, A0 pin - VS+
#define I2C_INA226_ADDR_GND_SDA         UINT8_C(0x42)       //!< ina226 I2C address, A1 pin - GND, A0 pin - SDA
#define I2C_INA226_ADDR_GND_SCL         UINT8_C(0x43)       //!< ina226 I2C address, A1 pin - GND, A0 pin - SCL
#define I2C_INA226_ADDR_VS_GND          UINT8_C(0x44)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - GND
#define I2C_INA226_ADDR_VS_VS           UINT8_C(0x45)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - VS+
#define I2C_INA226_ADDR_VS_SDA          UINT8_C(0x46)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - SDA
#define I2C_INA226_ADDR_VS_SCL          UINT8_C(0x47)       //!< ina226 I2C address, A1 pin - VS+, A0 pin - SCL
#define I2C_INA226_ADDR_SDA_GND         UINT8_C(0x48)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - GND
#define I2C_INA226_ADDR_SDA_VS          UINT8_C(0x49)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - VS+
#define I2C_INA226_ADDR_SDA_SDA         UINT8_C(0x4a)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - SDA
#define I2C_INA226_ADDR_SDA_SCL         UINT8_C(0x4b)       //!< ina226 I2C address, A1 pin - SDA, A0 pin - SCL
#define I2C_INA226_ADDR_SCL_GND         UINT8_C(0x4c)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - GND
#define I2C_INA226_ADDR_SCL_VS          UINT8_C(0x4d)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - VS+
#define I2C_INA226_ADDR_SCL_SDA         UINT8_C(0x4e)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - SDA
#define I2C_INA226_ADDR_SCL_SCL         UINT8_C(0x4f)       //!< ina226 I2C address, A1 pin - SCL, A0 pin - SCL

/*

INA266 Wiring for Voltage
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
 * INA226 macro definitions
*/
#define I2C_INA226_CONFIG_DEFAULT {                                     \
    .i2c_address                = I2C_INA226_ADDR_GND_GND,              \
    .i2c_clock_speed            = I2C_INA226_DEV_CLK_SPD,               \
    .averaging_mode             = INA226_AVG_MODE_1,                    \
    .shunt_voltage_conv_time    = INA226_VOLT_CONV_TIME_1_1MS,          \
    .bus_voltage_conv_time      = INA226_VOLT_CONV_TIME_1_1MS,          \
    .operating_mode             = INA226_OP_MODE_CONT_SHUNT_BUS,        \
    .shunt_resistance           = 0.002,                                \
    .max_current                = 0.5                                   \
    }

    // shunt resistor 0.002 ohms

/*
 * INA226 enumerator and structure declarations
*/


/**
 * ADC resolution/averaging
 */

/**
 * @brief Averaging modes enumerator for ADC resolution/averaging.
 */
typedef enum ina226_averaging_modes_e {
    INA226_AVG_MODE_1       = (0b000),  /*!< 1 sample averaged (default) */
    INA226_AVG_MODE_4       = (0b001),  /*!< 4 samples averaged */
    INA226_AVG_MODE_16      = (0b010),  /*!< 16 samples averaged */
    INA226_AVG_MODE_64      = (0b011),  /*!< 64 samples averaged */
    INA226_AVG_MODE_128     = (0b100),  /*!< 128 samples averaged */
    INA226_AVG_MODE_256     = (0b101),  /*!< 256 samples averaged */
    INA226_AVG_MODE_512     = (0b110),  /*!< 512 samples averaged */
    INA226_AVG_MODE_1024    = (0b111)   /*!< 1024 samples averaged */
} ina226_averaging_modes_t;

/**
 * @brief Voltage conversion times enumerator for ADC resolution/averaging.
 */
typedef enum ina226_volt_conv_times_e {
    INA226_VOLT_CONV_TIME_140US     = (0b000),  /*!< 140 us voltage conversion time */
    INA226_VOLT_CONV_TIME_204US     = (0b001),  /*!< 204 us voltage conversion time */
    INA226_VOLT_CONV_TIME_332US     = (0b010),  /*!< 332 us voltage conversion time */
    INA226_VOLT_CONV_TIME_588US     = (0b011),  /*!< 588 us voltage conversion time */
    INA226_VOLT_CONV_TIME_1_1MS     = (0b100),  /*!< 1.1 ms voltage conversion time (default) */
    INA226_VOLT_CONV_TIME_2_116MS   = (0b101),  /*!< 116 ms voltage conversion time */
    INA226_VOLT_CONV_TIME_4_156MS   = (0b110),  /*!< 156 ms voltage conversion time */
    INA226_VOLT_CONV_TIME_8_244MS   = (0b111)   /*!< 244 ms voltage conversion time */
} ina226_volt_conv_times_t;

/**
 * @brief Current conversion times enumerator for ADC resolution/averaging.
 */
typedef enum ina226_operating_modes_e {
    INA226_OP_MODE_SHUTDOWN         = (0b000),  /*!< device is powered down */
    INA226_OP_MODE_TRIG_SHUNT_VOLT  = (0b001),  /*!< triggers single-shot conversion when set */
    INA226_OP_MODE_TRIG_BUS_VOLT    = (0b010),  /*!< triggers single-shot conversion when set */
    INA226_OP_MODE_TRIG_SHUNT_BUS   = (0b011),  /*!< triggers single-shot conversion when set */
    INA226_OP_MODE_SHUTDOWN2        = (0b100),  /*!< device is powered down */
    INA226_OP_MODE_CONT_SHUNT_VOLT  = (0b101),  /*!< normal operating mode */
    INA226_OP_MODE_CONT_BUS_VOLT    = (0b110),  /*!< normal operating mode */
    INA226_OP_MODE_CONT_SHUNT_BUS   = (0b111)   /*!< normal operating mode default */
} ina226_operating_modes_t;

/**
 * @brief All-register reset, shunt voltage and bus voltage ADC 
 * conversion times and averaging, operating mode.
 */
typedef union __attribute__((packed)) ina226_config_register_u {
    struct {
        ina226_operating_modes_t    operating_mode:3; 
        ina226_volt_conv_times_t    shun_volt_conv_time:3;
        ina226_volt_conv_times_t    bus_volt_conv_time:3;
        ina226_averaging_modes_t    averaging_mode:3;
        uint16_t                    reserved:3;        
        bool                        reset_enabled:1;         /*!< reset state BIT15 */
    } bits;                  /*!< represents the 16-bit control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit control register as `uint16_t` */
} ina226_config_register_t;

/**
 * @brief Alert configuration and Conversion Ready flag.
 */
typedef union __attribute__((packed)) ina226_mask_enable_register_u {
    struct {
        bool        alert_latch_enable:1;       /*!<  (bit:0) */
        bool        alert_polarity_bit:1;       /*!<  (bit:1) */
        bool        math_overflow_flag:1;       /*!<  (bit:2) */
        bool        conversion_ready_flag:1;    /*!<  (bit:3) */
        bool        alert_func_flag:1;          /*!<  (bit:4) */
        uint16_t    reserved:5;                 /*!<  (bit:5-9) */
        bool        conversion_ready:1;         /*!<  (bit:10) */
        bool        power_over_limit:1;         /*!<  (bit:11) */
        bool        bus_volt_under_volt:1;      /*!<  (bit:12) */  
        bool        bus_volt_over_volt:1;       /*!<  (bit:13) */
        bool        shunt_volt_under_volt:1;    /*!<  (bit:14) */  
        bool        shunt_volt_over_volt:1;     /*!<  (bit:15) */
    } bits;                  /*!< represents the 16-bit mask/enable register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit mask/enable register as `uint16_t` */
} ina226_mask_enable_register_t;


/**
 * @brief INA226 device configuration.
 */
typedef struct ina226_config_s {
    uint16_t                        i2c_address;                /*!< ina226 i2c device address */
    uint32_t                        i2c_clock_speed;            /*!< ina226 i2c device scl clock speed  */
    ina226_averaging_modes_t        averaging_mode;             /*!< ina226 averaging mode */
    ina226_volt_conv_times_t        shunt_voltage_conv_time;    /*!< ina226 shunt voltage conversion time */
    ina226_volt_conv_times_t        bus_voltage_conv_time;      /*!< ina226 bus voltage conversion time */
    ina226_operating_modes_t        operating_mode;             /*!< ina226 operating mode */
    float                           shunt_resistance;           /*!< ina226 shunt resistance, Ohm */
    //float                           shunt_voltage;              /*!< ina226 shunt voltage, V */
    float                           max_current;                /*!< ina226 maximum expected current, A */
} ina226_config_t;


/**
 * @brief INA226 context structure.
 */
struct ina226_context_t {
    ina226_config_t                 dev_config;       /*!< ina226 device configuration */
    i2c_master_dev_handle_t         i2c_handle;       /*!< ina226 I2C device handle */
    float                           current_lsb;      /*!< ina226 current LSB value, uA/bit, this is automatically configured */
};

/**
 * @brief INA226 context structure definition.
 */
typedef struct ina226_context_t ina226_context_t;

/**
 * @brief INA226 handle structure definition.
 */
typedef struct ina226_context_t *ina226_handle_t;

/**
 * @brief Reads the configuration register from the INA226.
 * 
 * @param handle INA226 device handle.
 * @param reg INA226 configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_get_configuration_register(ina226_handle_t handle, ina226_config_register_t *const reg);

/**
 * @brief Writes the configuration register to the INA226.
 * 
 * @param handle INA226 device handle.
 * @param reg INA226 configuration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_set_configuration_register(ina226_handle_t handle, const ina226_config_register_t reg);

/**
 * @brief Reads the calibration register from the INA226.
 * 
 * @param handle INA226 device handle.
 * @param reg INA226 calibration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_get_calibration_register(ina226_handle_t handle, uint16_t *const reg);

/**
 * @brief Writes the calibration register to the INA226.
 * 
 * @param handle INA226 device handle.
 * @param reg INA226 calibration register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_set_calibration_register(ina226_handle_t handle, const uint16_t reg);

/**
 * @brief Reads the mask/enable register from the INA226.
 * 
 * @param handle INA226 device handle.
 * @param reg INA226 mask/enable register
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_get_mask_enable_register(ina226_handle_t handle, ina226_mask_enable_register_t *const reg);

/**
 * @brief initializes an INA226 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] ina226_config INA226 device configuration.
 * @param[out] ina226_handle INA226 device handle.
 * @return ESP_OK on success.
 */
esp_err_t ina226_init(i2c_master_bus_handle_t master_handle, const ina226_config_t *ina226_config, ina226_handle_t *ina226_handle);

/**
 * @brief Calibrates the INA266.
 *
 * @param[in] handle INA226 device handle
 * @param[in] max_current Maximum expected current, A
 * @param[in] shunt_resistance Shunt resistance, Ohm
 * @return ESP_OK on success.
 */
esp_err_t ina226_calibrate(ina226_handle_t handle, const float max_current, const float shunt_resistance);

/**
 * @brief Reads bus voltage (V) from INA226.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] voltage INA226 bus voltage, V.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_bus_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Triggers and reads bus voltage (V) from INA226.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] voltage INA226 bus voltage, V.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_triggered_bus_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Reads shunt voltage (V) from INA226.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] voltage INA226 shunt voltage, V.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_shunt_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Triggers and reads shunt voltage (V) from INA226.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] voltage INA226 shunt voltage, V.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_triggered_shunt_voltage(ina226_handle_t handle, float *const voltage);

/**
 * @brief Reads current (A) from INA226.
 *
 * @note This function works properly only after calibration.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] current INA226 current, A.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_current(ina226_handle_t handle, float *const current);

/**
 * @brief Triggers and reads current (A) from INA226.
 *
 * @note This function works properly only after calibration.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] current INA226 current, A.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_triggered_current(ina226_handle_t handle, float *const current);

/**
 * @brief Reads power (W) from INA226.
 *
 * @note This function works properly only after calibration.
 *
 * @param[in] handle INA226 device handle.
 * @param[out] power INA226 power, W.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_power(ina226_handle_t handle, float *const power);

/**
 * @brief Reads operating mode from the INA226.
 *
 * @param[in] handle INA226 device handle
 * @param[out] mode Operating mode setting.
 * @return ESP_OK on success.
 */
esp_err_t ina226_get_operating_mode(ina226_handle_t handle, ina226_operating_modes_t *const mode);

/**
 * @brief Writes operating mode to the INA226.
 *
 * @param[in] handle INA226 device handle
 * @param[out] mode Operating mode setting.
 * @return ESP_OK on success.
 */
esp_err_t ina226_set_operating_mode(ina226_handle_t handle, const ina226_operating_modes_t mode);

esp_err_t ina226_get_averaging_mode(ina226_handle_t handle, ina226_averaging_modes_t *const mode);

esp_err_t ina226_set_averaging_mode(ina226_handle_t handle, const ina226_averaging_modes_t mode);

esp_err_t ina226_get_bus_volt_conv_time(ina226_handle_t handle, ina226_volt_conv_times_t *const conv_time);

esp_err_t ina226_set_bus_volt_conv_time(ina226_handle_t handle, ina226_volt_conv_times_t *const conv_time);

esp_err_t ina226_get_shunt_volt_conv_time(ina226_handle_t handle, ina226_volt_conv_times_t *const conv_time);

esp_err_t ina226_set_shunt_volt_conv_time(ina226_handle_t handle, ina226_volt_conv_times_t *const conv_time);

/**
 * @brief Resets the INA226.
 *
 * Same as power-on reset. Resets all registers to default values.
 * Calibration is conducted automatically after reset.
 *
 * @param[in] handle INA226 device handle
 * @return ESP_OK on success.
 */
esp_err_t ina226_reset(ina226_handle_t handle);

/**
 * @brief Removes an INA226 device from master bus.
 *
 * @param[in] handle INA226 device handle
 * @return ESP_OK on success.
 */
esp_err_t ina226_remove(ina226_handle_t handle);

/**
 * @brief Removes an INA226 device from master bus and frees handle.
 * 
 * @param[in] handle INA226 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t ina226_delete(ina226_handle_t handle);

/**
 * @brief Converts INA226 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* INA226 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* ina226_get_fw_version(void);

/**
 * @brief Converts INA226 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t INA226 firmware version number.
 */
int32_t ina226_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __INA226_H__
