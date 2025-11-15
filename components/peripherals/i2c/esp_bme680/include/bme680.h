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
 * @file bme680.h
 * @defgroup drivers bme680
 * @{
 * 
 * https://github.com/boschsensortec/BME68x_SensorAPI/blob/master/bme68x.c
 *
 * ESP-IDF driver for bme680 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __BME680_H__
#define __BME680_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "bme680_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * BME680 definitions
*/
#define I2C_BME680_DEV_CLK_SPD      UINT32_C(100000)   //!< bme680 I2C default clock frequency (100KHz)

/*
 * supported device addresses
*/
#define I2C_BME680_DEV_ADDR_LO      UINT8_C(0x76) //!< bme680 I2C address when ADDR pin floating/low
#define I2C_BME680_DEV_ADDR_HI      UINT8_C(0x77) //!< bme680 I2C address when ADDR pin high

#define BME680_HEATER_TEMP_MIN      UINT8_C(200)  // min. 200 degree Celsius
#define BME680_HEATER_TEMP_MAX      UINT8_C(400)  // max. 200 degree Celsius
#define BME680_HEATER_PROFILE_SIZE  UINT8_C(10)   // max. 10 heater profiles (0..9)

/*
 * BME680 macros
*/
#define BME680_CONFIG_DEFAULT {                                             \
        .i2c_address                = I2C_BME680_DEV_ADDR_HI,                   \
        .i2c_clock_speed            = I2C_BME680_DEV_CLK_SPD,                   \
        .power_mode                 = BME680_POWER_MODE_FORCED,                 \
        .iir_filter                 = BME680_IIR_FILTER_OFF,                    \
        .pressure_oversampling      = BME680_PRESSURE_OVERSAMPLING_4X,          \
        .temperature_oversampling   = BME680_TEMPERATURE_OVERSAMPLING_4X,       \
        .humidity_oversampling      = BME680_HUMIDITY_OVERSAMPLING_4X,          \
        .gas_enabled                = true,                                     \
        .heater_temperature         = 300,                                      \
        .heater_duration            = 300,                                      \
        .heater_profile_size        = 1                                         \
    }

#define BME680_FORCED_CONFIG_DEFAULT {                                              \
        .i2c_address                = I2C_BME680_DEV_ADDR_HI,                               \
        .i2c_clock_speed            = I2C_BME680_DEV_CLK_SPD,                               \
        .power_mode                 = BME680_POWER_MODE_FORCED,                             \
        .iir_filter                 = BME680_IIR_FILTER_3,                                  \
        .standby_time               = BME680_STANDBY_TIME_500MS,                            \
        .pressure_oversampling      = BME680_PRESSURE_OVERSAMPLING_8X,                      \
        .temperature_oversampling   = BME680_TEMPERATURE_OVERSAMPLING_8X,                   \
        .humidity_oversampling      = BME680_HUMIDITY_OVERSAMPLING_8X,                      \
        .gas_enabled                = true,                                                 \
        .heater_temperature_profile = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 }, \
        .heater_duration_profile    = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }, \
        .heater_profile_size        = 10                                                    \
    }

#define BME680_SEQUENTIAL_CONFIG_DEFAULT {                                              \
        .i2c_address                = I2C_BME680_DEV_ADDR_HI,                               \
        .i2c_clock_speed            = I2C_BME680_DEV_CLK_SPD,                               \
        .power_mode                 = BME680_POWER_MODE_SEQUENTIAL,                         \
        .iir_filter                 = BME680_IIR_FILTER_3,                                  \
        .standby_time               = BME680_STANDBY_TIME_500MS,                            \
        .pressure_oversampling      = BME680_PRESSURE_OVERSAMPLING_8X,                      \
        .temperature_oversampling   = BME680_TEMPERATURE_OVERSAMPLING_8X,                   \
        .humidity_oversampling      = BME680_HUMIDITY_OVERSAMPLING_8X,                      \
        .gas_enabled                = true,                                                 \
        .heater_temperature_profile = { 200, 240, 280, 320, 360, 360, 320, 280, 240, 200 }, \
        .heater_duration_profile    = { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 }, \
        .heater_profile_size        = 10                                                    \
    }

/*
 * BME680 enumerator and structure declarations
*/

/**
 * @brief BME680 stand-by times (ODR) enumerator definition.
 * Standby time between sequential mode measurement profiles.
 */
typedef enum bme680_standby_times_e {
    BME680_STANDBY_TIME_0_5MS  = 0,  //!< stand by time 0.5ms
    BME680_STANDBY_TIME_62_5MS = 1,  //!< stand by time 62.5ms
    BME680_STANDBY_TIME_125MS  = 2,  //!< stand by time 125ms
    BME680_STANDBY_TIME_250MS  = 3,  //!< stand by time 250ms
    BME680_STANDBY_TIME_500MS  = 4,  //!< stand by time 500ms
    BME680_STANDBY_TIME_1000MS = 5,  //!< stand by time 1s
    BME680_STANDBY_TIME_10MS   = 6,  //!< stand by time 10ms
    BME680_STANDBY_TIME_20MS   = 7,  //!< stand by time 20ms
    BME680_STANDBY_TIME_NONE   = 8   //!< stand by time none
} bme680_standby_times_t;


/**
 * @brief BME680 heater set-points enumerator.
 */
typedef enum bme680_heater_setpoints_e {
    BME680_HEATER_SETPOINT_0  = (0b0000),
    BME680_HEATER_SETPOINT_1  = (0b0001),
    BME680_HEATER_SETPOINT_2  = (0b0010),
    BME680_HEATER_SETPOINT_3  = (0b0011),
    BME680_HEATER_SETPOINT_4  = (0b0100),
    BME680_HEATER_SETPOINT_5  = (0b0101),
    BME680_HEATER_SETPOINT_6  = (0b0110),
    BME680_HEATER_SETPOINT_7  = (0b0111),
    BME680_HEATER_SETPOINT_8  = (0b1000),
    BME680_HEATER_SETPOINT_9  = (0b1001)
} bme680_heater_setpoints_t;

/**
 * @brief BME680 gas wait multipliers enumerator.
 */
typedef enum bme680_gas_wait_multipliers_e {
    BME680_GAS_WAIT_MULT_1  = (0b00),
    BME680_GAS_WAIT_MULT_4  = (0b01),
    BME680_GAS_WAIT_MULT_16 = (0b10),
    BME680_GAS_WAIT_MULT_64 = (0b11)
} bme680_gas_wait_multipliers_t;

/**
 * @brief BME680 IIR filters coefficient enumerator.
 */
typedef enum bme680_iir_filters_e {
    BME680_IIR_FILTER_OFF = (0b000),
    BME680_IIR_FILTER_1   = (0b001),
    BME680_IIR_FILTER_3   = (0b010),
    BME680_IIR_FILTER_7   = (0b011),
    BME680_IIR_FILTER_15  = (0b100),
    BME680_IIR_FILTER_31  = (0b101),
    BME680_IIR_FILTER_63  = (0b110),
    BME680_IIR_FILTER_127 = (0b111),
} bme680_iir_filters_t;

/**
 * @brief BME680 power modes enumerator.
 */
typedef enum bme680_power_modes_e {
    BME680_POWER_MODE_SLEEP      = (0b00), //!< sleep mode, default after power-up
    BME680_POWER_MODE_FORCED     = (0b01), //!< measurement is initiated by user
    BME680_POWER_MODE_PARALLEL   = (0b10), //!< parallel mode, gas and TPH measurements are done in parallel
    BME680_POWER_MODE_SEQUENTIAL = (0b11)  //!< sequential mode, gas and TPH measurements are done in sequence
} bme680_power_modes_t;

/**
 * @brief BME680 pressure oversampling enumerator.
 */
typedef enum bme680_pressure_oversampling_e {
    BME680_PRESSURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BME680_PRESSURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    BME680_PRESSURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    BME680_PRESSURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    BME680_PRESSURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    BME680_PRESSURE_OVERSAMPLING_16X        = (0b101)   //!< ultra high resolution
} bme680_pressure_oversampling_t;

/**
 * @brief BME680 temperature oversampling enumerator.
 */
typedef enum bme680_temperature_oversampling_e {
    BME680_TEMPERATURE_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BME680_TEMPERATURE_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    BME680_TEMPERATURE_OVERSAMPLING_2X         = (0b010),  //!< low power
    BME680_TEMPERATURE_OVERSAMPLING_4X         = (0b011),  //!< standard
    BME680_TEMPERATURE_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    BME680_TEMPERATURE_OVERSAMPLING_16X        = (0b101),  //!< ultra high resolution
} bme680_temperature_oversampling_t;

/**
 * @brief BME680 humidity oversampling enumerator.
 */
typedef enum bme680_humidity_oversampling_e {
    BME680_HUMIDITY_OVERSAMPLING_SKIPPED    = (0b000),  //!< skipped, no measurement, output set to 0x80000
    BME680_HUMIDITY_OVERSAMPLING_1X         = (0b001),  //!< ultra low power
    BME680_HUMIDITY_OVERSAMPLING_2X         = (0b010),  //!< low power
    BME680_HUMIDITY_OVERSAMPLING_4X         = (0b011),  //!< standard
    BME680_HUMIDITY_OVERSAMPLING_8X         = (0b100),  //!< high resolution
    BME680_HUMIDITY_OVERSAMPLING_16X        = (0b101),  //!< ultra high resolution
} bme680_humidity_oversampling_t;

/**
 * @brief BME680 status 0 register (0x1d) structure.  The reset state is 0x00 for this register.
 * This register contains the gas measurement index, measuring status, gas measuring status, and
 * new data status.
 */
typedef union __attribute__((packed)) bme680_status0_register_u {
    struct {
        uint8_t gas_measurement_index:4; /*!< bme680 user can program sequence of up to 10 conversions by setting nb_conv<3:0> (bit:0-3) */
        uint8_t reserved:1;              /*!< reserved (bit:4) */
        bool    measuring:1;             /*!< bmp680 automatically set to 1 whenever a conversion is running and back to 0 when results transferred to data registers (bit:5) */
        bool    gas_measuring:1;         /*!< bme680 automatically set to 1 during gas measurement and back to 0 when results transferred to data registers (bit:6) */
        bool    new_data:1;              /*!< bme680 measured data are stored into the data registers when true (bit:7) */
    } bits;
    uint8_t reg;
} bme680_status0_register_t;

/**
 * @brief BME680 control measurement register (0x74) structure.  The reset state is 0x00 for this register.
 * This register contains the power mode, pressure and temperature oversampling settings.
 */
typedef union __attribute__((packed)) bme680_control_measurement_register_u {
    struct {
        bme680_power_modes_t                power_mode:2;               /*!< bme680 power mode of the device            (bit:0-1)  */
        bme680_pressure_oversampling_t      pressure_oversampling:3;    /*!< bme680 oversampling of pressure data       (bit:2-4) */
        bme680_temperature_oversampling_t   temperature_oversampling:3; /*!< bme680 oversampling of temperature data    (bit:5-7) */
    } bits;
    uint8_t reg;
} bme680_control_measurement_register_t;

/**
 * @brief BME680 control measurement register (0x72) structure.  The reset state is 0x00 for this register.
 * This register contains humidity oversampling and SPI interrupt settings.
 */
typedef union __attribute__((packed)) bme680_control_humidity_register_u {
    struct {
        bme680_humidity_oversampling_t      humidity_oversampling:3;    /*!< bme680 oversampling of humidity data          (bit:0-2)  */
        uint8_t                             reserved1:3;                /*!< bme680 reserved                               (bit:3-5) */
        bool                                spi_irq_enabled:1;          /*!< bme680 3-wire SPI interrupt enabled when true (bit:6)  */
        uint8_t                             reserved2:1;                /*!< bme680 reserved                               (bit:7) */
    } bits;
    uint8_t reg;
} bme680_control_humidity_register_t;

/**
 * @brief BME680 control gas 0 register (0x71) structure.  The reset state is 0x00 for this register.
 * This register contains the heater set-point and gas conversion settings.
 */
typedef union __attribute__((packed)) bme680_control_gas1_register_u {
    struct {
        bme680_heater_setpoints_t   heater_setpoint:4;        /*!< bme680           (bit:0-3)  */
        bool                        gas_conversion_enabled:1; /*!< bme680 gas conversions are started only appropriate mode when true (bit:4)  */
        uint8_t                     reserved:2;               /*!< bme680 reserved                               (bit:5-6) */
        uint8_t                     standby_period_msb:1;     /*!< bme680 standby time MSB (ODR)                 (bit:7) */
    } bits;
    uint8_t reg;
} bme680_control_gas1_register_t;

/**
 * @brief BME680 control gas 1 register (0x70) structure.  The reset state is 0x00 for this register.
 * This register contains heater setting.
 */
typedef union __attribute__((packed)) bme680_control_gas0_register_u {
    struct {
        uint8_t                 reserved1:3;    /*!< bme680           (bit:0-2)  */
        bool                    heater_disabled:1; /*!< bme680 heater is off when true (bit:3)  */
        uint8_t                 reserved2:4;    /*!< bme680 reserved   (bit:4-7) */
    } bits;
    uint8_t reg;
} bme680_control_gas0_register_t;

/**
 * @brief BME680 control gas_r_lsb register (0x2b) structure.  The reset state is 0x00 for this register.
 * This register contains the gas resistance lsb, gas validation flag, heater stab, and gas range settings.
 */
typedef union __attribute__((packed)) bme680_gas_r_lsb_register_u {
    struct {
        uint8_t             gas_range:4;        /*!< bme680 adc range of measure gas sensor resistance (bit:0-3)  */
        bool                heater_stable:1;    /*!< bme680 heater temperature stability for target heater resistance (bit:4)  */
        bool                gas_valid:1;        /*!< bme680 gas conversion is valid when true (bit:5)  */
        uint8_t             gas_lsb:2;          /*!< bme680 gas resistance lsb  (bit:6-7) */
    } bits;
    uint8_t reg;
} bme680_gas_lsb_register_t;

/**
 * @brief BME680 configuration register (0x75) structure.  The reset state is 0x00 for this register.
 * This register contains SPI and IIR filter settings.
 */
typedef union __attribute__((packed)) bme680_configuration_register_u {
    struct {
        bool                    spi_enabled:1;  /*!< bme680 3-wire SPI interface enabled when true  (bit:0)  */
        uint8_t                 reserved1:1;    /*!< bme680 reserved                                (bit:1) */
        bme680_iir_filters_t    iir_filter:3;   /*!< bme680 time constant of the IIR filter         (bit:2-4) */
        uint8_t                 standby_period_lsb:3; /*!< bme680 standby time LSB (ODR)             (bit:5-7) */
    } bits;
    uint8_t reg;
} bme680_config_register_t;

/**
 * @brief BME680 calibration factors structure definition.
 */
typedef struct bme680_cal_factors_s {
    /* temperature compensation */
    uint16_t                par_T1;
    int16_t                 par_T2;
    int8_t                  par_T3;
    float                   temperature_fine;
    /* humidity compensation */
    uint16_t                par_H1;
    uint16_t                par_H2;
    int8_t                  par_H3;
    int8_t                  par_H4;
    int8_t                  par_H5;
    uint8_t                 par_H6;
    int8_t                  par_H7;
    /* pressure compensation */
    uint16_t                par_P1;
    int16_t                 par_P2;
    int8_t                  par_P3;
    int16_t                 par_P4;
    int16_t                 par_P5;
    int8_t                  par_P6;
    int8_t                  par_P7;
    int16_t                 par_P8;
    int16_t                 par_P9;
    uint8_t                 par_P10;
    /* resistance heat compensation */
    int8_t                  par_G1;
    int16_t                 par_G2;
    int8_t                  par_G3;
    /* other */
    uint8_t                 res_heat_range;
    int8_t                  res_heat_val;
    int8_t                  range_switching_error;
} bme680_cal_factors_t;

/**
 * @brief BME680 data structure definition.
 */
typedef struct bme680_data_s {
    float    dewpoint_temperature;  /*!< dew-point temperature in degrees celsius */
    float    air_temperature;       /*!< air temperature in degrees celsius */
    float    relative_humidity;     /*!< relative humidity in percent */
    float    barometric_pressure;   /*!< barometric pressure in hecto-pascal */
    bool     gas_valid;             /*!< indicates that the gas measurement results are valid  */
    float    gas_resistance;        /*!< gas resistance in ohms */
    uint8_t  gas_range;             /*!< gas resistance range */
    uint8_t  gas_index;             /*!< heater profile used (0..9) */
    bool     heater_stable;         /*!< indicates that the heater temperature was stable */
    uint16_t iaq_score;             /*!< air quality index (0..500) */
    float    temperature_score;
    float    humidity_score;
    float    gas_score;
} bme680_data_t;

/**
 * @brief BME680 ADC data structure definition.
 */
typedef struct bme680_adc_data_s {
    uint32_t temperature;
    uint16_t humidity;
    uint32_t pressure;
    uint16_t gas;
    bool     gas_valid;
    uint8_t  gas_range;
    uint8_t  gas_index;
    bool     heater_stable;
} bme680_adc_data_t;

/**
 * @brief BME680 configuration structure definition.
 */
typedef struct bme680_config_s {
    uint16_t                                i2c_address;                    /*!< bme680 i2c device address */
    uint32_t                                i2c_clock_speed;                /*!< bme680 i2c device scl clock speed  */
    bme680_power_modes_t                    power_mode;                     /*!< bme680 power mode */
    bme680_iir_filters_t                    iir_filter;
    bme680_standby_times_t                  standby_time;
    bme680_pressure_oversampling_t          pressure_oversampling;
    bme680_temperature_oversampling_t       temperature_oversampling;
    bme680_humidity_oversampling_t          humidity_oversampling;
    bool                                    gas_enabled;                    /*!< bme680 gas measurements enabled when true */
    uint16_t                                heater_temperature;             /*!< bme680 heater temperature for forced mode in degrees celsius */
    uint16_t                                heater_duration;                /*!< bme680 heating duration for forced mode in milliseconds */
    uint16_t                                heater_temperature_profile[BME680_HEATER_PROFILE_SIZE]; /*!< bme680 heater temperature profile in degrees celsius */
    uint16_t                                heater_duration_profile[BME680_HEATER_PROFILE_SIZE];    /*!< bme680 heating duration profile in milliseconds */
    uint8_t                                 heater_profile_size;            /*!< bme680 size of the heating profile */
    uint16_t                                heater_shared_duration;         /*!< bme680 heating duration for parallel mode in milliseconds */
} bme680_config_t;


/**
 * @brief BME680 opaque handle structure definition.
 */
typedef void* bme680_handle_t;



/**
 * @brief Initializes an BME680 device onto the master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] bme680_config BME680 device configuration.
 * @param[out] bme680_handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_init(i2c_master_bus_handle_t master_handle, const bme680_config_t *bme680_config, bme680_handle_t *bme680_handle);

/**
 * @brief Reads humidity, temperature, and pressure ADC signals from BME680
 *
 * @param[in] handle BME680 device handle.
 * @param[out] data BME680 ADC data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_adc_signals(bme680_handle_t handle, bme680_adc_data_t *const data);

/**
 * @brief Reads humidity, temperature, pressure, and gas ADC signals from BME680
 * using a specific heater profile.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] profile_index Heater profile index (0..9).
 * @param[out] data BME680 ADC data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_adc_signals_by_heater_profile(bme680_handle_t handle, uint8_t profile_index, bme680_adc_data_t *const data);

/**
 * @brief Reads humidity, temperature, and pressure measurements from BME680.
 *
 * @param[in] handle BME680 device handle.
 * @param[out] data BME680 data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_data(bme680_handle_t handle, bme680_data_t *const data);

/**
 * @brief Reads humidity, temperature, pressure, and gas measurements from BME680
 * using a specific heater profile.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] profile_index Heater profile index (0..9).
 * @param[out] data BME680 data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_data_by_heater_profile(bme680_handle_t handle, const uint8_t profile_index, bme680_data_t *const data);

/**
 * @brief Reads data status of the BME680.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] ready Data is ready when asserted to true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_data_status(bme680_handle_t handle, bool *const ready);

/**
 * @brief Reads gas measurement index from the BME680.
 * 
 * @param handle BME680 device handle.
 * @param index Gas measurement index (0..9).
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_gas_measurement_index(bme680_handle_t handle, uint8_t *const index);

/**
 * @brief Reads power mode setting from the BME680.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] power_mode BME680 power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_power_mode(bme680_handle_t handle, bme680_power_modes_t *const power_mode);

/**
 * @brief Writes power mode setting to BME680.  See datasheet, section 3.6, table 10.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] power_mode BME680 power mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_set_power_mode(bme680_handle_t handle, const bme680_power_modes_t power_mode);

/**
 * @brief Reads pressure oversampling setting from BME680.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] oversampling BME680 pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_pressure_oversampling(bme680_handle_t handle, bme680_pressure_oversampling_t *const oversampling);

/**
 * @brief Writes pressure oversampling setting to BME680.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] oversampling BME680 pressure oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_set_pressure_oversampling(bme680_handle_t handle, const bme680_pressure_oversampling_t oversampling);

/**
 * @brief Reads temperature oversampling setting from BME680.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] oversampling BME680 temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_temperature_oversampling(bme680_handle_t handle, bme680_temperature_oversampling_t *const oversampling);

/**
 * @brief Writes temperature oversampling setting to BME680.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] oversampling BME680 temperature oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_set_temperature_oversampling(bme680_handle_t handle, const bme680_temperature_oversampling_t oversampling);

/**
 * @brief Reads humidity oversampling setting from BME680.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] oversampling BME680 humidity oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_humidity_oversampling(bme680_handle_t handle, bme680_humidity_oversampling_t *const oversampling);

/**
 * @brief Writes humidity oversampling setting to BME680.  See datasheet, section 3.3.1, table 4.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] oversampling BME680 humidity oversampling setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_set_humidity_oversampling(bme680_handle_t handle, const bme680_humidity_oversampling_t oversampling);

/**
 * @brief Reads IIR filter setting from BME680.
 * 
 * @param[in] handle BME680 device handle.
 * @param[out] iir_filter BME680 IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_iir_filter(bme680_handle_t handle, bme680_iir_filters_t *const iir_filter);

/**
 * @brief Writes IIR filter setting to BME680.  See datasheet, section 3.4, table 7.
 * 
 * @param[in] handle BME680 device handle.
 * @param[in] iir_filter BME680 IIR filter setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_set_iir_filter(bme680_handle_t handle, const bme680_iir_filters_t iir_filter);

/**
 * @brief Reads stand-by time (ODR) setting from BME680.
 * 
 * @param handle BME680 device handle.
 * @param standby_time BME680 stand-by time (ODR) setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_get_standby_time(bme680_handle_t handle, bme680_standby_times_t *const standby_time);

/**
 * @brief Writes stand-by time (ODR) setting to BME680.
 * 
 * @param handle BME680 device handle.
 * @param standby_time BME680 stand-by time (ODR) setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_set_standby_time(bme680_handle_t handle, const bme680_standby_times_t standby_time);

/**
 * @brief Issues soft-reset sensor and initializes registers for BME680.
 *
 * @param[in] handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_reset(bme680_handle_t handle);

/**
 * @brief Air quality as a string based on the IAQ score.
 * 
 * @param iaq_score Index of air quality score.
 * @return char* Air quality as a string.
 */
char *bme680_air_quality_to_string(float iaq_score);

/**
 * @brief Removes an BME680 device from master bus.
 *
 * @param[in] handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_remove(bme680_handle_t handle);

/**
 * @brief Removes an BME680 device from master bus and frees handle.
 *
 * @param[in] handle BME680 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t bme680_delete(bme680_handle_t handle);

/**
 * @brief Converts BME680 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* BME680 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* bme680_get_fw_version(void);

/**
 * @brief Converts BME680 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t BME680 firmware version number.
 */
int32_t bme680_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __BME680_H__
