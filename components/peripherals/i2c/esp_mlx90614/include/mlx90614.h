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
 * @file mlx90614.h
 * @defgroup drivers mlx90614
 * @{
 *
 * ESP-IDF driver for mlx90614 sensor
 * 
 * https://github.com/melexis/mlx90614-library
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MLX90614_H__
#define __MLX90614_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "mlx90614_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MLX90614 definitions
*/

#define I2C_MLX90614_DEV_CLK_SPD             UINT32_C(100000)  //!< mlx90614 I2C default clock frequency (100KHz)
#define I2C_MLX90614_DEV_ADDR                UINT8_C(0x5A)     //!< mlx90614 I2C address when ADDR pin floating/low


/*
 * macro definitions
*/
#define I2C_MLX90614_CONFIG_DEFAULT {                        \
        .i2c_address            = I2C_MLX90614_DEV_ADDR,     \
        .i2c_clock_speed        = I2C_MLX90614_DEV_CLK_SPD }

/*
 * SHT4X enumerator and structure declarations
*/

typedef enum mlx90614_sensor_iirs_e {
    MLX90614_SENSOR_IIR_100 = (0b100),
    MLX90614_SENSOR_IIR_80 = (0b101),
    MLX90614_SENSOR_IIR_67 = (0b110),
    MLX90614_SENSOR_IIR_57 = (0b111),
    MLX90614_SENSOR_IIR_50 = (0b000),
    MLX90614_SENSOR_IIR_25 = (0b001),
    MLX90614_SENSOR_IIR_17 = (0b010),
    MLX90614_SENSOR_IIR_13 = (0b011)
} mlx90614_sensor_iirs_t;

typedef enum mlx90614_sensor_test_repeat_states_e {
    MLX90614_SENSOR_TEST_REPEAT_OFF = 0,
    MLX90614_SENSOR_TEST_REPEAT_ON  = 1
} mlx90614_sensor_test_repeat_states_t;

typedef enum mlx90614_temperature_sensors_e {
    MLX90614_TEMPERATURE_SENSOR_TA_TOBJ1    = (0b00),
    MLX90614_TEMPERATURE_SENSOR_TA_TOBJ2    = (0b01),
    MLX90614_TEMPERATURE_SENSOR_TOBJ2       = (0b10),
    MLX90614_TEMPERATURE_SENSOR_TOBJ1_TOBJ2 = (0b11)
} mlx90614_temperature_sensors_t;

typedef enum mlx90614_sensor_ir_types_e {
    MLX90614_SENSOR_IR_TYPE_SINGLE = 0,
    MLX90614_SENSOR_IR_TYPE_DUAL   = 1
} mlx90614_sensor_ir_types_t;

typedef enum mlx90614_k_signs_e {
    MLX90614_K_SIGN_POSITIVE = 0,
    MLX90614_K_SIGN_NEGATIVE = 1
} mlx90614_k_signs_t;

typedef enum mlx90614_fir_values_e {
    MLX90614_FIR_128    = (0b100),
    MLX90614_FIR_256    = (0b101),
    MLX90614_FIR_512    = (0b110),
    MLX90614_FIR_1024   = (0b111)
} mlx90614_fir_values_t;

typedef enum ml90614_gains_e {
    MLX90614_GAIN_1     = (0b000),
    MLX90614_GAIN_3     = (0b001),
    MLX90614_GAIN_6     = (0b010),
    MLX90614_GAIN_12_5  = (0b011),
    MLX90614_GAIN_25    = (0b100),
    MLX90614_GAIN_50    = (0b101),
    MLX90614_GAIN_100A  = (0b110),
    MLX90614_GAIN_100B  = (0b111)
} ml90614_gains_t;

typedef enum mlx90614_nk2_signs_e {
    MLX90614_KT2_SIGN_POSITIVE = 0,
    MLX90614_KT2_SIGN_NEGATIVE = 1
} mlx90614_nk2_signs_t;

typedef enum mlx90614_sensor_test_states_e {
    MLX90614_SENSOR_TEST_ENABLED  = 0,
    MLX90614_SENSOR_TEST_DISABLED = 1
} mlx90614_sensor_test_states_t;


typedef enum mlx90614_pwm_modes_e {
    MLX90614_PWM_MODE_EXTENDED = 0,
    MLX90614_PWM_MODE_SINGLE   = 1
} mlx90614_pwm_modes_t;

typedef enum mlx90614_pwm_mode_states_e {
    MLX90614_PWM_MODE_STATE_DISABLED = 0,
    MLX90614_PWM_MODE_STATE_ENABLED  = 1
} mlx90614_pwm_mode_states_t;

typedef enum mlx90614_sda_pin_modes_e {
    MLX90614_SDA_PIN_MODE_OPEN_DRAIN  = 0,
    MLX90614_SDA_PIN_MODE_PUSH_PULL   = 1
} mlx90614_sda_pin_modes_t;

typedef enum mlx90614_thermal_modes_e {
    MLX90614_THERMAL_MODE_PWM           = 0,
    MLX90614_THERMAL_MODE_THERMAL_RELAY = 1
} mlx90614_thermal_modes_t;

/**
 * @brief MLX90614 configuration structure definition.
 */
typedef struct mlx90614_config_s {
    uint16_t                    i2c_address;        /*!< mlx90614 i2c device address */
    uint32_t                    i2c_clock_speed;    /*!< mlx90614 i2c device scl clock speed in hz */
} mlx90614_config_t;

/**
 * @brief MLX90614 opaque handle structure definition.
 */
typedef void* mlx90614_handle_t;




/**
 * @brief Initializes an MLX90614 device onto the master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] mlx90614_config MLX90614 device configuration.
 * @param[out] mlx90614_handle MLX90614 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_init(i2c_master_bus_handle_t master_handle, const mlx90614_config_t *mlx90614_config, mlx90614_handle_t *mlx90614_handle);

/**
 * @brief Reads all three temperatures (ambient, object 1 and object 2) from the MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param[out] ambient_temperature Ambient temperature in degrees celsius.
 * @param[out] object1_temperature Object 1 temperature in degrees celsius.
 * @param[out] object2_temperature Object 2 temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_temperatures(mlx90614_handle_t handle, float *const ambient_temperature, float *const object1_temperature, float *const object2_temperature);

/**
 * @brief Reads the ambient temperature from MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param[out] ambient_temperature Ambient temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_ambient_temperature(mlx90614_handle_t handle, float *const ambient_temperature);

/**
 * @brief Reads object 1 temperature from mlx90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param[out] object1_temperature Object 1 temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_object1_temperature(mlx90614_handle_t handle, float *const object1_temperature);

/**
 * @brief Reads object 2 temperature from MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param[out] object2_temperature Object 2 temperature in degrees celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_object2_temperature(mlx90614_handle_t handle, float *const object2_temperature);

/**
 * @brief Reads IR channel 1 from MLX90614.
 * 
 * @param handle MLX90614 device handle.
 * @param ir_channel1 IR channel 1.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_ir_channel1(mlx90614_handle_t handle, int16_t *const ir_channel1);

/**
 * @brief Reads IR channel 2 from MLX90614.
 * 
 * @param handle MLX90614 device handle.
 * @param ir_channel1 IR channel 2.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_ir_channel2(mlx90614_handle_t handle, int16_t *const ir_channel2);

/**
 * @brief Reads ambient temperature range from MLX90614.
 * 
 * @param handle MLX90614 device handle.
 * @param ambient_temperature_range Ambient temperature range.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_ambient_temperature_range(mlx90614_handle_t handle, float *const ambient_temperature_range);

/**
 * @brief Reads emissivity coefficient (0.1 to 1.0) setting from MLX90614.
 * 
 * @param handle MLX90614 device handle.
 * @param emissivity MLX90614 emissivity coefficient (0.1 to 1.0) setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_emissivity(mlx90614_handle_t handle, float *const coefficient);

/**
 * @brief Writes emissivity coefficient (0.1 to 1.0) setting to MLX90614.
 * 
 * @param handle MLX90614 device handle.
 * @param emissivity MLX90614 emissivity coefficient (0.1 to 1.0) setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_set_emissivity(mlx90614_handle_t handle, const float coefficient);

/**
 * @brief Reads maximum object temperature setting from MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param temperature MLX90614 maximum object temperature setting in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_object_maximum_temperature(mlx90614_handle_t handle, float *const temperature);

/**
 * @brief Writes maximum object temperature setting to MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param temperature MLX90614 maximum object temperature setting in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_set_object_maximum_temperature(mlx90614_handle_t handle, const float temperature);

/**
 * @brief Reads minimum object temperature setting from MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param temperature MLX90614 minimum object temperature setting in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_object_minimum_temperature(mlx90614_handle_t handle, float *const temperature);

/**
 * @brief Writes minimum object temperature setting to MLX90614.
 *
 * @param[in] handle MLX90614 device handle.
 * @param temperature MLX90614 minimum object temperature setting in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_set_object_minimum_temperature(mlx90614_handle_t handle, const float temperature);

/**
 * @brief Reads I2C address setting from MLX90614.
 * 
 * @param handle MLX90614 device handle.
 * @param address MLX90614 I2C address setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_get_address(mlx90614_handle_t handle, uint8_t *const address);

/**
 * @brief Writes I2C address setting to MLX90614.
 * 
 * @note MLX90614 device handle must be reinitialized when I2C address is changed.
 * 
 * @param handle MLX90614 device handle.
 * @param address MLX90614 I2C address setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_set_address(mlx90614_handle_t handle, const uint8_t address);

/**
 * @brief Puts the MLX90614 into sleep mode.
 *
 * @param[in] handle MLX90614 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_sleep(mlx90614_handle_t handle);

/**
 * @brief Wakes-up the MLX90614 from sleep mode.
 *
 * @param[in] handle MLX90614 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_wakeup(mlx90614_handle_t handle);

/**
 * @brief Removes an MLX90614 device from master bus.
 *
 * @param[in] handle MLX90614 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_remove(mlx90614_handle_t handle);

/**
 * @brief Removes an MLX90614 device from master bus and frees handle.
 *
 * @param[in] handle MLX90614 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mlx90614_delete(mlx90614_handle_t handle);

/**
 * @brief Converts MLX90614 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* MLX90614 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* mlx90614_get_fw_version(void);

/**
 * @brief Converts MLX90614 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t MLX90614 firmware version number.
 */
int32_t mlx90614_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MLX90614_H__
