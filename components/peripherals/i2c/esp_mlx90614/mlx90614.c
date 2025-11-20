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
 * @file mlx90614.c
 *
 * ESP-IDF driver for MLX90614 IR sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/mlx90614.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * MLX90614 definitions
*/

#define MLX90614_CMD_RAM_READ_AMB        UINT8_C(0x03)
#define MLX90614_CMD_RAM_READ_RAWIR1     UINT8_C(0x04)
#define MLX90614_CMD_RAM_READ_RAWIR2     UINT8_C(0x05)
#define MLX90614_CMD_RAM_READ_TA         UINT8_C(0x06)
#define MLX90614_CMD_RAM_READ_TOBJ1      UINT8_C(0x07)
#define MLX90614_CMD_RAM_READ_TOBJ2      UINT8_C(0x08)

#define MLX90614_CMD_EEPROM_RDWR_TOMAX   UINT8_C(0x20)
#define MLX90614_CMD_EEPROM_RDWR_TOMIN   UINT8_C(0x21)
#define MLX90614_CMD_EEPROM_RDWR_PWMCTRL UINT8_C(0x22)
#define MLX90614_CMD_EEPROM_RDWR_TARANGE UINT8_C(0x23)
#define MLX90614_CMD_EEPROM_RDWR_EMISS   UINT8_C(0x24)
#define MLX90614_CMD_EEPROM_RDWR_CFGREG  UINT8_C(0x25)
#define MLX90614_CMD_EEPROM_RDWR_SMBADDR UINT8_C(0x2E)
#define MLX90614_CMD_EEPROM_RD_IDNUM1    UINT8_C(0x3C)
#define MLX90614_CMD_EEPROM_RD_IDNUM2    UINT8_C(0x3D)
#define MLX90614_CMD_EEPROM_RD_IDNUM3    UINT8_C(0x3E)
#define MLX90614_CMD_EEPROM_RD_IDNUM4    UINT8_C(0x3F)

#define MLX90614_CMD_READ_FLAGS_REG      UINT8_C(0xF0)
#define MLX90614_CMD_SLEEP               UINT8_C(0xFF)

#define MLX90614_CMD_EEPROM_CLR_CELL     UINT8_C(0x00)

#define MLX90614_CRC8_POLYNOM            UINT8_C(7)        //!< mlx90614 I2C CRC8 polynomial

#define MLX90614_POWERUP_DELAY_MS        UINT16_C(10)
#define MLX90614_APPSTART_DELAY_MS       UINT16_C(10)
#define MLX90614_CMD_DELAY_MS            UINT16_C(5)
#define MLX90614_EEPROM_RDWR_DELAY_MS    UINT16_C(10)
#define MLX90614_TX_RX_DELAY_MS          UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/**
 * @brief MLX90614 configuration register structure.
 */
typedef union __attribute__((packed)) mlx90614_config_register_u {
    struct {
        mlx90614_sensor_iirs_t                  iir:3;                  /*!< iir setting 						(bit:0-2) */
        mlx90614_sensor_test_repeat_states_t    test_repeat_state:1;    /*!< repeat sensor test state 			(bit:3) */
        mlx90614_temperature_sensors_t          t_sensors:2;            /*!< temperature sensor configuration 	(bit:4-5) */
        mlx90614_sensor_ir_types_t              ir_type:1;              /*!< ir sensor type 					(bit:6) */
        mlx90614_k_signs_t                      k_sign:1;               /*!< positive or negative signs of k 	(bit:7) */
        mlx90614_sensor_firs_t                  fir:3;                  /*!< fir setting 						(bit:8-10) */
        mlx90614_sensor_gains_t                 gain:3;                 /*!< gain setting 						(bit:11-13) */
        mlx90614_k2_signs_t                     k2_sign:1;             /*!< positive or negative signs of kt2 	(bit:14) */
        mlx90614_sensor_test_states_t           test_state:1;           /*!< sensor test state 					(bit:15) */
    } bit;                          /*!< represents the 16-bit config register parts in bits. */
    uint16_t reg;                   /*!< represents the 16-bit config register as `uint16_t` */
} mlx90614_config_register_t;

/**
 * @brief MLX90614 PWM control register structure.
 */
typedef union __attribute__((packed)) mlx90614_pwmctrl_register_u {
    struct {
        mlx90614_pwm_modes_t        pwm_mode:1;         /*!< PWM mode       (bit:0) */ 
        mlx90614_pwm_mode_states_t  pwm_mode_state:1;   /*!< PWM mode state (bit:1) */
        mlx90614_sda_pin_modes_t    sda_pin_mode:1;     /*!< SDA pin mode   (bit:2) */
        mlx90614_thermal_modes_t    thermal_mode:1;     /*!< thermal mode   (bit:3) */
        uint16_t                    pwm_repetition:5;   /*!< PWM repetition number 0...62 step of 2 (bit:4-8)*/
        uint16_t                    pwm_period_mult:7;  /*!< PWM period in ms is 1.024*bits (single PWM mode) or 2.048*bits (extended PWM mode), bits is the multiplier (bit:9-15) */
    } bit;                  /*!< represents the 16-bit PWM control register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit PWM control register as `uint16_t` */
    float    pwm_period;    /*!< PWM period in ms is calculated from `pwm_period_mult` and `pwm_mode`. */
} mlx90614_pwmctrl_register_t;

/**
 * @brief MLX90614 flags register structure.
 */
typedef union __attribute__((packed)) mlx90614_flags_register_u {
    struct {
        uint16_t reserved1:3;         /*!< reserved 0     (bit:0-2) */ 
        bool     not_implemented:1;   /*!< not implemented (bit:3) */
        bool     init:1;              /*!< POR initialization routine is still ongoing. Low active.  (bit:4) */
        bool     ee_dead:1;           /*!< EEPROM double error has occurred. High active  (bit:5) */
        bool     unused:1;            /*!< unused (bit:6) */
        bool     ee_busy:1;           /*!< the previous write/erase EEPROM access is still in progress. High active (bit:7)*/
        uint16_t reserved2:8;         /*!< reserved 0 (bit:8-15) */
    } bit;                  /*!< represents the 16-bit flags register parts in bits. */
    uint16_t reg;           /*!< represents the 16-bit flags register as `uint16_t` */
} mlx90614_flags_register_t;




/**
 * @brief MLX90614 device descriptor structure definition.
 */
typedef struct mlx90614_device_s {
    mlx90614_config_t           config;             /*!< mlx90614 configuration */
    i2c_master_dev_handle_t     i2c_handle;         /*!< mlx90614 I2C device handle */
    //uint8_t                     i2c_dev_address;    /*!< mlx90614 I2C device address */
    uint32_t                    ident_number_hi;    /*!< mlx90614 device identification number 32-bit hi */
    uint32_t                    ident_number_lo;    /*!< mlx90614 device identification number 32-bit lo */
    float                       pwm_period_multiplier;
} mlx90614_device_t;

/*
* static constant declarations
*/
static const char *TAG = "mlx90614";

/*
* functions and subroutines
*/

/**
 * @brief Calculates mlx90614 crc8 value using a x^8+x^2+x^1+1 poly.  See datasheet for details.
 *
 * @param[in] crc Crc received for data to validate
 * @param[in] data Data to perform crc8 validation
 * @return uint8_t Calculated crc8 value.
 */
static inline uint8_t mlx90614_calculate_crc8(const uint8_t crc, const uint8_t data) {
    uint8_t i;
	uint8_t data_crc = crc ^ data;

	for ( i = 0; i < 8; i++ ) {
		if (( data_crc & 0x80 ) != 0 ) {
			data_crc <<= 1;
			data_crc ^= MLX90614_CRC8_POLYNOM;
		} else {
			data_crc <<= 1;
		}
	}

	return data_crc;
}

/**
 * @brief Converts milliseconds to ticks.
 *
 * @param[in] ms Milliseconds to convert to ticks.
 * @return size_t Converted ms in ticks.
 */
static inline size_t mlx90614_get_tick_duration(const uint16_t ms) {
    size_t res = pdMS_TO_TICKS(ms);

    return res == 0 ? 1 : res;
}

/**
 * @brief Decodes raw `uint16_t` temperature to floating point temperature in degrees celsius.
 *
 * @param[in] raw_data Raw `uint16_t` temperature to decode.
 * @return float Decoded floating point temperature in degrees celsius.
 */
static inline float mlx90614_decode_temperature(const uint16_t encoded_temperature) {
    float decoded_temperature = (float)encoded_temperature * 0.02f;
    decoded_temperature -= 273.15f;  // kelvin to celsius
    return decoded_temperature;
}

/**
 * @brief Encodes floating point temperature in degrees celsius to raw `uint16_t` temperature.
 *
 * @param[in] temperature Decoded floating point temperature to encode.
 * @return uint16_t Encoded raw `uint16_t` temperature.
 */
static inline uint16_t mlx90614_encode_temperature(const float decoded_temperature) {
    float encoded_temperature = decoded_temperature + 273.15f;
	encoded_temperature *= 50.0f;  // then multiply by 0.02 degK / bit
	return (uint16_t)encoded_temperature;
}

/**
 * @brief Decodes raw `uint16_t` IR data to `int16_t` IR data.
 * 
 * @param raw_data Raw `uint16_t` IR data to decode.
 * @return int16_t Decoded IR data.
 */
static inline int16_t mlx90614_decode_ir(const uint16_t raw_data) {
    int16_t ir = raw_data;
    if(raw_data > 0x7FFF) {
        ir = 0x8000 - ir;
    }
    return ir;
}

/**
 * @brief HAL that reads a word (2-bytes) with CRC validation from MLX90614.
 * 
 * @param device MLX90614 device descriptor.
 * @param reg_addr MLX90614 read register (1-byte).
 * @param word MLX90614 register data (2-bytes).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_read_word_from(mlx90614_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit24_uint8_buffer_t rx;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit_receive, i2c read from failed" );

    /* compute and validate crc */
    uint8_t crc = mlx90614_calculate_crc8(0, (device->config.i2c_address << 1));

    crc = mlx90614_calculate_crc8(crc, reg_addr);
	crc = mlx90614_calculate_crc8(crc, (device->config.i2c_address << 1) + 1);
	crc = mlx90614_calculate_crc8(crc, rx[0]); // lsb
	crc = mlx90614_calculate_crc8(crc, rx[1]); // msb

    // validate calculated crc vs pec received
    if (crc == rx[2]) {
        *word = rx[0] | (rx[1] << 8);
	} else {
		return ESP_ERR_INVALID_CRC;
	}

    return ESP_OK;
}

static inline esp_err_t mlx90614_i2c_write_command(mlx90614_device_t *const device, const uint8_t command) {
    bit16_uint8_buffer_t tx = { 0, 0 };
    uint8_t crc;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    tx[0] = command;       // command

    crc = mlx90614_calculate_crc8(0, (device->config.i2c_address << 1));
	crc = mlx90614_calculate_crc8(crc, tx[0]); // command

    tx[1] = crc;            // pec

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_mlx90614_write_command failed" );

    return ESP_OK;
}

/**
 * @brief HAL that writes a word (2-bytes) with CRC to MLX90614.
 * 
 * @param device MLX90614 device descriptor.
 * @param reg_addr MLX90614 write register (1-byte).
 * @param word MLX90614 data (2-bytes) to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_write_word_to(mlx90614_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    bit32_uint8_buffer_t tx = { 0 };
    uint8_t crc; 

    /* validate arguments */
    ESP_ARG_CHECK( device );

    tx[0] = reg_addr;       // register
    tx[1] = word & 0x00FF;  // lsb
    tx[2] = word >> 8;      // msb

    crc = mlx90614_calculate_crc8(0, (device->config.i2c_address << 1));
	crc = mlx90614_calculate_crc8(crc, tx[0]); // register
	crc = mlx90614_calculate_crc8(crc, tx[1]); // lsb
	crc = mlx90614_calculate_crc8(crc, tx[2]); // msb

    tx[3] = crc;            // pec

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT32_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_mlx90614_write_word failed" );

    return ESP_OK;
}

/**
 * @brief HAL that writes a word (2-bytes) to MLX90614 EEPROM.
 * 
 * @param device MLX90614 device descriptor.
 * @param reg_addr MLX90614 write register (1-byte).
 * @param word MLX90614 data (2-bytes) to write.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_write_eeprom_to(mlx90614_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    // clear eeprom register
    ESP_ERROR_CHECK( mlx90614_i2c_write_word_to(device, reg_addr, MLX90614_CMD_EEPROM_CLR_CELL) );

    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(MLX90614_EEPROM_RDWR_DELAY_MS));

    // write data to register
    ESP_ERROR_CHECK( mlx90614_i2c_write_word_to(device, reg_addr, word) );

    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(MLX90614_EEPROM_RDWR_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MLX90614 I2C HAL read the identification number as two 32-bit values (hi and lo).
 *
 * @param[in] device MLX90614 device descriptor.
 * @param ident_number_hi MLX90614 identifier number high.
 * @param ident_number_lo MLX90614 identifier number low.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_get_ident_registers(mlx90614_device_t *const device, uint32_t *const ident_number_hi, uint32_t *const ident_number_lo) {
    uint16_t id_num[BIT32_UINT8_BUFFER_SIZE]; // 64-bit ident value

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RD_IDNUM1, &id_num[0]) );
    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RD_IDNUM2, &id_num[1]) );
    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RD_IDNUM3, &id_num[2]) );
    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RD_IDNUM4, &id_num[3]) );

    *ident_number_hi = id_num[2] | (id_num[3] << 16);
    *ident_number_lo = id_num[0] | (id_num[1] << 16);

    return ESP_OK;
}

/**
 * @brief MLX90614 I2C HAL read configuration register.
 * 
 * @param[in] device MLX90614 device descriptor.
 * @param[out] reg MLX90614 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_get_config_register(mlx90614_device_t *const device, mlx90614_config_register_t *const reg) {
    uint16_t tmp = 0;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_CFGREG, &tmp) );

    reg->reg = tmp;

    return ESP_OK;
}

/**
 * @brief MLX90614 I2C HAL write configuration register.
 * 
 * @param[in] device MLX90614 device descriptor.
 * @param[in] reg MLX90614 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_set_config_register(mlx90614_device_t *const device, const mlx90614_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_write_word_to(device, MLX90614_CMD_EEPROM_RDWR_CFGREG, reg.reg) );

    return ESP_OK;
}

/**
 * @brief MLX90614 I2C HAL read PWM control register.
 * 
 * @param[in] device MLX90614 device descriptor.
 * @param[out] reg MLX90614 PWM control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_get_pwmctrl_register(mlx90614_device_t *const device, mlx90614_pwmctrl_register_t *const reg) {
    uint16_t tmp = 0;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_PWMCTRL, &tmp) );

    reg->reg = tmp;

    if(reg->bit.pwm_mode == MLX90614_PWM_MODE_SINGLE) {
        reg->pwm_period = (float)reg->bit.pwm_period_mult * 1.024f;
    } else {
        reg->pwm_period = (float)reg->bit.pwm_period_mult * 2.048f;
    }

    return ESP_OK;
}

/**
 * @brief MLX90614 I2C HAL write PWM control register.
 * 
 * @param[in] device MLX90614 device descriptor.
 * @param[in] reg MLX90614 PWM control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_set_pwmctrl_register(mlx90614_device_t *const device, const mlx90614_pwmctrl_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_write_word_to(device, MLX90614_CMD_EEPROM_RDWR_PWMCTRL, reg.reg) );

    return ESP_OK;
}

/**
 * @brief MLX90614 I2C HAL read flags register.
 * 
 * @param[in] device MLX90614 device descriptor.
 * @param[out] reg MLX90614 flags register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t mlx90614_i2c_get_flags_register(mlx90614_device_t *const device, mlx90614_flags_register_t *const reg) {
    uint16_t tmp = 0;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_READ_FLAGS_REG, &tmp) );

    reg->reg = tmp;

    return ESP_OK;
}

esp_err_t mlx90614_init(i2c_master_bus_handle_t master_handle, const mlx90614_config_t *mlx90614_config, mlx90614_handle_t *mlx90614_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && mlx90614_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(MLX90614_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, mlx90614_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, mlx90614 device handle initialization failed", mlx90614_config->i2c_address);

    /* validate memory availability for handle */
    mlx90614_device_t* device = (mlx90614_device_t*)calloc(1, sizeof(mlx90614_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c mlx90614 device");

    /* copy configuration */
    device->config = *mlx90614_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MLX90614_CMD_DELAY_MS));

    /* mlx90614 attempt to read configured identification numbers */
    //ESP_GOTO_ON_ERROR(mlx90614_get_ident_numbers(out_handle, &out_handle->ident_number_hi, &out_handle->ident_number_lo), err_handle, TAG, "i2c mlx90614 read identification numbers failed");

    /* set device handle */
    *mlx90614_handle = (mlx90614_handle_t)device;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MLX90614_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t mlx90614_get_temperatures(mlx90614_handle_t handle, float *const ambient_temperature, float *const object1_temperature, float *const object2_temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && ambient_temperature && object1_temperature && object2_temperature );

    ESP_ERROR_CHECK( mlx90614_get_ambient_temperature(handle, ambient_temperature) );
    ESP_ERROR_CHECK( mlx90614_get_object1_temperature(handle, object1_temperature) );
    ESP_ERROR_CHECK( mlx90614_get_object2_temperature(handle, object2_temperature) );

    return ESP_OK;
}

esp_err_t mlx90614_get_ambient_temperature(mlx90614_handle_t handle, float *const ambient_temperature) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_RAM_READ_TA, &raw_data) );

    //if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *ambient_temperature = mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_get_object1_temperature(mlx90614_handle_t handle, float *const object1_temperature) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_RAM_READ_TOBJ1, &raw_data) );

    //if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *object1_temperature = mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_get_object2_temperature(mlx90614_handle_t handle, float *const object2_temperature) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_RAM_READ_TOBJ2, &raw_data) );

    //if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *object2_temperature = mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_get_ir_channel1(mlx90614_handle_t handle, int16_t *const ir_channel1) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    uint16_t raw_data;
    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_RAM_READ_RAWIR1, &raw_data) );

    *ir_channel1 = mlx90614_decode_ir(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_get_ir_channel2(mlx90614_handle_t handle, int16_t *const ir_channel2) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    uint16_t raw_data;
    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_RAM_READ_RAWIR2, &raw_data) );

    *ir_channel2 = mlx90614_decode_ir(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_get_ambient_temperature_range(mlx90614_handle_t handle, float *const ambient_temperature_range) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_TARANGE, &raw_data) );

    if (raw_data > 0x7FFF) return ESP_ERR_INVALID_RESPONSE;

    *ambient_temperature_range = mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_get_emissivity(mlx90614_handle_t handle, float *const coefficient) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_EMISS, &raw_data) );

    // if we successfully read from the ke register
	// calculate the emissivity coefficient between 0.05 and 1.0:
    *coefficient = ((float)raw_data) / 0xffff;

    return ESP_OK;
}

esp_err_t mlx90614_set_emissivity(mlx90614_handle_t handle, const float coefficient) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    // validate emissivity coefficient range is between 0.05 and 1.0
    ESP_RETURN_ON_FALSE((coefficient <= 1.0f), ESP_ERR_INVALID_ARG, TAG, "emissivity coefficient range must be between 0.05 and 1.0, set emissivity failed");
    ESP_RETURN_ON_FALSE((coefficient >= 0.05f), ESP_ERR_INVALID_ARG, TAG, "emissivity coefficient range must be between 0.05 and 1.0, set emissivity failed");

    uint16_t raw_data = (uint16_t)(0xffff * coefficient);

    ESP_ERROR_CHECK( mlx90614_i2c_write_eeprom_to(device, MLX90614_CMD_EEPROM_RDWR_EMISS, raw_data) );

    return ESP_OK;
}


esp_err_t mlx90614_get_object_maximum_temperature(mlx90614_handle_t handle, float *const temperature) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_TOMAX, &raw_data) );

    *temperature = mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_set_object_maximum_temperature(mlx90614_handle_t handle, const float temperature) {
        mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    uint16_t raw_data = mlx90614_encode_temperature(temperature);

    ESP_ERROR_CHECK( mlx90614_i2c_write_eeprom_to(device, MLX90614_CMD_EEPROM_RDWR_TOMAX, raw_data) );

    return ESP_OK;
}

esp_err_t mlx90614_get_object_minimum_temperature(mlx90614_handle_t handle, float *const temperature) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_TOMIN, &raw_data) );

    *temperature = mlx90614_decode_temperature(raw_data);

    return ESP_OK;
}

esp_err_t mlx90614_set_object_minimum_temperature(mlx90614_handle_t handle, const float temperature) {
        mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    uint16_t raw_data = mlx90614_encode_temperature(temperature);

    ESP_ERROR_CHECK( mlx90614_i2c_write_eeprom_to(device, MLX90614_CMD_EEPROM_RDWR_TOMIN, raw_data) );

    return ESP_OK;
}

esp_err_t mlx90614_get_address(mlx90614_handle_t handle, uint8_t *const address) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_SMBADDR, &raw_data) );

    *address = (uint8_t)raw_data;

    return ESP_OK;
}

esp_err_t mlx90614_set_address(mlx90614_handle_t handle, const uint8_t address) {
    uint16_t raw_data;
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    // make sure the address is within the proper range:
	if ((address >= 0x80) || (address == 0x00))
		return ESP_ERR_INVALID_ARG;

    // read the existing device address
    ESP_ERROR_CHECK( mlx90614_i2c_read_word_from(device, MLX90614_CMD_EEPROM_RDWR_SMBADDR, &raw_data) );

    raw_data &= 0xFF00;  // mask out the address (msb)
	raw_data |= address; // add the new address

    ESP_ERROR_CHECK( mlx90614_i2c_write_eeprom_to(device, MLX90614_CMD_EEPROM_RDWR_SMBADDR, raw_data) );

    return ESP_OK;
}

esp_err_t mlx90614_get_iir(mlx90614_handle_t handle, mlx90614_sensor_iirs_t *const iir) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;
    mlx90614_config_register_t reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_get_config_register(device, &reg) );

    *iir = reg.bit.iir;

    return ESP_OK;
}

esp_err_t mlx90614_set_iir(mlx90614_handle_t handle, const mlx90614_sensor_iirs_t iir) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;
    mlx90614_config_register_t reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_get_config_register(device, &reg) );

    reg.bit.iir = iir;

    ESP_ERROR_CHECK( mlx90614_i2c_set_config_register(device, reg) );

    return ESP_OK;
}

esp_err_t mlx90614_get_fir(mlx90614_handle_t handle, mlx90614_sensor_firs_t *const fir) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;
    mlx90614_config_register_t reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_get_config_register(device, &reg) );

    *fir = reg.bit.fir;

    return ESP_OK;
}

esp_err_t mlx90614_set_fir(mlx90614_handle_t handle, const mlx90614_sensor_firs_t fir) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;
    mlx90614_config_register_t reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_get_config_register(device, &reg) );

    reg.bit.fir = fir;

    ESP_ERROR_CHECK( mlx90614_i2c_set_config_register(device, reg) );

    return ESP_OK;
}

esp_err_t mlx90614_get_gain(mlx90614_handle_t handle, mlx90614_sensor_gains_t *const gain) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;
    mlx90614_config_register_t reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_get_config_register(device, &reg) );

    *gain = reg.bit.gain;

    return ESP_OK;
}

esp_err_t mlx90614_set_gain(mlx90614_handle_t handle, const mlx90614_sensor_gains_t gain) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;
    mlx90614_config_register_t reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_ERROR_CHECK( mlx90614_i2c_get_config_register(device, &reg) );

    reg.bit.gain = gain;

    ESP_ERROR_CHECK( mlx90614_i2c_set_config_register(device, reg) );

    return ESP_OK;
}

esp_err_t mlx90614_sleep(mlx90614_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t mlx90614_wakeup(mlx90614_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t mlx90614_remove(mlx90614_handle_t handle) {
    mlx90614_device_t* device = (mlx90614_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    return i2c_master_bus_rm_device(device->i2c_handle);
}

esp_err_t mlx90614_delete(mlx90614_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( mlx90614_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* mlx90614_get_fw_version(void) {
    return (const char*)MLX90614_FW_VERSION_STR;
}

int32_t mlx90614_get_fw_version_number(void) {
    return (int32_t)MLX90614_FW_VERSION_INT32;
}