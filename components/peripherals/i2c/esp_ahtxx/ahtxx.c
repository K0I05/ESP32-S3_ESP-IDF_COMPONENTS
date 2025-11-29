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
 * @file ahtxx.c
 *
 * ESP-IDF driver for AHTXX temperature and humidity sensor
 * 
 * https://github.com/libdriver/aht30/blob/main/src/driver_aht30.c
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

/**
 * dependency includes
 */

#include "include/ahtxx.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * constant definitions
 */

#define AHTXX_CRC8_MASK             UINT8_C(0x80)   /*!< ahtxx CRC8 mask */
#define AHTXX_CRC8_INIT             UINT8_C(0xff)   /*!< ahtxx CRC8 initialization */
#define AHTXX_CRC8_POLYNOM          UINT8_C(0x31)   /*!< ahtxx CRC8 polynomial */

#define AHTXX_STATUS_WORD           UINT8_C(0x18)   /*!< ahtxx initialization status word (default) */

/* Register addresses */
#define AHTXX_REG_1B                UINT8_C(0x1b)   /*!< AHT2x/3x initialization register */
#define AHTXX_REG_1C                UINT8_C(0x1c)   /*!< AHT2x/3x initialization register */
#define AHTXX_REG_1E                UINT8_C(0x1e)   /*!< AHT2x/3x initialization register */

/* Control bytes */
#define AHTXX_CTRL_CALI             UINT8_C(0x08)   /*!< Calibration enable control byte */
#define AHTXX_CTRL_MEAS             UINT8_C(0x33)   /*!< Measurement control byte */
#define AHTXX_CTRL_NOP              UINT8_C(0x00)   /*!< No operation control byte */

/* Commands */
#define AHTXX_CMD_AHT10_INIT        UINT8_C(0xe1)   /*!< AHT10 initialization command + 0x08 + 0x00 */
#define AHTXX_CMD_AHT20_INIT        UINT8_C(0xbe)   /*!< AHT20/2x/3x initialization command + 0x08 + 0x00 */
#define AHTXX_CMD_STATUS            UINT8_C(0x71)   /*!< Status register read command */
#define AHTXX_CMD_TRIGGER_MEAS      UINT8_C(0xac)   /*!< Measurement trigger command + 0x33 + 0x00 */
#define AHTXX_CMD_RESET             UINT8_C(0xba)   /*!< Soft-reset command */

/* Timing constants - All values from datasheet specifications */
#define AHTXX_DATA_POLL_TIMEOUT_MS  UINT16_C(100)   /*!< Maximum time to wait for data ready (datasheet: max 80ms) */
#define AHTXX_DATA_READY_DELAY_MS   UINT16_C(2)     /*!< Delay between status polls */
#define AHTXX_POWERUP_DELAY_MS      UINT16_C(120)   /*!< Power-up time (datasheet: max 100ms, +20ms margin) */
#define AHTXX_RESET_DELAY_MS        UINT16_C(25)    /*!< Reset recovery time (datasheet: max 20ms, +5ms margin) */
#define AHTXX_SETUP_DELAY_MS        UINT16_C(15)    /*!< Initialization command processing time (datasheet: max 10ms) */
#define AHTXX_APPSTART_DELAY_MS     UINT16_C(10)    /*!< Delay after init before first measurement */
#define AHTXX_RETRY_DELAY_MS        UINT16_C(2)     /*!< Delay between I2C transaction retry attempts */
#define AHTXX_CMD_DELAY_MS          UINT16_C(5)     /*!< Inter-command delay for bus stability */
#define AHTXX_MEAS_PROC_DELAY_MS    UINT16_C(80)    /*!< Measurement processing time (datasheet: typical 75-80ms) */
#define AHTXX_TX_RX_DELAY_MS        UINT16_C(10)    /*!< Delay between write and read transactions */


#define AHTXX_DRYBULB_MAX           (float)(125.0)  //!< ahtxx maximum dry-bulb temperature range
#define AHTXX_DRYBULB_MIN           (float)(-40.0)  //!< ahtxx minimum dry-bulb temperature range
#define AHTXX_HUMIDITY_MAX          (float)(100.0)  //!< ahtxx maximum relative humidity range
#define AHTXX_HUMIDITY_MIN          (float)(0.0)    //!< ahtxx minimum relative humidity range


/**
 * macro definitions
 */

#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief AHTXX status register structure definition.
 */
typedef union __attribute__((packed)) ahtxx_status_register_u {
    struct {
        uint8_t reserved1:3; /*!< reserved                       (bit:0-2)  */
        bool calibrated:1;   /*!< ahtxx is calibrated when true  (bit:3) */
        uint8_t reserved2:3; /*!< reserved                       (bit:4-6) */
        bool busy:1;         /*!< ahtxx is busy when true        (bit:7) */
    } bits;
    uint8_t reg;
} ahtxx_status_register_t;

/**
 * @brief AHTXX device descriptor structure definition.
 */
typedef struct ahtxx_device_s {
    ahtxx_config_t          config;     /*!< ahtxx device configuration */
    hal_master_dev_handle_t hal_handle; /*!< ahtxx HAL device communication handle */
} ahtxx_device_t;

/**
 * static constant declarations
 */

static const char* TAG = "ahtxx";

/**
 * static function and subroutine declarations
 */

/**
 * function and subroutine declarations
 */

/**
 * @brief Calculates AHTXX sensor types with CRC8 value.  See datasheet for details.
 *
 * @param[in] buffer[] Data buffer to perform CRC8 calculation against.
 * @param[in] len Length of data buffer.
 * @return uint8_t Calculated CRC8 value.
 */
static inline uint8_t calculate_crc8(const uint8_t buffer[], const uint8_t len) {
    uint8_t crc = AHTXX_CRC8_INIT;
    for (uint8_t byte = 0; byte < len; byte++) {
        crc ^= buffer[byte];
        for (uint8_t i = 0; i < 8; i++) {
            crc = crc & AHTXX_CRC8_MASK ? (uint8_t)(crc << 1) ^ AHTXX_CRC8_POLYNOM : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/**
 * @brief Validates AHTXX measurement data using CRC8 checksum.
 *
 * @param[in] data Measurement data buffer (6 or 7 bytes).
 * @param[in] len Length of data buffer.
 * @return esp_err_t ESP_OK if CRC is valid, ESP_ERR_INVALID_CRC otherwise.
 */
static inline esp_err_t validate_crc(const uint8_t data[], const uint8_t len) {
    /* CRC is only available for 7-byte responses (AHT20/21/25/30) */
    if (len != BIT56_UINT8_BUFFER_SIZE) {
        return ESP_OK;  // No CRC available for AHT10
    }
    
    /* Calculate CRC on first 6 bytes */
    const uint8_t calculated_crc = calculate_crc8(data, len - 1);
    const uint8_t received_crc = data[len - 1];
    
    if (calculated_crc != received_crc) {
        ESP_LOGE(TAG, "CRC mismatch: calculated=0x%02x, received=0x%02x", calculated_crc, received_crc);
        return ESP_ERR_INVALID_CRC;
    }
    
    return ESP_OK;
}

/**
 * @brief Helper: check if value is in range.
 * 
 * @param value value to check.
 * @param min minimum acceptable value.
 * @param max maximum acceptable value.
 * @return true if value is in range, false otherwise.
 */
static inline bool is_value_in_range(const float value, const float min, const float max) {
    return (value <= max && value >= min);
}

/**
 * @brief Helper: check if dry-bulb temperature is in range.
 * 
 * @param drybulb dry-bulb temperature in degrees Celsius.
 * @return true if dry-bulb temperature is in range, false otherwise.
 */
static inline bool is_drybulb_temperature_in_range(const float drybulb) {
    return is_value_in_range(drybulb, AHTXX_DRYBULB_MIN, AHTXX_DRYBULB_MAX);
}

/**
 * @brief Helper: check if relative humidity is in range.
 * 
 * @param humidity relative humidity in percent.
 * @return true if relative humidity is in range, false otherwise.
 */
static inline bool is_relative_humidity_in_range(const float humidity) {
    return is_value_in_range(humidity, AHTXX_HUMIDITY_MIN, AHTXX_HUMIDITY_MAX);
}

/**
 * @brief Calculates dew-point temperature from air temperature and relative humidity.
 *
 * @param[in] drybulb Dry-bulb temperature in degrees Celsius (valid range: -40 to 80°C).
 * @param[in] humidity Relative humidity in percent (valid range: 0 to 100%).
 * @param[out] dewpoint Calculated dew-point temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if parameters are out of range.
 */
static inline esp_err_t get_dewpoint_temperature(const float drybulb, const float humidity, float *dewpoint) {
    /* validate parameters */
    ESP_ARG_CHECK( dewpoint );
    
    /* validate range of dry-bulb temperature parameter */
    ESP_RETURN_ON_FALSE( is_drybulb_temperature_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated dew-point temperature failed");

    /* validate range of relative humidity parameter */
    ESP_RETURN_ON_FALSE( is_relative_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated dew-point temperature failed");
    
    /* calculate dew-point temperature using Magnus formula */
    const float H = (log10f(humidity) - 2.0f) / 0.4343f + (17.62f * drybulb) / (243.12f + drybulb);
    *dewpoint = 243.12f * H / (17.62f - H);
    
    return ESP_OK;
}

/**
 * @brief Calculates wet-bulb temperature from air temperature and relative humidity.
 *
 * @param[in] drybulb Dry-bulb temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percent.
 * @param[out] wetbulb Calculated wet-bulb temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t get_wetbulb_temperature(const float drybulb, const float humidity, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK( wetbulb );

    /* validate range of dry-bulb temperature parameter */
    ESP_RETURN_ON_FALSE( is_drybulb_temperature_in_range(drybulb), ESP_ERR_INVALID_ARG, TAG, "dry-bulb temperature is out of range, get calculated wet-bulb temperature failed");

    /* validate range of relative humidity parameter */
    ESP_RETURN_ON_FALSE( is_relative_humidity_in_range(humidity), ESP_ERR_INVALID_ARG, TAG, "relative humidity is out of range, get calculated wet-bulb temperature failed");
    
    // calculate wet-bulb temperature
    *wetbulb = drybulb * atanf( 0.151977f * powf( (humidity + 8.313659f), 1.0f/2.0f ) ) + atanf(drybulb + humidity) - atanf(humidity - 1.676331f) + 0.00391838f * powf(humidity, 3.0f/2.0f) * atanf(0.023101f * humidity) - 4.686035f;
    
    return ESP_OK;
}

/**
 * @brief Converts temperature signal to engineering units of measure.
 * 
 * @param temperature_sig ADC temperature signal from AHTXX.
 * @return float Converted temperature measurement from AHTXX in degrees Celsius.
 */
static inline float convert_adc_signal_to_temperature(const uint32_t signal) {
    return ((float)signal / powf(2.0f, 20.0f)) * 200.0f - 50.0f;
}

/**
 * @brief Converts humidity signal to engineering units of measure.
 * 
 * @param humidity_sig ADC humidity signal from AHTXX.
 * @return float Converted humidity measurement from AHTXX in percent.
 */
static inline float convert_adc_signal_to_humidity(uint32_t signal) {
    return ((float)signal / powf(2.0f, 20.0f)) * 100.0f;
}


/**
 * @brief AHTXX HAL reset and initialization of register by register address.
 * 
 * @param device AHTXX device descriptor.
 * @param reg_addr AHTXX reset register address.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_reset_init_register(ahtxx_device_t *device, const uint8_t reg_addr) {
    bit24_uint8_buffer_t tx = { 0 };
    bit24_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate aht type */
    if(device->config.sensor_type == AHTXX_AHT10) {
        return ESP_ERR_NOT_ALLOWED;
    }

    /* set tx command packet */
    tx[0] = reg_addr;
    tx[1] = 0x00;
    tx[2] = 0x00;

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write command to register 0x%02x for reset initialization register failed", reg_addr );
    
    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read(device->hal_handle, rx, BIT24_UINT8_BUFFER_SIZE ), TAG, "read from register 0x%02x for reset initialization register failed", reg_addr );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    /* set tx data packet */
    tx[0] = 0xb0 | reg_addr;
    tx[1] = rx[1];
    tx[2] = rx[2];

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write data to register 0x%02x for reset initialization register failed", reg_addr );
    
    return ESP_OK;
}

/**
 * @brief AHTXX HAL read status register.
 *
 * @param[in] device AHTXX device descriptor.
 * @param[out] reg AHTXX status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_status_register(ahtxx_device_t *device, ahtxx_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt write/read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read_from(device->hal_handle, AHTXX_CMD_STATUS, &reg->reg, BIT8_UINT8_BUFFER_SIZE), TAG, "read status register failed" );

    return ESP_OK;
}

/**
 * @brief AHTXX HAL write reset register to reset device with restart delay.
 * 
 * @param device AHTXX device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_set_reset_register(ahtxx_device_t *device) {
    const bit8_uint8_buffer_t tx = { AHTXX_CMD_RESET };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "write reset register failed" );

    /* delay task before transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_RESET_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief AHTXX HAL initialization and calibration setup.  This is a one-time call at start-up if the device isn't initialized and calibrated.
 *
 * @param device AHTXX device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_setup_registers(ahtxx_device_t *const device) {
    const bit24_uint8_buffer_t aht10_tx = { AHTXX_CMD_AHT10_INIT, AHTXX_CTRL_CALI, AHTXX_CTRL_NOP };
    const bit24_uint8_buffer_t aht20_tx = { AHTXX_CMD_AHT20_INIT, AHTXX_CTRL_CALI, AHTXX_CTRL_NOP };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* handle aht init command by sensor type */
    switch(device->config.sensor_type) {
        case AHTXX_AHT10:
            /* attempt write transaction for aht10 sensor type initialization */
            ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, aht10_tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write initialization register 0xe1 failed" );
            break;
        case AHTXX_AHT20:
            /* attempt write transaction for aht20 sensor type initialization */
            ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, aht20_tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write initialization register 0xbe failed" );
            break;
        case AHTXX_AHT21:
        case AHTXX_AHT25:
        case AHTXX_AHT30:
            /* attempt reset transaction for aht21, aht25, and aht30 sensor types initialization */
            ESP_RETURN_ON_ERROR( hal_reset_init_register(device, AHTXX_REG_1B), TAG, "reset initialization registers 0x1b failed" );
            ESP_RETURN_ON_ERROR( hal_reset_init_register(device, AHTXX_REG_1C), TAG, "reset initialization registers 0x1c failed" );
            ESP_RETURN_ON_ERROR( hal_reset_init_register(device, AHTXX_REG_1E), TAG, "reset initialization registers 0x1e failed" );
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* delay task before next transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_SETUP_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief AHTXX HAL calibration validation.
 * 
 * @param device AHTXX device descriptor.
 * @return esp_err_t ESP_OK on success, device is calibrated.
 */
static inline esp_err_t hal_is_calibrated(ahtxx_device_t *const device) {
    ahtxx_status_register_t status_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( hal_get_status_register(device, &status_reg), TAG, "read status register for calibrate failed" );

    /* handle sensor setup by sensor type */
    if(device->config.sensor_type == AHTXX_AHT10 || device->config.sensor_type == AHTXX_AHT20) {
        /* validate calibration status */
        if(status_reg.bits.calibrated == false) {
            /* attempt to write init command */
            ESP_RETURN_ON_ERROR( hal_setup_registers(device), TAG, "setup sensor for calibrate failed" );
        }
    } else {
        /* validate register status */
        if(status_reg.reg != AHTXX_STATUS_WORD) {
            /* attempt to reset initialization registers */
            ESP_RETURN_ON_ERROR( hal_setup_registers(device), TAG, "setup sensor for calibrate failed" );
        }
    }

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( hal_get_status_register(device, &status_reg), TAG, "read status register for calibrate failed" );

    /* validate calibration status */
    if(status_reg.bits.calibrated == false) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_STATE, TAG, "setup and initialize sensor for calibrate failed" );
    }

    return ESP_OK;
}

/**
 * @brief AHTXX HAL get raw ADC temperature and humidity signals.
 * 
 * @param device AHTXX device descriptor.
 * @param temperature Pointer to store raw ADC temperature signal.
 * @param humidity Pointer to store raw ADC humidity signal.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_adc_signals(ahtxx_device_t *const device, uint32_t *const temperature, uint32_t *const humidity) {
    const bit24_uint8_buffer_t tx = { AHTXX_CMD_TRIGGER_MEAS, AHTXX_CTRL_MEAS, AHTXX_CTRL_NOP };
    esp_err_t                 ret = ESP_OK;
    const uint64_t     start_time = esp_timer_get_time();
    bool            data_is_ready = false;
    bit56_uint8_buffer_t       rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write(device->hal_handle, tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write measurement trigger command for get measurement failed" );

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_MEAS_PROC_DELAY_MS));

    /* attempt to poll status until data is available or timeout occurs  */
    do {
        ahtxx_status_register_t status_reg = { 0 };

        /* attempt to read status register to check if data is ready */
        ESP_GOTO_ON_ERROR( hal_get_status_register(device, &status_reg), err, TAG, "read status register for busy status failed" );

        /* set data is ready flag */
        data_is_ready = !status_reg.bits.busy;

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (AHTXX_DATA_POLL_TIMEOUT_MS * 1000))) {
            return ESP_ERR_TIMEOUT;
        }

        /* delay task before next poll if data is not ready */
        if (!data_is_ready) {
            vTaskDelay(pdMS_TO_TICKS(AHTXX_DATA_READY_DELAY_MS));
        }
    } while (data_is_ready == false);

    /* handle aht sensor read by type */
    if(device->config.sensor_type == AHTXX_AHT10) {
        /* aht10 returns 6 bytes */

        /* attempt read transaction for aht10 sensor type */
        ESP_RETURN_ON_ERROR( hal_master_read(device->hal_handle, rx, BIT48_UINT8_BUFFER_SIZE), TAG, "read measurement data for get measurement failed" );
    } else {
        /* aht20, aht21, aht25, and aht30 return 7 bytes */

        /* attempt read transaction for aht20, aht21, aht25, and aht30 sensor types */
        ESP_RETURN_ON_ERROR( hal_master_read(device->hal_handle, rx, BIT56_UINT8_BUFFER_SIZE), TAG, "read measurement data for get measurement failed" );

        /* validate CRC if available (AHT20/21/25/30) */
        ESP_RETURN_ON_ERROR( validate_crc(rx, BIT56_UINT8_BUFFER_SIZE), TAG, "CRC validation failed for measurement data" );
    }

    /* concat humidity signal */
    *humidity = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | ((uint32_t)rx[3] >> 4);

    /* concat temperature signal */
    *temperature = ((uint32_t)(rx[3] & 0x0f) << 16) | ((uint32_t)rx[4] << 8) | (uint32_t)rx[5];

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ahtxx_init(const void* master_handle, const ahtxx_config_t *ahtxx_config, ahtxx_handle_t *const ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && (ahtxx_config || ahtxx_handle) );

    /* instantiate and validate memory availability for handle */
    esp_err_t ret = ESP_OK;
    ahtxx_device_t* device = (ahtxx_device_t*)calloc(1, sizeof(ahtxx_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for ahtxx device, init failed");

    /* copy configuration */
    device->config = *ahtxx_config;

    /* validate hal master bus interface */
    ESP_GOTO_ON_FALSE( ahtxx_config->hal_bif == HAL_MASTER_BIF_I2C, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid HAL master bus interface, device handle initialization failed");

    /* cast i2c_master_bus_handle_t to void pointer */
    i2c_master_bus_handle_t i2c_master_handle = (i2c_master_bus_handle_t)master_handle;

    /* validate i2c master bus handle */
    ESP_GOTO_ON_FALSE( i2c_master_handle, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid master bus handle, device handle initialization failed");

    /* cast hal_config to i2c_device_config_t pointer */
    const i2c_device_config_t* i2c_dev_conf_ptr = (const i2c_device_config_t*)device->config.hal_config;

    /* validate i2c device configuration */
    ESP_GOTO_ON_FALSE(i2c_dev_conf_ptr, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid i2c device configuration, device handle initialization failed");
    
    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_dev_conf_ptr->device_address,
        .scl_speed_hz    = i2c_dev_conf_ptr->scl_speed_hz
    };

    /* delay task before transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_POWERUP_DELAY_MS));

    /* create hal master device */
    ret = hal_master_new_i2c_device(i2c_master_handle, &i2c_dev_conf, &device->hal_handle);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal_master_new_i2c_device failed, device handle initialization failed");

    /* validate device exists on the hal master communication bus */
    ret = hal_master_probe(device->hal_handle, i2c_dev_conf_ptr->device_address);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "device does not exist on the hal master communication bus, device handle initialization failed");

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_set_reset_register(device), TAG, "write reset register for init failed" );

    /* attempt to check sensor calibration */
    ESP_RETURN_ON_ERROR( hal_is_calibrated(device), TAG, "calibration for init failed" );

    /* set device handle */
    *ahtxx_handle = (ahtxx_handle_t)device;

    /* delay task before transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_APPSTART_DELAY_MS));

    return ESP_OK;

    /* something went wrong - after device descriptor instantiation */
    err_handle:
        /* remove device from hal master communication bus */
        if (device->hal_handle) {
            hal_master_delete(device->hal_handle);
        }

        /* free device handle */
        free(device);
    err:
        return ret;
}

esp_err_t ahtxx_get_measurement(ahtxx_handle_t handle, float *const temperature, float *const humidity) {
    uint32_t   temp_signal = 0;
    uint32_t    hum_signal = 0;
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && humidity );

    /* attempt to get adc signals */
    ESP_RETURN_ON_ERROR( hal_get_adc_signals(device, &temp_signal, &hum_signal), TAG, "get adc signals for get measurement failed" );

    /* compute and set temperature */
    *temperature = convert_adc_signal_to_temperature(temp_signal);

    /* compute and set humidity */
    *humidity = convert_adc_signal_to_humidity(hum_signal);
    
    return ESP_OK;
}


esp_err_t ahtxx_get_measurements(ahtxx_handle_t handle, float *const drybulb, float *const humidity, float *const dewpoint, float *const wetbulb) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && drybulb && humidity && dewpoint && wetbulb );

    /* attempt to get temperature and humidity measurements */
    ESP_RETURN_ON_ERROR( ahtxx_get_measurement(handle, drybulb, humidity), TAG, "read measurement for get measurements failed" );

    /* compute dew-point from temperature and humidity */
    ESP_RETURN_ON_ERROR( get_dewpoint_temperature(*drybulb, *humidity, dewpoint), TAG, "calculate dew-point temperature for get measurements failed" );

    /* compute wet-bulb from temperature and humidity */
    ESP_RETURN_ON_ERROR( get_wetbulb_temperature(*drybulb, *humidity, wetbulb), TAG, "calculate wet-bulb temperature for get measurements failed" );

    return ESP_OK;
}

esp_err_t ahtxx_get_busy_status(ahtxx_handle_t handle, bool *const busy) {
    ahtxx_status_register_t status_reg = { 0 };
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && busy );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( hal_get_status_register(device, &status_reg), TAG, "read status register for busy status failed" );

    /* set output parameter */
    *busy = status_reg.bits.busy;

    //ESP_LOGD(TAG, "aht2x busy state    %s", busy ? "true" : "false");

    return ESP_OK;
}

esp_err_t ahtxx_get_calibration_status(ahtxx_handle_t handle, bool *const calibrated) {
    ahtxx_status_register_t status_reg = { 0 };
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && calibrated );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( hal_get_status_register(device, &status_reg), TAG, "read status register for calibration status failed" );

    /* set output parameter */
    *calibrated = status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t ahtxx_get_status(ahtxx_handle_t handle, bool *const busy, bool *const calibrated) {
    ahtxx_status_register_t status_reg = { 0 };
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && (busy || calibrated) );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( hal_get_status_register(device, &status_reg), TAG, "read status register for status failed" );

    /* set output parameters */
    *busy       = status_reg.bits.busy;
    *calibrated = status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t ahtxx_reset(ahtxx_handle_t handle) {
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( hal_set_reset_register(device), TAG, "write reset register for reset failed" );

    /* attempt to check sensor calibration */
    ESP_RETURN_ON_ERROR( hal_is_calibrated(device), TAG, "calibration for reset failed" );
    
    return ESP_OK;
}

esp_err_t ahtxx_remove(ahtxx_handle_t handle) {
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from hal bus */
    return hal_master_remove(device->hal_handle);
}

esp_err_t ahtxx_delete(ahtxx_handle_t handle) {
    ahtxx_device_t* device = (ahtxx_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* delete device handle */
    if (device->hal_handle) {
        hal_master_delete(device->hal_handle);
    }

    free(device);
    return ESP_OK;
}

const char* ahtxx_get_fw_version(void) {
    return (const char*)AHTXX_FW_VERSION_STR;
}

int32_t ahtxx_get_fw_version_number(void) {
    return (int32_t)AHTXX_FW_VERSION_INT32;
}