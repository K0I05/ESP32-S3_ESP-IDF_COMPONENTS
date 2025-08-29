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

#define AHTXX_REG_1B                UINT8_C(0x1b)
#define AHTXX_REG_1C                UINT8_C(0x1c)
#define AHTXX_REG_1E                UINT8_C(0x1e)

#define AHTXX_CTRL_CALI             UINT8_C(0x08)
#define AHTXX_CTRL_MEAS             UINT8_C(0x33)
#define AHTXX_CTRL_NOP              UINT8_C(0x00)

#define AHTXX_CMD_AHT10_INIT        UINT8_C(0xe1)   /*!< aht10 initialization command + 0x08 + 0x00 */
#define AHTXX_CMD_AHT20_INIT        UINT8_C(0xbe)   /*!< aht20 initialization command + 0x08 + 0x00 */
#define AHTXX_CMD_STATUS            UINT8_C(0x71)   /*!< ahtxx status register command */
#define AHTXX_CMD_TRIGGER_MEAS      UINT8_C(0xac)   /*!< ahtxx measurement trigger command + 0x33 + 0x00 */
#define AHTXX_CMD_RESET             UINT8_C(0xba)   /*!< ahtxx soft-reset command */

#define AHTXX_DATA_POLL_TIMEOUT_MS  UINT16_C(100)
#define AHTXX_DATA_READY_DELAY_MS   UINT16_C(2)
#define AHTXX_POWERUP_DELAY_MS      UINT16_C(120)
#define AHTXX_RESET_DELAY_MS        UINT16_C(25)
#define AHTXX_SETUP_DELAY_MS        UINT16_C(15)
#define AHTXX_APPSTART_DELAY_MS     UINT16_C(10)    /*!< ahtxx delay after initialization before application start-up */
#define AHTXX_RETRY_DELAY_MS        UINT16_C(2)     /*!< ahtxx delay between an I2C receive transaction retry */
#define AHTXX_CMD_DELAY_MS          UINT16_C(5)     /*!< ahtxx delay before attempting command transactions after a command is issued */
#define AHTXX_MEAS_PROC_DELAY_MS    UINT16_C(80)    /*!< ahtxx delay before attempting read transaction after a measurement trigger command is issued */
#define AHTXX_TX_RX_DELAY_MS        UINT16_C(10)    /*!< ahtxx delay after attempting a transmit transaction and attempting a receive transaction */

#define I2C_XFR_TIMEOUT_MS          (500)          //!< I2C transaction timeout in milliseconds


/**
 * macro definitions
 */

#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


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
static inline uint8_t ahtxx_calculate_crc8(const uint8_t buffer[], const uint8_t len) {
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
 * @brief Calculates dew-point temperature from air temperature and relative humidity.
 *
 * @param[in] temperature air temperature in degrees Celsius.
 * @param[in] humidity relative humidity in percent.
 * @return float calculated dew-point temperature in degrees Celsius.
 */
static inline float ahtxx_calculate_dewpoint(const float temperature, const float humidity) {
    // validate parameters
    if(temperature > 80 || temperature < -40) return ESP_ERR_INVALID_ARG;
    if(humidity > 100 || humidity < 0) return ESP_ERR_INVALID_ARG;
    // calculate dew-point temperature
    float H = (log10f(humidity)-2)/0.4343f + (17.62f*temperature)/(243.12f+temperature);
    return 243.12f*H/(17.62f-H);
}

/**
 * @brief Converts temperature signal to engineering units of measure.
 * 
 * @param temperature_sig ADC temperature signal from AHTXX.
 * @return float Converted temperature measurement from AHTXX in degrees Celsius.
 */
static inline float ahtxx_convert_temperature_signal(const uint32_t temperature_sig) {
    return ((float)temperature_sig / powf(2, 20)) * 200.0f - 50.0f;
}

/**
 * @brief Converts humidity signal to engineering units of measure.
 * 
 * @param humidity_sig ADC humidity signal from AHTXX.
 * @return float Converted humidity measurement from AHTXX in percent.
 */
static inline float ahtxx_convert_humidity_signal(uint32_t humidity_sig) {
    return ((float)humidity_sig / powf(2, 20)) * 100.0f;
}

/**
 * @brief AHTXX I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle AHTXX device handle.
 * @param reg_addr AHTXX register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ahtxx_i2c_read_from(ahtxx_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read from failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(handle->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read from failed" );

    return ESP_OK;
}

/**
 * @brief AHTXX I2C read transaction.
 * 
 * @param handle AHTXX device handle.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ahtxx_i2c_read(ahtxx_handle_t handle, uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(handle->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read failed" );

    return ESP_OK;
}

/**
 * @brief AHTXX I2C write transaction.
 * 
 * @param handle AHTXX device handle.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ahtxx_i2c_write(ahtxx_handle_t handle, const uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Resets AHTXX initialization register by register address.
 * 
 * @param handle AHTXX device handle.
 * @param reg_addr AHTXX reset register address.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ahtxx_reset_init_register(ahtxx_handle_t handle, const uint8_t reg_addr) {
    bit24_uint8_buffer_t tx = { 0 };
    bit24_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate aht type */
    if(handle->dev_config.sensor_type == AHTXX_AHT10) {
        return ESP_ERR_NOT_ALLOWED;
    }

    /* set tx command packet */
    tx[0] = reg_addr;
    tx[1] = 0x00;
    tx[2] = 0x00;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ahtxx_i2c_write(handle, tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write command to register 0x%02x for reset initialization register failed", reg_addr );
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ahtxx_i2c_read(handle, rx, BIT24_UINT8_BUFFER_SIZE ), TAG, "read from register 0x%02x for reset initialization register failed", reg_addr );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    /* set tx data packet */
    tx[0] = 0xb0 | reg_addr;
    tx[1] = rx[1];
    tx[2] = rx[2];

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ahtxx_i2c_write(handle, tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write data to register 0x%02x for reset initialization register failed", reg_addr );
    
    return ESP_OK;
}

/**
 * @brief AHTXX initialization and calibration setup.  This is a one-time call at start-up if the device isn't initialized and calibrated.
 *
 * @param handle AHTXX device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ahtxx_setup(ahtxx_handle_t handle) {
    const bit24_uint8_buffer_t aht10_tx = { AHTXX_CMD_AHT10_INIT, AHTXX_CTRL_CALI, AHTXX_CTRL_NOP };
    const bit24_uint8_buffer_t aht20_tx = { AHTXX_CMD_AHT20_INIT, AHTXX_CTRL_CALI, AHTXX_CTRL_NOP };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* handle aht init command by sensor type */
    switch(handle->dev_config.sensor_type) {
        case AHTXX_AHT10:
            /* attempt i2c write transaction for aht10 sensor type initialization */
            ESP_RETURN_ON_ERROR( ahtxx_i2c_write(handle, aht10_tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write initialization register 0xe1 failed" );
            break;
        case AHTXX_AHT20:
            /* attempt i2c write transaction for aht20 sensor type initialization */
            ESP_RETURN_ON_ERROR( ahtxx_i2c_write(handle, aht20_tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write initialization register 0xbe failed" );
            break;
        case AHTXX_AHT21:
        case AHTXX_AHT25:
        case AHTXX_AHT30:
            /* attempt i2c reset transaction for aht21, aht25, and aht30 sensor types initialization */
            ESP_RETURN_ON_ERROR( ahtxx_reset_init_register(handle, AHTXX_REG_1B), TAG, "reset initialization registers 0x1b failed" );
            ESP_RETURN_ON_ERROR( ahtxx_reset_init_register(handle, AHTXX_REG_1C), TAG, "reset initialization registers 0x1c failed" );
            ESP_RETURN_ON_ERROR( ahtxx_reset_init_register(handle, AHTXX_REG_1E), TAG, "reset initialization registers 0x1e failed" );
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_SETUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t ahtxx_get_status_register(ahtxx_handle_t handle, ahtxx_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && reg );

    /* attempt i2c write/read transaction */
    ESP_RETURN_ON_ERROR( ahtxx_i2c_read_from(handle, AHTXX_CMD_STATUS, &reg->reg, BIT8_UINT8_BUFFER_SIZE), TAG, "read status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t ahtxx_init(const i2c_master_bus_handle_t master_handle, const ahtxx_config_t *ahtxx_config, ahtxx_handle_t *const ahtxx_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && (ahtxx_config || ahtxx_handle) );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ahtxx_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ahtxx device handle initialization failed", ahtxx_config->i2c_address);

    /* validate memory availability for handle */
    ahtxx_handle_t out_handle;
    out_handle = (ahtxx_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ahtxx device, init failed");

    /* copy configuration */
    out_handle->dev_config = *ahtxx_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));

    /* attempt soft-reset */
    ESP_GOTO_ON_ERROR(ahtxx_reset(out_handle), err_handle, TAG, "soft-reset for init failed");

    /* set device handle */
    *ahtxx_handle = out_handle;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t ahtxx_get_measurement(ahtxx_handle_t handle, float *const temperature, float *const humidity) {
    const bit24_uint8_buffer_t tx = { AHTXX_CMD_TRIGGER_MEAS, AHTXX_CTRL_MEAS, AHTXX_CTRL_NOP };
    esp_err_t     ret        = ESP_OK;
    uint64_t      start_time = esp_timer_get_time();
    bool          is_busy    = true;
    bit56_uint8_buffer_t rx  = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle && (temperature || humidity) );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ahtxx_i2c_write(handle, tx, BIT24_UINT8_BUFFER_SIZE ), TAG, "write measurement trigger command for get measurement failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_MEAS_PROC_DELAY_MS));

    /* attempt to poll status until data is available or timeout occurs  */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ahtxx_get_busy_status(handle, &is_busy), err, TAG, "is busy read for get measurement failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AHTXX_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (AHTXX_DATA_POLL_TIMEOUT_MS * 1000)))
            return ESP_ERR_TIMEOUT;
    } while (is_busy == true);

    /* handle aht sensor read by type */
    if(handle->dev_config.sensor_type == AHTXX_AHT10) {
        /* aht10 returns 6 bytes */

        /* attempt i2c read transaction for aht10 sensor type */
        ESP_RETURN_ON_ERROR( ahtxx_i2c_read(handle, rx, BIT48_UINT8_BUFFER_SIZE), TAG, "read measurement data for get measurement failed" );
    } else {
        /* aht20, aht21, aht25, and aht30 return 7 bytes */

        /* attempt i2c read transaction for aht20, aht21, aht25, and aht30 sensor types */
        ESP_RETURN_ON_ERROR( ahtxx_i2c_read(handle, rx, BIT56_UINT8_BUFFER_SIZE), TAG, "read measurement data for get measurement failed" );

        /* validate crc ? */
    }

    /* concat humidity signal */
    const uint32_t humidity_sig = ((uint32_t)rx[1] << 12) | ((uint32_t)rx[2] << 4) | (rx[3] >> 4);

    /* concat temperature signal */
    const uint32_t temperature_sig = ((uint32_t)(rx[3] & 0x0f) << 16) | ((uint32_t)rx[4] << 8) | rx[5];

    /* compute and set temperature */
    *temperature = ahtxx_convert_temperature_signal(temperature_sig);

    /* compute and set humidity */
    *humidity = ahtxx_convert_humidity_signal(humidity_sig);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));
    
    return ESP_OK;

    err:
        return ret;
}

esp_err_t ahtxx_get_measurements(ahtxx_handle_t handle, float *const temperature, float *const humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && (temperature || humidity || dewpoint) );

    /* attempt to get temperature and humidity measurements */
    ESP_RETURN_ON_ERROR( ahtxx_get_measurement(handle, temperature, humidity), TAG, "read measurement for get measurements failed" );

    /* compute dew-point from temperature and humidity */
    *dewpoint = ahtxx_calculate_dewpoint(*temperature, *humidity);

    return ESP_OK;
}

esp_err_t ahtxx_get_busy_status(ahtxx_handle_t handle, bool *const busy) {
    ahtxx_status_register_t status_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle && busy );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ahtxx_get_status_register(handle, &status_reg), TAG, "read status register for busy status failed" );

    /* set output parameter */
    *busy = status_reg.bits.busy;

    //ESP_LOGD(TAG, "aht2x busy state    %s", busy ? "true" : "false");

    return ESP_OK;
}

esp_err_t ahtxx_get_calibration_status(ahtxx_handle_t handle, bool *const calibrated) {
    ahtxx_status_register_t status_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle && calibrated );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ahtxx_get_status_register(handle, &status_reg), TAG, "read status register for calibration status failed" );

    /* set output parameter */
    *calibrated = status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t ahtxx_get_status(ahtxx_handle_t handle, bool *const busy, bool *const calibrated) {
    ahtxx_status_register_t status_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle && (busy || calibrated) );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ahtxx_get_status_register(handle, &status_reg), TAG, "read status register for status failed" );

    /* set output parameters */
    *busy       = status_reg.bits.busy;
    *calibrated = status_reg.bits.calibrated;

    return ESP_OK;
}

esp_err_t ahtxx_reset(ahtxx_handle_t handle) {
    const bit8_uint8_buffer_t tx = { AHTXX_CMD_RESET };
    ahtxx_status_register_t status_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ahtxx_i2c_write(handle, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "write reset command for reset failed" );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_RESET_DELAY_MS));

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ahtxx_get_status_register(handle, &status_reg), TAG, "read status register for reset failed" );

    /* handle sensor setup by sensor type */
    if(handle->dev_config.sensor_type == AHTXX_AHT10 || handle->dev_config.sensor_type == AHTXX_AHT20) {
        /* validate calibration status */
        if(status_reg.bits.calibrated == false) {
            /* attempt to write init command */
            ESP_RETURN_ON_ERROR( ahtxx_setup(handle), TAG, "setup sensor for reset failed" );
        }
    } else {
        /* validate register status */
        if(status_reg.reg != AHTXX_STATUS_WORD) {
            /* attempt to reset initialization registers */
            ESP_RETURN_ON_ERROR( ahtxx_setup(handle), TAG, "setup sensor for reset failed" );
        }
    }

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ahtxx_get_status_register(handle, &status_reg), TAG, "read status register for reset failed" );

    /* validate calibration status */
    if(status_reg.bits.calibrated == false) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_STATE, TAG, "setup and initialize sensor for reset failed" );
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AHTXX_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t ahtxx_remove(ahtxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t ahtxx_delete(ahtxx_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( ahtxx_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* ahtxx_get_fw_version(void) {
    return AHTXX_FW_VERSION_STR;
}

int32_t ahtxx_get_fw_version_number(void) {
    return AHTXX_FW_VERSION_INT32;
}