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
 * @file bh1750.c
 *
 * ESP-IDF driver for BH1750 illuminance sensor.
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/bh1750.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * BH1750 definitions
*/
#define BH1750_OPCODE_MT_HI         UINT8_C(0x40)   /*!< measurement time hi-bit */
#define BH1750_OPCODE_MT_LO         UINT8_C(0x60)   /*!< measurement time lo-bit */

#define BH1750_CMD_POWER_DOWN       UINT8_C(0b00000000)
#define BH1750_CMD_POWER_UP         UINT8_C(0b00000001)
#define BH1750_CMD_RESET            UINT8_C(0b00000111)

#define BH1750_POWERUP_DELAY_MS     UINT16_C(10)
#define BH1750_APPSTART_DELAY_MS    UINT16_C(10)
#define BH1750_RESET_DELAY_MS       UINT16_C(25)
#define BH1750_CMD_DELAY_MS         UINT16_C(5)
#define BH1750_RETRY_DELAY_MS       UINT16_C(2)     /*!< bh1750 delay between an I2C receive transaction retry */
#define BH1750_TX_RX_DELAY_MS       UINT16_C(10)    /*!< bh1750 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief BH1750 device structure definition.
 */
typedef struct bh1750_device_s {
    bh1750_config_t                 config;         /*!< bh1750 device configuration */ 
    i2c_master_dev_handle_t         i2c_handle;     /*!< bh1750 I2C device handle */
} bh1750_device_t;

/*
* static constant declarations
*/
static const char *TAG = "bh1750";

/*
* functions and subroutines
*/


/**
 * @brief BH1750 I2C HAL read transaction.
 * 
 * @param device BH1750 device descriptor.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_read(bh1750_device_t *const device, uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read failed" );

    return ESP_OK;
}

/**
 * @brief BH1750 I2C HAL read byte from register address transaction.
 * 
 * @param device BH1750 device descriptor.
 * @param reg_addr BH1750 register address to read from.
 * @param byte BH1750 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_read_byte_from(bh1750_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "bh1750_i2c_read_byte_from, i2c read failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief BH1750 I2C HAL write transaction.
 * 
 * @param device BH1750 device descriptor.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_write(bh1750_device_t *const device, const uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief BH1750 I2C HAL write command to register address transaction.
 * 
 * @param device BH1750 device descriptor.
 * @param reg_addr BH1750 command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_write_command(bh1750_device_t *const device, const uint8_t reg_addr) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Gets BH1750 measurement duration in milli-seconds from device handle.  See datasheet for details.
 *
 * @param[in] device BH1750 device descriptor.
 * @return duration in milliseconds.
 */
static inline uint32_t bh1750_get_duration(bh1750_device_t *const device) {
    /* validate arguments */
    if (!device) return 180;
    /* todo - duration when measurement time is modified */
    switch (device->config.mode) {
        case BH1750_MODE_OM_HI_RESOLUTION:
            return 180;
        case BH1750_MODE_OM_HI2_RESOLUTION:
            return 180;
        case BH1750_MODE_OM_LO_RESOLUTION:
            return 25;
        case BH1750_MODE_CM_HI_RESOLUTION:
            return 180;
        case BH1750_MODE_CM_HI2_RESOLUTION:
            return 180;
        case BH1750_MODE_CM_LO_RESOLUTION:
            return 25;
        default:
            return 180;
    }
}

/**
 * @brief Gets bh1750 measurement tick duration from device handle.
 *
 * @param[in] device BH1750 device descriptor.
 * @return duration in ticks.
 */
static inline uint32_t bh1750_get_tick_duration(bh1750_device_t *const device) {
    /* validate arguments */
    if (!device) return 0;
    uint32_t res = pdMS_TO_TICKS(bh1750_get_duration(device));
    return res == 0 ? 1 : res;
}

/**
 * @brief Reads the current measurement mode from the BH1750 device.
 * 
 * @param device BH1750 device descriptor.
 * @param mode Pointer to store the current measurement mode
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_get_measurement_mode_register(bh1750_device_t *const device, bh1750_measurement_modes_t *const mode) {
    /* validate arguments */
    ESP_ARG_CHECK( device && mode );

    /* attempt to read measurement mode */
    ESP_RETURN_ON_ERROR( bh1750_i2c_read_byte_from(device, 0x00, (uint8_t *)mode), TAG, "unable to read measurement mode from device, get measurement mode failed" );

    /* update device config */
    device->config.mode = *mode;

    return ESP_OK;
}

/**
 * @brief Writes measurement mode to bh1750.
 *
 * @param[in] device bh1750 device handle.
 * @param[in] mode bh1750 measurement mode.
 * @return ESP_OK on success.
 */
static inline esp_err_t bh1750_set_measurement_mode_register(bh1750_device_t *const device, const bh1750_measurement_modes_t mode) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, mode), TAG, "unable to write measurement mode to device, set measurement mode failed");

    /* update device config */
    device->config.mode = mode;

    return ESP_OK;
}

/**
 * @brief Reads the current measurement time from the BH1750 device.
 * 
 * @param device BH1750 device descriptor.
 * @param timespan Pointer to store the current measurement time duration
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_get_measurement_time_register(bh1750_device_t *const device, uint8_t *const timespan) {
    /* validate arguments */
    ESP_ARG_CHECK( device && timespan );

    uint8_t mt_hi = 0;
    uint8_t mt_lo = 0;

    /* attempt to read measurement time hi and lo */
    ESP_RETURN_ON_ERROR( bh1750_i2c_read_byte_from(device, BH1750_OPCODE_MT_HI, &mt_hi), TAG, "unable to read measurement time hi from device, get measurement time register failed" );
    ESP_RETURN_ON_ERROR( bh1750_i2c_read_byte_from(device, BH1750_OPCODE_MT_LO, &mt_lo), TAG, "unable to read measurement time lo from device, get measurement time register failed" );

    /* set output parameter */
    *timespan = (uint8_t)(((mt_hi & 0x1F) << 5) | (mt_lo & 0x1F));

    return ESP_OK;
}

/**
 * @brief Writes measurement time to bh1750.
 *
 * @param[in] device bh1750 device handle.
 * @param[in] timespan bh1750 measurement time duration.
 * @return ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_set_measurement_time_register(bh1750_device_t *const device, const uint8_t timespan) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to write measurement hi and lo timespan */
    ESP_RETURN_ON_ERROR( bh1750_i2c_write_command(device, BH1750_OPCODE_MT_HI | (timespan >> 5)), TAG, "unable to write measurement time hi to device, set measurement time register failed" );
    ESP_RETURN_ON_ERROR( bh1750_i2c_write_command(device, BH1750_OPCODE_MT_LO | (timespan >> 0x1f)), TAG, "unable to write measurement time lo to device, set measurement time register failed" );

    /* update device config */
    device->config.timespan = timespan;

    return ESP_OK;
}

/**
 * @brief Writes soft-reset command to BH1750 register.
 * 
 * @param device BH1750 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_set_reset_register(bh1750_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, BH1750_CMD_RESET), TAG, "write soft-reset command failed");

    /* delay before next command - power cycle */
    vTaskDelay(pdMS_TO_TICKS(BH1750_RESET_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Setup and configure BH1750 registers.
 * 
 * @param device BH1750 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t bh1750_i2c_setup_registers(bh1750_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate power status */
    if(device->config.power_enabled == true) {
        /* attempt i2c write transaction */
        ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, BH1750_CMD_POWER_UP), TAG, "unable to power-up device, bh1750 device setup failed");
    }

    /* validate measurement time */
    if(device->config.set_timespan == true) {
        /* attempt to write measurement hi and lo timespan */
        ESP_RETURN_ON_ERROR( bh1750_i2c_write_command(device, BH1750_OPCODE_MT_HI | (device->config.timespan >> 5)), TAG, "unable to write measurement time hi to device, bh1750 device setup failed" );
        ESP_RETURN_ON_ERROR( bh1750_i2c_write_command(device, BH1750_OPCODE_MT_LO | (device->config.timespan >> 0x1f)), TAG, "unable to write measurement time lo to device, bh1750 device setup failed" );
    }

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, device->config.mode), TAG, "unable to write measurement mode to device, bh1750 device setup failed");

    return ESP_OK;
}

esp_err_t bh1750_init(i2c_master_bus_handle_t master_handle, const bh1750_config_t *bh1750_config, bh1750_handle_t *bh1750_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && bh1750_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(BH1750_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, bh1750_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, bh1750 device handle initialization failed", bh1750_config->i2c_address);

    /* validate memory availability for handle */
    bh1750_device_t* device = (bh1750_device_t*)calloc(1, sizeof(bh1750_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for device, bh1750 device handle initialization failed");

    /* copy configuration */
    device->config = *bh1750_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "unable to add device to master bus, bh1750 device handle initialization failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(BH1750_CMD_DELAY_MS));

    /* attempt to reset the device */
    ESP_GOTO_ON_ERROR(bh1750_i2c_set_reset_register(device), err_handle, TAG, "unable to soft-reset device, bh1750 device handle initialization failed");

    /* attempt to setup the device */
    ESP_GOTO_ON_ERROR(bh1750_i2c_setup_registers(device), err_handle, TAG, "unable to setup device, bh1750 device handle initialization failed");

    /* set device handle */
    *bh1750_handle = (bh1750_handle_t)device;

    /* application start delay */
    vTaskDelay(pdMS_TO_TICKS(BH1750_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t bh1750_get_ambient_light(bh1750_handle_t handle, float *const ambient_light) {
    const uint8_t rx_retry_max  = 5;
    uint8_t rx_retry_count      = 0;
    esp_err_t ret               = ESP_OK;
    bh1750_device_t* device     = (bh1750_device_t*)handle;
    bit16_uint8_buffer_t rx     = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    const bit8_uint8_buffer_t tx = { device->config.mode };
    
    /* set delay */
    const uint32_t delay_ticks = bh1750_get_tick_duration(device);

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write(device, tx, BIT8_UINT8_BUFFER_SIZE), TAG, "unable to write measurement mode command to device, get measurement failed");

    /* delay task - allow time for the sensor to process measurement request */
    if(delay_ticks) vTaskDelay(delay_ticks);

    /* retry needed - unexpected nack indicates that the sensor is still busy */
    do {
        /* attempt i2c read transaction */
        ret = bh1750_i2c_read(device, rx, BIT16_UINT8_BUFFER_SIZE);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(BH1750_RETRY_DELAY_MS));
    } while (ret != ESP_OK && ++rx_retry_count <= rx_retry_max);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, get measurement failed" );

    /* concat hi and low bytes */
    const uint16_t val = rx[0] << 8 | rx[1];
    
    /* convert bh1750 results to engineering units of measure (lux) */
    *ambient_light = (float)(val * 10.0f) / 12.0f;

    /* set handle power status */
     if(device->config.mode == BH1750_MODE_OM_HI_RESOLUTION ||
        device->config.mode == BH1750_MODE_OM_HI2_RESOLUTION ||
        device->config.mode == BH1750_MODE_OM_LO_RESOLUTION) ESP_RETURN_ON_ERROR(bh1750_disable_power(handle), TAG, "disable power failed");

    return ESP_OK;
}

esp_err_t bh1750_get_clearness_index(bh1750_handle_t handle, float *const index) {
    float ambient_light = 0.0f;
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /*
    $K_t > 0.75$: Clear Sky (Sunny)
    $0.25 < K_t < 0.75$: Partly Cloudy
    $K_t < 0.25$: Overcast
    
    To use this, a system needs to know your exact latitude, longitude, and time of day to calculate where the sun is and how bright it "should" be.
     */

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to get ambient light */
    ESP_RETURN_ON_ERROR( bh1750_get_ambient_light(handle, &ambient_light), TAG, "unable to get ambient light, get clearness index failed" );

    /* calculate clearness index */
    *index = ambient_light / 120000.0f;

    return ESP_OK;
}

esp_err_t bh1750_set_measurement_mode(bh1750_handle_t handle, const bh1750_measurement_modes_t mode) {
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to write measurement mode */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, mode), TAG, "write measurement mode command failed");

    /* set handle measurement mode parameter */
    device->config.mode = mode;

    ESP_LOGD(TAG, "i2c_bh1750_set_measurement_mode (VAL = 0x%02x)", mode);

    /* set handle power status */
     if(device->config.mode == BH1750_MODE_OM_HI_RESOLUTION ||
        device->config.mode == BH1750_MODE_OM_HI2_RESOLUTION ||
        device->config.mode == BH1750_MODE_OM_LO_RESOLUTION) ESP_RETURN_ON_ERROR(bh1750_disable_power(handle), TAG, "disable power failed");

    return ESP_OK;
}

esp_err_t bh1750_set_measurement_time(bh1750_handle_t handle, const uint8_t timespan) {
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate timespan */
    if(timespan < 31 || timespan > 254) return ESP_ERR_INVALID_ARG;

    /* attempt to write measurement hi and lo timespan */
    ESP_ERROR_CHECK( bh1750_i2c_write_command(device, BH1750_OPCODE_MT_HI | (timespan >> 5)) );
    ESP_ERROR_CHECK( bh1750_i2c_write_command(device, BH1750_OPCODE_MT_LO | (timespan >> 0x1f)) );

    /* set handle measurement timespan parameter */
    device->config.timespan = timespan;

    return ESP_OK;
}

esp_err_t bh1750_enable_power(bh1750_handle_t handle) {
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, BH1750_CMD_POWER_UP), TAG, "write power-up command failed");

    /* delay before next command - power cycle */
    //vTaskDelay(pdMS_TO_TICKS(BH1750_POWERUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t bh1750_disable_power(bh1750_handle_t handle) {
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(bh1750_i2c_write_command(device, BH1750_CMD_POWER_DOWN), TAG, "write power-down command failed");

    /* delay before next command - power cycle */
    //vTaskDelay(pdMS_TO_TICKS(BH1750_POWERUP_DELAY_MS));

    return ESP_OK;
}

esp_err_t bh1750_reset(bh1750_handle_t handle) {
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR(bh1750_i2c_set_reset_register(device), TAG, "write soft-reset command failed");

    /* attempt to setup device */
    ESP_RETURN_ON_ERROR(bh1750_i2c_setup_registers(device), TAG, "unable to setup device, bh1750 device reset failed");

    return ESP_OK;
}

esp_err_t bh1750_remove(bh1750_handle_t handle) {
    bh1750_device_t* device = (bh1750_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate handle instance */
    if(device->i2c_handle) {
        /* remove device from i2c master bus */
        esp_err_t ret = i2c_master_bus_rm_device(device->i2c_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_rm_device failed");
            return ret;
        }
        device->i2c_handle = NULL;
    }

    return ESP_OK;
}

esp_err_t bh1750_delete(bh1750_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    esp_err_t ret = bh1750_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}

const char* bh1750_get_fw_version(void) {
    return (const char*)BH1750_FW_VERSION_STR;
}

int32_t bh1750_get_fw_version_number(void) {
    return (int32_t)BH1750_FW_VERSION_INT32;
}