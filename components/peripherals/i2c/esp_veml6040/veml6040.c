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
 * @file veml6040.c
 *
 * ESP-IDF driver for VEML6040 colour sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/veml6040.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * VEML6040 definitions
*/

#define VEML6040_POLY_COEF_A        (6.0135e-13)
#define VEML6040_POLY_COEF_B        (-9.3924e-9)
#define VEML6040_POLY_COEF_C        (8.1488e-5)
#define VEML6040_POLY_COEF_D        (1.0023)

#define VEML6040_CMD_CONF           UINT8_C(0x00)
#define VEML6040_CMD_R_DATA         UINT8_C(0x08)
#define VEML6040_CMD_G_DATA         UINT8_C(0x09)
#define VEML6040_CMD_B_DATA         UINT8_C(0x0a)
#define VEML6040_CMD_W_DATA         UINT8_C(0x0b)

#define VEML6040_POWERUP_DELAY_MS   UINT16_C(5)     /*!< veml7700 delay on power-up before attempting I2C transactions */
#define VEML6040_APPSTART_DELAY_MS  UINT16_C(10)    /*!< veml7700 delay after initialization before application start-up */
#define VEML6040_CMD_DELAY_MS       UINT16_C(5)     /*!< veml7700 delay before attempting I2C transactions after a command is issued */
#define VEML6040_RETRY_DELAY_MS     UINT16_C(2)     /*!< veml7700 delay between an I2C receive transaction retry */
#define VEML6040_TX_RX_DELAY_MS     UINT16_C(10)    /*!< veml7700 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define VEML6040_IT_TIMES_COUNT     UINT8_C(6)      /*!< Possible integration time values count */
#define VEML6040_IT_OPTIONS_COUNT   UINT8_C(4)      /*!< Possible integration time values count */

#define VEML6040_IT_OPT_ENUM_INDEX  UINT8_C(0)      /*!< integration time enumerator */
#define VEML6040_IT_OPT_IT_INDEX    UINT8_C(1)      /*!< integration time in milliseconds */
#define VEML6040_IT_OPT_SVTY_INDEX  UINT8_C(2)      /*!< integration time gain sensitivity */
#define VEML6040_IT_OPT_MXLX_INDEX  UINT8_C(3)      /*!< integration time maximum lux value */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief VEML6040 device descriptor structure definition.
 */
typedef struct veml6040_device_s {
    veml6040_config_t                   config;                 /*!< veml6040 device configuration */
    i2c_master_dev_handle_t             i2c_handle;             /*!< veml6040 i2c device handle */
} veml6040_device_t;

/*
* static constant declarations
*/
static const char *TAG = "veml6040";


/**
 * @brief List of all possible values for configuring sensor integration time.
 * 
 * Options: integration time (enum), integration time (ms), g-sensitivity, max. detectable lux
 */
static const float veml6040_integration_time_map[VEML6040_IT_TIMES_COUNT][VEML6040_IT_OPTIONS_COUNT] = {
    {VEML6040_INTEGRATION_TIME_40MS,   40,   0.25168,  16496 },
    {VEML6040_INTEGRATION_TIME_80MS,   80,   0.12584,  8248 },
    {VEML6040_INTEGRATION_TIME_160MS,  160,  0.06292,  4124 },
    {VEML6040_INTEGRATION_TIME_320MS,  320,  0.03146,  2062 },
    {VEML6040_INTEGRATION_TIME_640MS,  640,  0.01573,  1031 },
    {VEML6040_INTEGRATION_TIME_1280MS, 1280, 0.007865, 515.4 }
};


/**
 * @brief Converts VEML6040 channel signal to illuminance (lux).
 * 
 * @param device VEML6040 device descriptor.
 * @param signal VEML6040 channel signal to convert
 * @param lux Converted VEML6040 channel signal to illuminance (lux).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t veml6040_convert_signal(veml6040_device_t *const device, const uint16_t signal, float *const lux) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* get gain sensitivity from map */
    float g_sensitivity = veml6040_integration_time_map[device->config.integration_time][VEML6040_IT_OPT_SVTY_INDEX];

    /* convert signal to lux */
    *lux = (float)signal * g_sensitivity;

    return ESP_OK;
}

/**
 * @brief VEML7700 I2C HAL read word from register address transaction.
 * 
 * @param device VEML6040 device descriptor.
 * @param reg_addr VEML7700 register address to read from.
 * @param word VEML7700 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t veml6040_i2c_read_word_from(veml6040_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "hdc1080_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief VEML7700 I2C HAL write word to register address transaction.
 * 
 * @param device VEML6040 device descriptor.
 * @param reg_addr VEML7700 register address to write to.
 * @param word VEML7700 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t veml6040_i2c_write_word_to(veml6040_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}


/**
 * @brief Reads signal from VEML6050 by channel.
 * 
 * @param device VEML6040 device descriptor.
 * @param channel VEML6040 signal channel to read.
 * @param signal VEML6040 signal from channel.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t veml6040_get_signal(veml6040_device_t *const device, const veml6040_channels_t channel, uint16_t *const signal) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* get integration time from map i.e. time to wait for measurement */
    float it_ms = veml6040_integration_time_map[device->config.integration_time][VEML6040_IT_OPT_IT_INDEX];

    /* wait for measurement */
    vTaskDelay(pdMS_TO_TICKS( it_ms ));

    switch(channel) {
        case VEML6040_CHANNEL_RED:
            /* attempt i2c read transaction */
            ESP_RETURN_ON_ERROR( veml6040_i2c_read_word_from(device, VEML6040_CMD_R_DATA, signal), TAG, "read red signal for get signal failed" );
            break;
        case VEML6040_CHANNEL_GREEN:
            /* attempt i2c read transaction */
            ESP_RETURN_ON_ERROR( veml6040_i2c_read_word_from(device, VEML6040_CMD_G_DATA, signal), TAG, "read green signal for get signal failed" );
            break;
        case VEML6040_CHANNEL_BLUE:
            /* attempt i2c read transaction */
            ESP_RETURN_ON_ERROR( veml6040_i2c_read_word_from(device, VEML6040_CMD_B_DATA, signal), TAG, "read blue signal for get signal failed" );
            break;
        case VEML6040_CHANNEL_WHITE:
            /* attempt i2c read transaction */
            ESP_RETURN_ON_ERROR( veml6040_i2c_read_word_from(device, VEML6040_CMD_W_DATA, signal), TAG, "read white signal for get signal failed" );
            break;
        default:
            ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "invalid channel (red, gree, blue or white), get signal failed" );
            break;
    }

    return ESP_OK;
}

esp_err_t veml6040_get_configuration_register(veml6040_handle_t handle, veml6040_config_register_t *const reg) {
    uint16_t config = 0;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_i2c_read_word_from(dev, VEML6040_CMD_CONF, &config), TAG, "read configuration register for get configuration register failed" );

    /* set output parameter */
    reg->reg = config;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML6040_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml6040_set_configuration_register(veml6040_handle_t handle, const veml6040_config_register_t reg) {
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    veml6040_config_register_t config = { .reg = reg.reg };

    /* set reserved values */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;
    config.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml6040_i2c_write_word_to(dev, VEML6040_CMD_CONF, config.reg), TAG, "write configuration register for set configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML6040_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml6040_init(i2c_master_bus_handle_t master_handle, const veml6040_config_t *veml6040_config, veml6040_handle_t *veml6040_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && veml6040_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(VEML6040_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, veml6040_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, veml6040 device handle initialization failed", veml6040_config->i2c_address);

    /* validate memory availability for handle */
    veml6040_device_t* dev = (veml6040_device_t*)calloc(1, sizeof(veml6040_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for device, veml6040 device handle initialization failed");

    /* copy configuration */
    dev->config = *veml6040_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c0 new bus failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML6040_CMD_DELAY_MS));

    /* attempt to read initialization registers */
    veml6040_config_register_t       cfg_reg;
    
    /* attempt to read configuration register */
    ESP_GOTO_ON_ERROR(veml6040_get_configuration_register((veml6040_handle_t)dev, &cfg_reg), err_handle, TAG, "read configuration register failed");

    /* set configuration register */
    cfg_reg.bits.integration_time   = dev->config.integration_time;
    cfg_reg.bits.mode               = dev->config.mode;
    cfg_reg.bits.trigger            = dev->config.trigger_method;
    cfg_reg.bits.shutdown_enabled   = dev->config.shutdown_enabled;

    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(veml6040_set_configuration_register((veml6040_handle_t)dev, cfg_reg), err_handle, TAG, "write configuration register failed");

    /* set device handle */
    *veml6040_handle = (veml6040_handle_t)dev;

    /* application start delay  */
    vTaskDelay(pdMS_TO_TICKS(VEML6040_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t veml6040_get_red_als(veml6040_handle_t handle, float *const als) {
    uint16_t signal;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_signal(dev, VEML6040_CHANNEL_RED, &signal), TAG, "read red signal for get red als failed" );

    /* convert signal */
    ESP_RETURN_ON_ERROR( veml6040_convert_signal(dev, signal, als), TAG, "convert red signal for get red als failed" );

    return ESP_OK;
}

esp_err_t veml6040_get_green_als(veml6040_handle_t handle, float *const als) {
    uint16_t signal;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_signal(dev, VEML6040_CHANNEL_GREEN, &signal), TAG, "read green signal for get green als failed" );

    /* convert signal */
    ESP_RETURN_ON_ERROR( veml6040_convert_signal(dev, signal, als), TAG, "convert green signal for get green als failed" );

    return ESP_OK;
}

esp_err_t veml6040_get_blue_als(veml6040_handle_t handle, float *const als) {
    uint16_t signal;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_signal(dev, VEML6040_CHANNEL_BLUE, &signal), TAG, "read blue signal for get blue als failed" );

    /* convert signal */
    ESP_RETURN_ON_ERROR( veml6040_convert_signal(dev, signal, als), TAG, "convert blue signal for get blue als failed" );

    return ESP_OK;
}

esp_err_t veml6040_get_white_als(veml6040_handle_t handle, float *const als) {
    uint16_t signal;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_signal(dev, VEML6040_CHANNEL_WHITE, &signal), TAG, "read white signal for get white als failed" );

    /* convert signal */
    ESP_RETURN_ON_ERROR( veml6040_convert_signal(dev, signal, als), TAG, "convert white signal for get white als failed" );

    return ESP_OK;
}

esp_err_t veml6040_get_als(veml6040_handle_t handle, float *const red_als, float *const green_als, float *const blue_als, float *const white_als) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transactions */
    ESP_RETURN_ON_ERROR( veml6040_get_red_als(handle, red_als), TAG, "read red als for get als failed" );
    ESP_RETURN_ON_ERROR( veml6040_get_green_als(handle, green_als), TAG, "read green als for get als failed" );
    ESP_RETURN_ON_ERROR( veml6040_get_blue_als(handle, blue_als), TAG, "read blue als for get als failed" );
    ESP_RETURN_ON_ERROR( veml6040_get_white_als(handle, white_als), TAG, "read white als for get als failed" );

    return ESP_OK;
}

esp_err_t veml6040_get_integration_time(veml6040_handle_t handle, veml6040_integration_times_t *const integration_time) {
    veml6040_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "write configuration register for get integration time failed" );

    *integration_time = config.bits.integration_time;

    return ESP_OK;
}

esp_err_t veml6040_set_integration_time(veml6040_handle_t handle, const veml6040_integration_times_t integration_time) {
    veml6040_config_register_t config;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "write configuration register for set integration time failed" );

    /* set parameters */
    config.bits.integration_time = integration_time;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml6040_set_configuration_register(handle, config), TAG, "write configuration register for set integration time failed" );

    /* set config parameter */
    dev->config.integration_time = config.bits.integration_time;

    return ESP_OK;
}

esp_err_t veml6040_get_trigger_method(veml6040_handle_t handle, veml6040_triggers_t *const trigger_method) {
    veml6040_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "write configuration register for get trigger method failed" );

    *trigger_method = config.bits.trigger;

    return ESP_OK;
}

esp_err_t veml6040_set_trigger_method(veml6040_handle_t handle, const veml6040_triggers_t trigger_method) {
    veml6040_config_register_t config;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "write configuration register for set trigger method failed" );

    /* set parameters */
    config.bits.trigger = trigger_method;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml6040_set_configuration_register(handle, config), TAG, "write configuration register for set trigger method failed" );

    /* set config parameter */
    dev->config.trigger_method = config.bits.trigger;

    return ESP_OK;
}

esp_err_t veml6040_get_mode(veml6040_handle_t handle, veml6040_modes_t *const mode) {
    veml6040_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "write configuration register for get mode failed" );

    *mode = config.bits.mode;

    return ESP_OK;
}

esp_err_t veml6040_set_mode(veml6040_handle_t handle, const veml6040_modes_t mode) {
    veml6040_config_register_t config;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "write configuration register for set mode failed" );

    /* set parameters */
    config.bits.mode = mode;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml6040_set_configuration_register(handle, config), TAG, "write configuration register for set mode failed" );

    /* set config parameter */
    dev->config.mode = config.bits.mode;

    return ESP_OK;
}

esp_err_t veml6040_disable(veml6040_handle_t handle) {
    veml6040_config_register_t config;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "read configuration register for shutdown failed" );

    /* shutdown device */
    config.bits.shutdown_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml6040_set_configuration_register(handle, config), TAG, "write configuration register for shutdown failed" );

    /* set config parameter */
    dev->config.shutdown_enabled = config.bits.shutdown_enabled;

    return ESP_OK;
}

esp_err_t veml6040_enable(veml6040_handle_t handle) {
    veml6040_config_register_t config;
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml6040_get_configuration_register(handle, &config), TAG, "read configuration register for wake-up failed" );

    /* wakeup device */
    config.bits.shutdown_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml6040_set_configuration_register(handle, config), TAG, "write configuration register for wake-up failed" );

    /* set config parameter */
    dev->config.shutdown_enabled = config.bits.shutdown_enabled;

    return ESP_OK;
}

esp_err_t veml6040_remove(veml6040_handle_t handle) {
    veml6040_device_t* dev = (veml6040_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t veml6040_delete(veml6040_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( veml6040_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* veml6040_get_fw_version(void) {
    return (const char*)VEML6040_FW_VERSION_STR;
}

int32_t veml6040_get_fw_version_number(void) {
    return (int32_t)VEML6040_FW_VERSION_INT32;
}