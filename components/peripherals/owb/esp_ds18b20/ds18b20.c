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
 * @file ds18b20.c
 *
 * ESP-IDF driver for DS18B20 temperature sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ds18b20.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <onewire_cmd.h>
#include <onewire_crc.h>

/*
 * DS18B20 definitions
 */

#define DS18B20_CMD_TEMP_CONVERT        UINT8_C(0x44)   /*!< initiate a single temperature conversion */
#define DS18B20_CMD_SCRATCHPAD_WRITE    UINT8_C(0x4E)   /*!< write 3 bytes of data to the device scratchpad at positions 2, 3 and 4 */
#define DS18B20_CMD_SCRATCHPAD_READ     UINT8_C(0xBE)   /*!< read 9 bytes of data (including CRC) from the device scratchpad */
#define DS18B20_CMD_SCRATCHPAD_COPY     UINT8_C(0x48)   /*!< copy the contents of the scratchpad to the device EEPROM */
#define DS18B20_CMD_EEPROM_RECALL       UINT8_C(0xB8)   /*!< restore alarm trigger values and configuration data from EEPROM to the scratchpad */
#define DS18B20_CMD_POWER_SUPPLY_READ   UINT8_C(0xB4)   /*!< determine if a device is using parasitic power */

#define DS18B20_DEVICE_MAX              UINT16_C(10)    /*!< maximum number of ds18b20 devices on the 1-wire bus */

#define DS18B20_POWERUP_DELAY_MS        UINT16_C(20)
#define DS18B20_RESET_DELAY_MS          UINT16_C(25)
#define DS18B20_APPSTART_DELAY_MS       UINT16_C(10)    /*!< ds18b20 delay after initialization before application start-up */
#define DS18B20_EEPROM_WRITE_DELAY_MS   UINT16_C(15)


/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief DS18B20 device descriptor structure definition.
 */
typedef struct ds18b20_device_s {
    ds18b20_config_t          config;       /*!< ds18b20 device configuration */
    onewire_bus_handle_t      owb_handle;   /*!< ds18b20 1-wire bus handle */
    onewire_device_address_t  owb_address;  /*!< ds18b20 1-wire device address */
} ds18b20_device_t;

/*
* static constant declarations
*/
static const char *TAG = "ds18b20";

/**
 * @brief Checks scratchpad to determine if it is valid.
 * 
 * @param scratchpad DS18B20 scratchpad structure.
 * @return bool DS18B20 scratchpad is valid when true.
 */
static inline bool ds18b20_validate_scratchpad(const ds18b20_scratchpad_t scratchpad) {
    if(scratchpad.configuration == 0 && scratchpad.crc == 0 && scratchpad.reserved1 == 0 &&
        scratchpad.reserved2 == 0 && scratchpad.reserved3 == 0 && scratchpad.temp_lsb == 0 &&
        scratchpad.temp_msb == 0 && scratchpad.trigger_high == 0 && scratchpad.trigger_low == 0) {
        return false;
    }

    return true;
}

/**
 * @brief Writes command to DS18B20.
 * 
 * @param handle DS18B20 device handle.
 * @param cmd DS18B20 command value.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ds18b20_send_command(ds18b20_device_t *const device, const uint8_t cmd) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    // build command
    uint8_t tx_buffer[10] = {0};
    tx_buffer[0] = ONEWIRE_CMD_MATCH_ROM;
    memcpy(&tx_buffer[1], &device->owb_address, sizeof(device->owb_address));
    tx_buffer[sizeof(device->owb_address) + 1] = cmd;

    /* write command */
    ESP_RETURN_ON_ERROR( onewire_bus_write_bytes(device->owb_handle, tx_buffer, sizeof(tx_buffer)), TAG, "unable to write bytes, send command failed" );

    return ESP_OK;
}

bool ds18b20_validate_address(const onewire_device_address_t address) {
    /* validate device address is a ds18b20, the family code of DS18B20 is 0x28 */
    if ((address & 0xFF) == 0x28) return true;

    return false;
}

esp_err_t ds18b20_detect(onewire_bus_handle_t owb_handle, onewire_device_t *const devices, const uint8_t device_size, uint8_t *const device_count) {
    onewire_device_iter_handle_t dev_iter_hdl; 

    /* validate arguments */
    ESP_ARG_CHECK( owb_handle && devices );

    /* validate size of array */
    ESP_RETURN_ON_FALSE( device_size <= DS18B20_DEVICE_MAX, ESP_ERR_INVALID_SIZE, TAG, "maximum number of devices that can be detected is 10, ds18b20 device detect failed" );

    /* instantiate 1-wire device iterator handle */
    ESP_RETURN_ON_ERROR( onewire_new_device_iter(owb_handle, &dev_iter_hdl), TAG, "unable to instantiate 1-wire device iterator, ds18b20 device detect failed" );

    /* init device index and count */
    uint8_t dev_iter_index = 0;
    *device_count = 0;

    /* iterate and detect devices */
    for(uint8_t i = 0; i < device_size; i++) {
        /* validate 1-wire device iterator results */
        if (onewire_device_iter_get_next(dev_iter_hdl, &devices[dev_iter_index]) == ESP_OK) { // found a new device, but is it a DS18B20
            /* validate device found on the 1-wire bus is a ds18b20 */
            if(ds18b20_validate_address(devices[dev_iter_index].address) == true) {
                /* increment device index and count */
                ++dev_iter_index;
                *device_count = *device_count + 1;
            }
        }
    }

    // free device iter handle
    ESP_RETURN_ON_ERROR( onewire_del_device_iter(dev_iter_hdl), TAG, "unable to delete 1-wire device iterator, ds18b20 device detect failed" );

    return ESP_OK;
}

esp_err_t ds18b20_connected(ds18b20_handle_t handle, bool *const connected) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, connected failed" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_READ), TAG, "unable to send OWB_DS18B20_CMD_SCRATCHPAD_READ command, connected failed" );

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(dev->owb_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "unable to read scratchpad data, connected failed" );

    // validate scratchpad and crc
    if(ds18b20_validate_scratchpad(scratchpad) == true && onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc) {
        *connected = true;
    } else {
        *connected = false;
    }

    return ESP_OK;
}

esp_err_t ds18b20_init(onewire_device_t *device, const ds18b20_config_t *ds18b20_config, ds18b20_handle_t *ds18b20_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( device && ds18b20_config );

    /* check ROM ID, the family code of DS18B20 is 0x28 */
    ESP_RETURN_ON_FALSE( ds18b20_validate_address(device->address), ESP_ERR_NOT_SUPPORTED, TAG, "%016llX is not a ds18b20 device, ds18b20 device handle initialization failed", device->address );

    /* validate memory availability for handle */
    ds18b20_device_t* dev = (ds18b20_device_t*)calloc(1, sizeof(ds18b20_device_t));
    ESP_RETURN_ON_FALSE( dev, ESP_ERR_NO_MEM, TAG, "no memory for device, ds18b20 device handle initialization failed" );

    /* copy configuration */
	dev->owb_handle  = device->bus;
    dev->owb_address = device->address;
    dev->config      = *ds18b20_config;

    /* set temperature resolution */
    ESP_RETURN_ON_ERROR( ds18b20_set_resolution((ds18b20_handle_t)dev, dev->config.resolution), TAG, "unable to write temperature resolution, ds18b20 device handle initialization failed" );

    /* set trigger thresholds if enabled */
    if(ds18b20_config->trigger_enabled == true) {
        ESP_RETURN_ON_ERROR( ds18b20_set_alarm_thresholds((ds18b20_handle_t)dev, dev->config.trigger_high, dev->config.trigger_low), TAG, "unable to write trigger thresholds, ds18b20 device handle initialization failed" );
    }

    /* set device handle */
    *ds18b20_handle = (ds18b20_handle_t)dev;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(DS18B20_APPSTART_DELAY_MS));

    return ESP_OK;
}

esp_err_t ds18b20_get_temperature(ds18b20_handle_t handle, float *const temperature) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* trigger temperature conversion */
    ESP_RETURN_ON_ERROR( ds18b20_trigger_temperature_conversion(handle), TAG, "unable to trigger temperature conversion, get measurement failed" );

    /* read temperature */
    ESP_RETURN_ON_ERROR( ds18b20_get_measurement(handle, temperature), TAG, "unable to read temperature, get measurement failed" );

    return ESP_OK;
}

esp_err_t ds18b20_get_measurement(ds18b20_handle_t handle, float *const temperature) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "reset bus error" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_READ), TAG, "send DS18B20_CMD_READ_SCRATCHPAD failed" );

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(dev->owb_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "error while reading scratchpad data" );

    // validate crc
    ESP_RETURN_ON_FALSE( onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc error");

    // combine the MSB and LSB into a signed 16-bit integer
    int16_t temperature_raw = (((int16_t)scratchpad.temp_msb) << 8) | scratchpad.temp_lsb;

    // init resolution multipliers
    const float resolution_multipliers[] = { 0.5f, 0.25f, 0.125f, 0.0625f };

    // convert the raw temperature to a float
    switch(dev->config.resolution) {
        case DS18B20_RESOLUTION_9BIT:
            *temperature = (float)(temperature_raw >> 3) * resolution_multipliers[dev->config.resolution];
            break;
        case DS18B20_RESOLUTION_10BIT:
            *temperature = (float)(temperature_raw >> 2) * resolution_multipliers[dev->config.resolution];
            break;
        case DS18B20_RESOLUTION_11BIT:
            *temperature = (float)(temperature_raw >> 1) * resolution_multipliers[dev->config.resolution];
            break;
        case DS18B20_RESOLUTION_12BIT:
            *temperature = (float)temperature_raw * resolution_multipliers[dev->config.resolution];
            break;
    }

    return ESP_OK;
}

esp_err_t ds18b20_trigger_temperature_conversion(ds18b20_handle_t handle) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, trigger temperature conversion failed" );

    // send command: DS18B20_CMD_CONVERT_TEMP
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_TEMP_CONVERT), TAG, "unable to send DS18B20_CMD_CONVERT_TEMP command, trigger temperature conversion failed" );

    // temperature conversion delays by resolution (9, 10, 11, 12 bit)
    const uint16_t delays_ms[] = { 100, 200, 400, 800 };

    // delay for temperature conversion - based on resolution setting
    vTaskDelay(pdMS_TO_TICKS(delays_ms[dev->config.resolution]));

    return ESP_OK;
}

esp_err_t ds18b20_get_resolution(ds18b20_handle_t handle, ds18b20_resolutions_t *const resolution) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "reset bus error" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_READ), TAG, "send DS18B20_CMD_READ_SCRATCHPAD failed" );

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(dev->owb_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "error while reading scratchpad data" );

    // validate crc
    ESP_RETURN_ON_FALSE( onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc error");

    // init configuration register
    const ds18b20_configuration_register_t cfg = { .reg = scratchpad.configuration };

    // set output parameter
    *resolution = cfg.bits.resolution;

    return ESP_OK;
}

esp_err_t ds18b20_set_resolution(ds18b20_handle_t handle, const ds18b20_resolutions_t resolution) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* ##### read existing configuration ##### */

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, set resolution failed" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_READ), TAG, "unable to send OWB_DS18B20_CMD_SCRATCHPAD_READ command, set resolution failed" );

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(dev->owb_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "unable to read scratchpad data, set resolution failed" );

    // validate crc
    ESP_RETURN_ON_FALSE( onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc is invalid, set resolution failed");

    /* ##### write updated configuration ##### */

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, set resolution failed" );

    // send command: DS18B20_CMD_WRITE_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_WRITE), TAG, "unable to send DS18B20_CMD_WRITE_SCRATCHPAD command, set resolution failed" );

    // init configuration register and buffer data
    const ds18b20_configuration_register_t cfg = { .bits.reserved1 = 1, .bits.reserved2 = 0, .bits.resolution = resolution };
    const uint8_t tx_buffer[] = { scratchpad.trigger_high, scratchpad.trigger_low, cfg.reg };

    // write resolution data to scratchpad
    ESP_RETURN_ON_ERROR( onewire_bus_write_bytes(dev->owb_handle, tx_buffer, sizeof(tx_buffer)), TAG, "unable to write resolution, set resolution failed" );

    // set handle resolution setting
    dev->config.resolution = resolution;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(DS18B20_EEPROM_WRITE_DELAY_MS));

    return ESP_OK;
}

esp_err_t ds18b20_get_alarm_thresholds(ds18b20_handle_t handle, int8_t *const high, int8_t *const low) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, get alarm thresholds failed" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_READ), TAG, "unable to send OWB_DS18B20_CMD_SCRATCHPAD_READ command, get alarm thresholds failed" );

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(dev->owb_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "unable to read scratchpad data, get alarm thresholds failed" );

    // validate crc
    ESP_RETURN_ON_FALSE( onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc is invalid, get alarm thresholds failed");

    // set output parameters
    *high = scratchpad.trigger_high;
    *low  = scratchpad.trigger_low;

    return ESP_OK;
}

esp_err_t ds18b20_set_alarm_thresholds(ds18b20_handle_t handle, const int8_t high, const int8_t low) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* validate threshold limits */
    ESP_RETURN_ON_FALSE( high > low, ESP_ERR_INVALID_ARG, TAG, "high threshold must be greater than the low threshold, set alarm thresholds failed" );
    ESP_RETURN_ON_FALSE( high >= -55 && high <= 125, ESP_ERR_INVALID_ARG, TAG, "allowable threshold range is from -55 to 125 degree Celsius, set alarm thresholds failed" );
    ESP_RETURN_ON_FALSE( low >= -55 && low <= 125, ESP_ERR_INVALID_ARG, TAG, "allowable threshold range is from -55 to 125 degree Celsius, set alarm thresholds failed" );

    /* ##### read existing configuration ##### */

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, set alarm thresholds failed" );

    // send command: DS18B20_CMD_READ_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_READ), TAG, "unable to send OWB_DS18B20_CMD_SCRATCHPAD_READ command, set alarm thresholds failed" );

    // read scratchpad data
    ds18b20_scratchpad_t scratchpad;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bytes(dev->owb_handle, (uint8_t *)&scratchpad, sizeof(scratchpad)), TAG, "unable to read scratchpad data, set alarm thresholds failed" );

    // validate crc
    ESP_RETURN_ON_FALSE( onewire_crc8(0, (uint8_t *)&scratchpad, 8) == scratchpad.crc, ESP_ERR_INVALID_CRC, TAG, "scratchpad crc is invalid, set alarm thresholds failed");

    /* ##### write updated configuration ##### */

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "unable to reset bus, set alarm thresholds failed" );

    // send command: DS18B20_CMD_WRITE_SCRATCHPAD
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_SCRATCHPAD_WRITE), TAG, "unable to send DS18B20_CMD_WRITE_SCRATCHPAD command, set alarm thresholds failed" );

    // init resolution data
    const uint8_t tx_buffer[] = { (uint8_t)high, (uint8_t)low, scratchpad.configuration };

    // write resolution data to scratchpad
    ESP_RETURN_ON_ERROR( onewire_bus_write_bytes(dev->owb_handle, tx_buffer, sizeof(tx_buffer)), TAG, "unable to write thresholds, set alarm thresholds failed" );

    // set handle parameters
    dev->config.trigger_high = high;
    dev->config.trigger_low  = low;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(DS18B20_EEPROM_WRITE_DELAY_MS));

    return ESP_OK;
}

esp_err_t ds18b20_get_power_supply_mode(ds18b20_handle_t handle, bool *const parasitic) {
    ds18b20_device_t* dev = (ds18b20_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    // reset bus and check if the ds18b20 is present
    ESP_RETURN_ON_ERROR( onewire_bus_reset(dev->owb_handle), TAG, "reset bus error" );

    // send command: OWB_DS18B20_CMD_POWER_SUPPLY_READ
    ESP_RETURN_ON_ERROR( ds18b20_send_command(dev, DS18B20_CMD_POWER_SUPPLY_READ), TAG, "send OWB_DS18B20_CMD_POWER_SUPPLY_READ failed" );

    // read power supply type
    uint8_t value = 0;
    ESP_RETURN_ON_ERROR( onewire_bus_read_bit(dev->owb_handle, &value), TAG, "read bit failed" );

    /* set output parameter */
    *parasitic = !(bool)(value & 0x01u);

    return ESP_OK;
}

esp_err_t ds18b20_delete(ds18b20_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* ds18b20_get_fw_version(void) {
    return (const char*)DS18B20_FW_VERSION_STR;
}

int32_t ds18b20_get_fw_version_number(void) {
    return (int32_t)DS18B20_FW_VERSION_INT32;
}