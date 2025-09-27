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
 * @file ina226_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <ina226_task.h>

/*

INA266 MJKDZ Board http://www.mjkdz.com/mjkdz/products/17051533.html

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


void i2c0_ina226_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ina226_config_t dev_cfg          = INA226_CONFIG_DEFAULT;
    ina226_handle_t dev_hdl;
    //
    // init device
    ina226_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ina226 handle init failed");
        assert(dev_hdl);
    }

    ina226_config_register_t config;
    ina226_get_configuration_register(dev_hdl, &config);
    ESP_LOGI(APP_TAG, "Configuration (0x%04x): %s", config.reg, uint16_to_binary(config.reg));
    
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## INA226 - START #########################");
        
        // handle sensor
        float bus_voltage, shunt_voltage, power, current;
        esp_err_t result = ina226_get_bus_voltage(dev_hdl, &bus_voltage);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ina226 device read bus voltage failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "bus voltage:     %.2f V", bus_voltage);
        }
        result = ina226_get_shunt_voltage(dev_hdl, &shunt_voltage);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ina226 device read shunt voltage failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "shunt voltage:   %.2f mV", shunt_voltage * 1000);
        }
        result = ina226_get_current(dev_hdl, &current);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ina226 device current failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "current:         %.2f mA", current * 1000);
        }
        result = ina226_get_power(dev_hdl, &power);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ina226 device power failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "power:           %.2f mW", power * 1000);
        }

        
        ESP_LOGI(APP_TAG, "######################## INA226 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ina226_delete( dev_hdl );
    vTaskDelete( NULL );
}