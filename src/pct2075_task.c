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
 * @file pct2075_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <pct2075_task.h>


void i2c0_pct2075_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    pct2075_config_t dev_cfg          = I2C_PCT2075_CONFIG_DEFAULT;
    pct2075_handle_t dev_hdl;
    //
    // init device
    pct2075_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "pct2075 handle init failed");
        assert(dev_hdl);
    }
    //
    esp_err_t result;
    // set overtemperature shutdown temperature
    float ots_temperature;
    result = pct2075_set_ots_temperature(dev_hdl, 85.6f);
    if (result != ESP_OK) {
        ESP_LOGE(APP_TAG, "pct2075 set overtemperature shutdown temperature failed (%s)", esp_err_to_name(result));
        assert(result == ESP_OK);
    }
    result = pct2075_get_ots_temperature(dev_hdl, &ots_temperature);
    if (result != ESP_OK) {
        ESP_LOGE(APP_TAG, "pct2075 get overtemperature shutdown temperature failed (%s)", esp_err_to_name(result));
        assert(result == ESP_OK);
    } else {
        ESP_LOGI(APP_TAG, "overtemperature shutdown temperature: %.2f °C", ots_temperature);
    }
    //
    // set hysteresis temperature
    float hys_temperature;
    result = pct2075_set_hys_temperature(dev_hdl, 70.7f);
    if (result != ESP_OK) {
        ESP_LOGE(APP_TAG, "pct2075 set hysteresis temperature failed (%s)", esp_err_to_name(result));
        assert(result == ESP_OK);
    }
    result = pct2075_get_hys_temperature(dev_hdl, &hys_temperature);
    if (result != ESP_OK) {
        ESP_LOGE(APP_TAG, "pct2075 get hysteresis temperature failed (%s)", esp_err_to_name(result));
        assert(result == ESP_OK);
    } else {
        ESP_LOGI(APP_TAG, "hysteresis temperature: %.2f °C", hys_temperature);
    }
    //
    // set sampling period
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## PCT2075 - START #########################");
        //
        // handle sensor
        float temperature;
        esp_err_t result = pct2075_get_temperature(dev_hdl, &temperature);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "pct2075 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
        }
        //
        ESP_LOGI(APP_TAG, "######################## PCT2075 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    pct2075_delete( dev_hdl );
    vTaskDelete( NULL );
}