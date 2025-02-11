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
 * @file ltr390uv_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <ltr390uv_task.h>
#include <ltr390uv.h>


void i2c0_ltr390uv_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    i2c_ltr390uv_config_t dev_cfg          = I2C_LTR390UV_CONFIG_DEFAULT;
    i2c_ltr390uv_handle_t dev_hdl;
    //
    // init device
    i2c_ltr390uv_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ltr390uv handle init failed");
        assert(dev_hdl);
    }
    //
    ESP_LOGI(APP_TAG, "Control Register (0x%02x): %s", dev_hdl->control_reg.reg, uint8_to_binary(dev_hdl->control_reg.reg));
    ESP_LOGI(APP_TAG, "Measure Register (0x%02x): %s", dev_hdl->measure_reg.reg, uint8_to_binary(dev_hdl->measure_reg.reg));
    ESP_LOGI(APP_TAG, "Gain Register    (0x%02x): %s", dev_hdl->gain_reg.reg, uint8_to_binary(dev_hdl->gain_reg.reg));
    ESP_LOGI(APP_TAG, "IRQ Cfg Register (0x%02x): %s", dev_hdl->irq_config_reg.reg, uint8_to_binary(dev_hdl->irq_config_reg.reg));
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## LTR390UV - START #########################");
        //
        // handle sensor
        
        float ambient_light; 
        esp_err_t result = i2c_ltr390uv_get_ambient_light(dev_hdl, &ambient_light);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ambient light:     %.2f Lux", ambient_light);
        }

        uint32_t sensor_counts;
        result = i2c_ltr390uv_get_als(dev_hdl, &sensor_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "als sensor counts: %lu", sensor_counts);
        }

        float uvi;
        result = i2c_ltr390uv_get_ultraviolet_index(dev_hdl, &uvi);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ultraviolet index: %f", uvi);
        }

        result = i2c_ltr390uv_get_uvs(dev_hdl, &sensor_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ltr390uv device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "uvs sensor counts: %lu", sensor_counts);
        }
        //
        ESP_LOGI(APP_TAG, "######################## LTR390UV - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    i2c_ltr390uv_delete( dev_hdl );
    vTaskDelete( NULL );
}