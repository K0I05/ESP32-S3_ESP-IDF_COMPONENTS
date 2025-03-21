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
 * @file ens160_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <ens160_task.h>


void i2c0_ens160_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ens160_config_t dev_cfg          = I2C_ENS160_CONFIG_DEFAULT;
    ens160_handle_t dev_hdl;
    //
    // init device
    ens160_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ens160 handle init failed");
        assert(dev_hdl);
    }
    //
    uint16_t startup_time = 0; // seconds
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## ENS160 - START #########################");
        //
        // handle sensor
        ens160_validity_flags_t dev_flag;
        if(ens160_get_validity_status(dev_hdl, &dev_flag) == ESP_OK) {
            // validate device status
            if(dev_flag == ENS160_VALFLAG_NORMAL) {
                ens160_air_quality_data_t aq_data;
                esp_err_t result = ens160_get_measurement(dev_hdl, &aq_data);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "ens160 device read failed (%s)", esp_err_to_name(result));
                } else {
                    ens160_aqi_uba_row_t uba_aqi = ens160_aqi_index_to_definition(aq_data.uba_aqi);

                    ESP_LOGW(APP_TAG, "index    %1x (%s)", aq_data.uba_aqi, uba_aqi.rating);
                    ESP_LOGW(APP_TAG, "tvco     %d (0x%04x)", aq_data.tvoc, aq_data.tvoc);
                    ESP_LOGW(APP_TAG, "etoh     %d (0x%04x)", aq_data.etoh, aq_data.etoh);
                    ESP_LOGW(APP_TAG, "eco2     %d (0x%04x)", aq_data.eco2, aq_data.eco2);
                }
                //
                ens160_air_quality_raw_data_t aq_raw_data;
                result = ens160_get_raw_measurement(dev_hdl, &aq_raw_data);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "ens160 device read failed (%s)", esp_err_to_name(result));
                } else {
                    ESP_LOGW(APP_TAG, "ri-res 0 %lu | bl-res 0 %lu", aq_raw_data.hp0_ri, aq_raw_data.hp0_bl);
                    ESP_LOGW(APP_TAG, "ri-res 1 %lu | bl-res 1 %lu", aq_raw_data.hp1_ri, aq_raw_data.hp1_bl);
                    ESP_LOGW(APP_TAG, "ri-res 2 %lu | bl-res 2 %lu", aq_raw_data.hp2_ri, aq_raw_data.hp2_bl);
                    ESP_LOGW(APP_TAG, "ri-res 3 %lu | bl-res 3 %lu", aq_raw_data.hp3_ri, aq_raw_data.hp3_bl);
                }
            } else if(dev_flag == ENS160_VALFLAG_WARMUP) {
                ESP_LOGW(APP_TAG, "ens160 device is warming up (180-sec wait [%u-sec])", startup_time);
                startup_time = startup_time + I2C0_TASK_SAMPLING_RATE;
            } else if(dev_flag == ENS160_VALFLAG_INITIAL_STARTUP) {
                ESP_LOGW(APP_TAG, "ens160 device is undergoing initial starting up (3600-sec wait [%u-sec])", startup_time);
                startup_time = startup_time + I2C0_TASK_SAMPLING_RATE;
            } else if(dev_flag == ENS160_VALFLAG_INVALID_OUTPUT) {
                ESP_LOGW(APP_TAG, "ens160 device signals are giving unexpected values");
            }
        }
        //
        ESP_LOGI(APP_TAG, "######################## ENS160 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ens160_delete( dev_hdl );
    vTaskDelete( NULL );
}