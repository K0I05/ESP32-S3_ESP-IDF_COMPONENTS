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
 * @file max30105_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <max30105_task.h>


void i2c0_max30105_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    //max30105_config_t dev_cfg          = MAX30105_CONFIG_DEFAULT;
    max30105_config_t dev_cfg          = MAX30105_PARTICLE_CONFIG_DEFAULT;
    max30105_handle_t dev_hdl          = NULL;
    //
    // init device
    max30105_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "max30105 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## MAX30105 - START #########################");
        //
        // handle sensor
        esp_err_t                                  result = ESP_OK;
        max30105_adc_channels_count_data_t adc_count_data = { 0 };
        //
        result = max30105_get_optical_counts(dev_hdl, &adc_count_data);
        if (result != ESP_OK) {
            ESP_LOGE(APP_TAG, "max30105 get optical channel counts failed: %s", esp_err_to_name(result));
            //assert(result == ESP_OK);
        } else {

            ESP_LOGW(APP_TAG, "number of samples: %d", adc_count_data.sample_size);

            for(uint8_t i = 0; i < adc_count_data.sample_size; i++) {
                switch(dev_cfg.control_mode) {
                    case MAX30105_CM_RED_LED:
                        ESP_LOGW(APP_TAG, "sample[%d]: Red(%u)", i, adc_count_data.red_count[i]);
                        break;
                    case MAX30105_CM_RED_IR_LED:
                        ESP_LOGW(APP_TAG, "sample[%d]: Red(%u) | IR(%u)", i, adc_count_data.red_count[i], adc_count_data.ir_count[i]);
                        break;
                    case MAX30105_CM_GREEN_RED_IR_LED:
                        ESP_LOGW(APP_TAG, "sample[%d]: Red(%u) | IR(%u) | Green(%u)", i, adc_count_data.red_count[i], adc_count_data.ir_count[i], adc_count_data.green_count[i]);
                        break;
                    default:
                        ESP_LOGE(APP_TAG, "unknown control mode in mode configuration register");
                        break;
                }
            }
        }

        //
        ESP_LOGI(APP_TAG, "######################## MAX30105 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    max30105_delete( dev_hdl );
    vTaskDelete( NULL );
}