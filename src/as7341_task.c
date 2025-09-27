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
 * @file as7341_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <as7341_task.h>

static inline void get_measurements(as7341_handle_t handle) {
    as7341_channels_spectral_data_t adc_data;
    esp_err_t result = as7341_get_spectral_measurements(handle, &adc_data);
    if(result != ESP_OK) {
        ESP_LOGE(APP_TAG, "spectral measurement failed (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGW(APP_TAG, "F1    %d", adc_data.f1);
        ESP_LOGW(APP_TAG, "F2    %d", adc_data.f2);
        ESP_LOGW(APP_TAG, "F3    %d", adc_data.f3);
        ESP_LOGW(APP_TAG, "F4    %d", adc_data.f4);
        ESP_LOGW(APP_TAG, "F5    %d", adc_data.f5);
        ESP_LOGW(APP_TAG, "F6    %d", adc_data.f6);
        ESP_LOGW(APP_TAG, "F7    %d", adc_data.f7);
        ESP_LOGW(APP_TAG, "F8    %d", adc_data.f8);
        ESP_LOGW(APP_TAG, "NIR   %d", adc_data.nir);
        ESP_LOGW(APP_TAG, "CLEAR %d", adc_data.clear);
    }

    /* get basic counts */
    as7341_channels_basic_counts_data_t basic_counts_data;
    result = as7341_get_basic_counts(handle, adc_data, &basic_counts_data);
    if(result != ESP_OK) {
        ESP_LOGE(APP_TAG, "basic counts conversion failed (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGW(APP_TAG, "F1    %f", basic_counts_data.f1);
        ESP_LOGW(APP_TAG, "F2    %f", basic_counts_data.f2);
        ESP_LOGW(APP_TAG, "F3    %f", basic_counts_data.f3);
        ESP_LOGW(APP_TAG, "F4    %f", basic_counts_data.f4);
        ESP_LOGW(APP_TAG, "F5    %f", basic_counts_data.f5);
        ESP_LOGW(APP_TAG, "F6    %f", basic_counts_data.f6);
        ESP_LOGW(APP_TAG, "F7    %f", basic_counts_data.f7);
        ESP_LOGW(APP_TAG, "F8    %f", basic_counts_data.f8);
        ESP_LOGW(APP_TAG, "NIR   %f", basic_counts_data.nir);
        ESP_LOGW(APP_TAG, "CLEAR %f", basic_counts_data.clear);
    }
}

static inline void get_flicker_detection_status(as7341_handle_t handle) {
    as7341_flicker_detection_states_t flicker_state;
    esp_err_t result = as7341_get_flicker_detection_status(handle, &flicker_state);
    if(result != ESP_OK) {
        ESP_LOGE(APP_TAG, "flicker detection failed (%s)", esp_err_to_name(result));
    } else {
        /* handle flicker state */
        switch(flicker_state) {
            case AS7341_FLICKER_DETECTION_INVALID:
                ESP_LOGW(APP_TAG, "Flicker Detection: Invalid");
                break;
            case AS7341_FLICKER_DETECTION_UNKNOWN:
                ESP_LOGW(APP_TAG, "Flicker Detection: Unknown");
                break;
            case AS7341_FLICKER_DETECTION_SATURATED:
                ESP_LOGW(APP_TAG, "Flicker Detection: Saturated");
                break;
            case AS7341_FLICKER_DETECTION_100HZ:
                ESP_LOGW(APP_TAG, "Flicker Detection: 100 Hz");
                break;
            case AS7341_FLICKER_DETECTION_120HZ:
                ESP_LOGW(APP_TAG, "Flicker Detection: 120 Hz");
                break;
        }
    }
    as7341_clear_flicker_detection_status_register(handle);
}

void i2c0_as7341_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t      last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    as7341_config_t dev_cfg          = AS7341_CONFIG_DEFAULT;
    as7341_handle_t dev_hdl;
    bool                flicker_completed = false;
    uint8_t             flicker_cycles = 0;
    //
    // init device
    as7341_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "as7341 handle init failed");
        assert(dev_hdl);
    }
    
    //as7341_disable_led(dev_hdl);
    
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AS7341 - START #########################");

        /* handle sensor */

        if(flicker_completed == true) {
            /* get spectral measurements */
            get_measurements(dev_hdl);
        } else {
            /* handle flicker detection */
            if(flicker_cycles < 5) {
                /* get flicker detection status */
                get_flicker_detection_status(dev_hdl);
                ++flicker_cycles;
            } else {
                flicker_completed = true;
            }
        }
        //
        ESP_LOGI(APP_TAG, "######################## AS7341 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    as7341_delete( dev_hdl );
    vTaskDelete( NULL );
}