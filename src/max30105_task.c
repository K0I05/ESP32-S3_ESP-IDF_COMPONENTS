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

#include <esp_timer.h>
#include <max30105_task.h>


void i2c0_max30105_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    max30105_config_t dev_cfg          = MAX30105_CONFIG_DEFAULT;
    //max30105_config_t dev_cfg          = MAX30105_PRESENCE_CONFIG_DEFAULT;
    //max30105_config_t dev_cfg          = MAX30105_PARTICLE_CONFIG_DEFAULT;
    max30105_handle_t dev_hdl          = NULL;
    //
    // init device
    max30105_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "max30105 handle init failed");
        assert(dev_hdl);
    }
    //
    /*
    //
    // presence detection example
    const uint32_t start_time = esp_timer_get_time() / 1000; // Get current time in milliseconds
    uint8_t num_of_samples = 0;
    uint32_t samples_taken = 0;
    uint32_t unblocked_value = 0;
    for(uint8_t x = 0; x < 32; x++) {
        uint32_t ir;
        max30105_fifo_get_ir_ch_head(dev_hdl, &ir);
        if(ir > 0) {
            num_of_samples++;
            ESP_LOGI(APP_TAG, "IR Value: %u", ir);
            unblocked_value += ir;
        }
    }
    if(num_of_samples > 0) {
        unblocked_value /= num_of_samples; // Prevent division by zero
    }
    ESP_LOGI(APP_TAG, "Unblocked Value: %u", unblocked_value);
    */
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## MAX30105 - START #########################");
        //
        // handle sensor
        esp_err_t                                  result = ESP_OK;


        /*
        // presence detection example
        samples_taken++;
        uint32_t ir_value;
        max30105_fifo_get_ir_ch_head(dev_hdl, &ir_value);
        ESP_LOGI(APP_TAG,"IR[%u],HZ[%.2f]", ir_value, (float)samples_taken / (((esp_timer_get_time() / 1000) - start_time) / 1000.0));
        int32_t current_delta = abs((int32_t)ir_value - (int32_t)unblocked_value); // Calculate the difference from the unblocked value
        ESP_LOGI(APP_TAG, "Current Delta: %i", current_delta);
        if (current_delta > 1000) // If the difference is greater than 1000, we assume the sensor is blocked
        {
            ESP_LOGI(APP_TAG, "Something is there");
        }

*/


        
        uint8_t num_of_samples = 0;
        if (max30105_fifo_sampling_check(dev_hdl, &num_of_samples) == ESP_OK)
        {
            while (max30105_fifo_samples_available(dev_hdl))
            { // Do we have new data?
                uint32_t red, ir, green;
                max30105_fifo_get_red_ch_head(dev_hdl, &red);
                max30105_fifo_get_ir_ch_head(dev_hdl, &ir);
                max30105_fifo_get_green_ch_head(dev_hdl, &green);
                ESP_LOGW(APP_TAG, "samples (%d): Red(%u) | IR(%u) | Green(%u)", (int)num_of_samples, red, ir, green);
                max30105_fifo_next_sample(dev_hdl); // Move to the next sample in the FIFO
                vTaskDelay(1);
            }
        }




        /*
        max30105_adc_channels_count_data_t adc_count_data = { 0 };
        result = max30105_get_optical_counts(dev_hdl, &adc_count_data);
        if (result != ESP_OK) {
            ESP_LOGE(APP_TAG, "max30105 get optical channel counts failed: %s", esp_err_to_name(result));
            //assert(result == ESP_OK);
        } else {

            ESP_LOGW(APP_TAG, "FIFO overflow has occurred %d times", adc_count_data.overflow_count);
            ESP_LOGW(APP_TAG, "number of samples: %d", adc_count_data.number_of_samples);

            for(uint8_t i = 0; i < adc_count_data.number_of_samples; i++) {
                switch(adc_count_data.control_mode) {
                    case MAX30105_CM_RED_LED:
                        ESP_LOGW(APP_TAG, "sample[%d]: Red(%u)", i, adc_count_data.red_count[i]);
                        break;
                    case MAX30105_CM_RED_IR_LED:
                        ESP_LOGW(APP_TAG, "sample[%d]: Red(%u) | IR(%u)", i, adc_count_data.red_count[i], adc_count_data.ir_count[i]);
                        break;
                    case MAX30105_CM_MULTI_LED:
                        ESP_LOGW(APP_TAG, "sample[%d]: Red(%u) | IR(%u) | Green(%u)", i, adc_count_data.red_count[i], adc_count_data.ir_count[i], adc_count_data.green_count[i]);
                        break;
                    default:
                        ESP_LOGE(APP_TAG, "unknown control mode in mode configuration register");
                        break;
                }
            }
        }
        
*/
        
        float temperature;
        result = max30105_get_temperature(dev_hdl, &temperature);
        if (result != ESP_OK) {
            ESP_LOGE(APP_TAG, "max30105 get die temperature failed: %s", esp_err_to_name(result));
            //assert(result == ESP_OK);
        } else {
            ESP_LOGW(APP_TAG, "die temperature: %.2f C", temperature);
        }
        
        //
        ESP_LOGI(APP_TAG, "######################## MAX30105 - END ###########################");
        //
        //
        // pause the task per defined wait period
        //vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE / 2 );
        vTaskDelaySecUntil( &last_wake_time, 1 );
    }
    //
    // free resources
    max30105_delete( dev_hdl );
    vTaskDelete( NULL );
}