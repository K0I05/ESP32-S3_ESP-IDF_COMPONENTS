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
 * @file sgp4x_task.c
 * @defgroup 
 * @{
 *
 * https://github.com/Sensirion/raspberry-pi-i2c-sgp41/blob/master/README.md
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <sgp4x_task.h>



void i2c0_sgp4x_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t     last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    sgp4x_config_t dev_cfg          = I2C_SGP41_CONFIG_DEFAULT;
    sgp4x_handle_t dev_hdl;
    bool           dev_self_tested  = false;
    bool           dev_conditioned  = false;
    
    /* initialize gas index parameters */
    GasIndexAlgorithmParams voc_params;
    GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
    GasIndexAlgorithmParams nox_params;
    GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
    //
    // init device
    sgp4x_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "sgp4x handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SGP4X - START #########################");
        
        /* handle sensor */
        if(dev_self_tested == false) {
            sgp4x_self_test_result_t self_test_result;
            esp_err_t result = sgp4x_execute_self_test(dev_hdl, &self_test_result);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "sgp4x device self-test failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGI(APP_TAG, "VOC Pixel:   %d", self_test_result.pixels.voc_pixel_failed);
                ESP_LOGI(APP_TAG, "NOX Pixel:   %d", self_test_result.pixels.nox_pixel_failed);
            }
            dev_self_tested = true;
        }
        /* conditioning validation */
        if(dev_conditioned == false) {
            for(int i = 0; i < 10; i++) {
                uint16_t sraw_voc; 
                esp_err_t result = sgp4x_execute_conditioning(dev_hdl, &sraw_voc);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
                } else {
                    ESP_LOGI(APP_TAG, "SRAW VOC: %u", sraw_voc);
                }
                vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second * 10 iterations = 10-seconds
            }
            dev_conditioned = true;
        } else {
            /* measure signals and process gas algorithm */
            uint16_t sraw_voc; uint16_t sraw_nox;
            int32_t voc_index; int32_t nox_index;
            esp_err_t result = sgp4x_measure_signals(dev_hdl, &sraw_voc, &sraw_nox);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
            } else {
                GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index);
                GasIndexAlgorithm_process(&nox_params, sraw_nox, &nox_index);

                ESP_LOGI(APP_TAG, "SRAW VOC: %u | VOC Index: %li | SRAW NOX: %u | NOX Index: %li", sraw_voc, voc_index, sraw_nox, nox_index);
            }
        }
        
        //
        ESP_LOGI(APP_TAG, "######################## SGP4X - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    sgp4x_delete( dev_hdl );
    vTaskDelete( NULL );
}