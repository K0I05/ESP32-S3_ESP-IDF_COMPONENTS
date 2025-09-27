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
 * @file ds18b20_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <ds18b20_task.h>



void owb0_ds18b20_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t                   last_wake_time = xTaskGetTickCount ();
    //
    // initialize owb device configuration
    ds18b20_config_t             dev_cfg = DS18B20_CONFIG_DEFAULT;
    ds18b20_handle_t             dev_hdl;
    onewire_device_iter_handle_t dev_iter_hdl;
    onewire_device_t             dev;
    // owb ds18b20 device detection
    onewire_device_t             devs[5];
    uint8_t                      devs_size = sizeof(devs) / sizeof(devs[0]);
    uint8_t                      devs_count = 0;
    
    /* detect ds18b20 devices on 1-wire bus */
    esp_err_t ret = ds18b20_detect(owb0_bus_hdl, devs, devs_size, &devs_count);
    if(ret == ESP_OK) {
        ESP_LOGW(APP_TAG, "ds18b20 devices detected: %u", devs_count);
        for(uint8_t i = 0; i < devs_count; i++) {
            ESP_LOGI(APP_TAG, "ds18b20(%u), address: %016llX", i, devs[i].address);
        }
    } else {
        ESP_LOGE(APP_TAG, "ds18b20 device detect failed (%s)", esp_err_to_name(ret));
    }
    
    /* instantiate 1-wire device iterator handle */
    ESP_ERROR_CHECK( onewire_new_device_iter(owb0_bus_hdl, &dev_iter_hdl) );
    
    /* get 1-wire device - assumes there is only one ds18b20 device on the bus */
    if (onewire_device_iter_get_next(dev_iter_hdl, &dev) == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
        // check if the device is a ds18b20, if so, return the ds18b20 handle
        if (ds18b20_init(&dev, &dev_cfg, &dev_hdl) == ESP_OK) {
            ESP_LOGI(APP_TAG, "found a ds18b20, address: %016llX", dev.address);
        } else {
            ESP_LOGI(APP_TAG, "found an unknown device, address: %016llX", dev.address);
            assert(dev.address);
        }
    }

    /* free device iter handle */
    ESP_ERROR_CHECK( onewire_del_device_iter(dev_iter_hdl) );
    
    // validate device handle
    if(dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ds18b20 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for( ;; ) {
        ESP_LOGI(APP_TAG, "######################## DS18B20 - START #########################");
        
        /* handle sensor */
        float temperature;
        esp_err_t result = ds18b20_get_temperature(dev_hdl, &temperature);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "ds18b20 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "temperature:     %.2f°C", temperature);
        }
        //
        ESP_LOGI(APP_TAG, "######################## DS18B20 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, OWB0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    ds18b20_delete( dev_hdl );
    vTaskDelete( NULL );
}