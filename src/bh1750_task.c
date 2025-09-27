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
 * @file bh1750_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <bh1750_task.h>



void i2c0_bh1750_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t      last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    bh1750_config_t dev_cfg         = BH1750_CONFIG_DEFAULT;
    bh1750_handle_t dev_hdl;
    //
    // init device
    bh1750_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bh1750 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BH1750 - START #########################");
        //
        // handle sensor
        float ambient_light;
        esp_err_t result = bh1750_get_ambient_light(dev_hdl, &ambient_light);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bh1750 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "ambient light:     %.2f lux", ambient_light);

        }
        //
        ESP_LOGI(APP_TAG, "######################## BH1750 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    bh1750_delete( dev_hdl );
    vTaskDelete( NULL );
}