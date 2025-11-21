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
 * @file tcs3472_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <tcs3472_task.h>


void i2c0_tcs3472_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t       last_wake_time = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    tcs3472_config_t dev_cfg        = TCS3472_CONFIG_DEFAULT;
    tcs3472_handle_t dev_hdl        = NULL;
    //
    // init device
    tcs3472_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "tcs3472 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "############################ TCS3472 - START #############################");
        //
        // handle sensor
        esp_err_t result = ESP_OK;
        tcs3472_channels_data_t channels;
        tcs3472_colours_data_t colours;

        result = tcs3472_get_channels_count(dev_hdl, &channels);

        if (result != ESP_OK) {
            ESP_LOGE(APP_TAG, "tcs3472 get channels count failed (%s)", esp_err_to_name(result));
            //assert(result == ESP_OK);
        } else {
            tcs3472_normalize_colours(channels, &colours);

            uint16_t colour_temp = tcs3472_get_colour_temperature(channels);
            uint16_t ir = tcs3472_get_ir_light(channels);
            float illuminance;
            tcs3472_get_illuminance(dev_hdl, channels, &illuminance);

            ESP_LOGI(APP_TAG, "Colour Temp %u K | IR %u | Light %.2f Lux: R[%u|%d] | G[%u|%d] | B[%u|%d] | C[%u]", colour_temp, ir, illuminance,
                channels.red, (int)colours.red, channels.green, (int)colours.green, channels.blue, (int)colours.blue, channels.clear);
        }
        //
        ESP_LOGI(APP_TAG, "############################ TCS3472 - END ###############################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    tcs3472_delete( dev_hdl );
    vTaskDelete( NULL );
}