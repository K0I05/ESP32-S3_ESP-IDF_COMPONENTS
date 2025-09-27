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
 * @file as3935_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <as3935_task.h>


static void as3935_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    as3935_monitor_context_t *as3935_monitor_context = (as3935_monitor_context_t *)event_data;

    switch (event_id) {
        case AS3935_INT_NOISE:
            ESP_LOGW(APP_TAG, "as3935 device interrupt was noise related");
            break;
        case AS3935_INT_DISTURBER:
            ESP_LOGW(APP_TAG, "as3935 device interrupt was disturber related");
            break;
        case AS3935_INT_LIGHTNING:
            ESP_LOGW(APP_TAG, "as3935 device interrupt was lightning related");
            ESP_LOGW(APP_TAG, "Lightning distance: %d", as3935_monitor_context->base.lightning_distance);
            break;
        case AS3935_INT_NONE:
            ESP_LOGW(APP_TAG, "as3935 device interrupt was related to nothing");
            break;
        default:
            ESP_LOGW(APP_TAG, "as3935 device interrupt is unknown %li", event_id);
            break;
    }
}


void i2c0_as3935_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    as3935_monitor_handle_t monitor_hdl;
    as3935_config_t dev_cfg          = AS3935_CONFIG_DEFAULT;
    //as3935_handle_t dev_hdl;
    //
    // init device
    //as3935_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    as3935_monitor_init(i2c0_bus_hdl, &dev_cfg, &monitor_hdl);
    if (monitor_hdl == NULL) {
        ESP_LOGE(APP_TAG, "as3935_monitor_init handle init failed");
        assert(as3935_monitor_init);
    }
    //
    esp_err_t err = as3935_monitor_add_handler(monitor_hdl, as3935_event_handler, (void *)dev_cfg.irq_io_num);
    if (err != ESP_OK) {
        ESP_LOGE(APP_TAG, "as3935_monitor_add_handler failed");
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AS3935 - START #########################");
        //
        // handle sensor
        //
        ESP_LOGI(APP_TAG, "######################## AS3935 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    //as3935_delete( dev_hdl );
    vTaskDelete( NULL );
}