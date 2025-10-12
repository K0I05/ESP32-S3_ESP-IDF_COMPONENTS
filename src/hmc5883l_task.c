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
 * @file hmc5883l_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <hmc5883l_task.h>



void i2c0_hmc5883l_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t         last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    hmc5883l_config_t dev_cfg       = HMC5883L_CONFIG_DEFAULT;
    hmc5883l_handle_t dev_hdl;
    //
    // init device
    hmc5883l_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "hmc5883l handle init failed");
        assert(dev_hdl);
    }
    //
    // calibrate device
    //i2c_hmc5883l_get_calibrated_offsets(dev_hdl, I2C_HMC5883L_CAL_BOTH);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## HMC5883L - START #########################");
        //
        // handle sensor
        hmc5883l_magnetic_axes_data_t magnetic_axes;
        esp_err_t result = hmc5883l_get_magnetic_axes(dev_hdl, &magnetic_axes);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "hmc5883l device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "Compass X-Axis:  %f mG", magnetic_axes.x_axis);
            ESP_LOGI(APP_TAG, "Compass Y-Axis:  %f mG", magnetic_axes.y_axis);
            ESP_LOGI(APP_TAG, "Compass Z-Axis:  %f mG", magnetic_axes.z_axis);
            ESP_LOGI(APP_TAG, "Compass Heading: %f °", magnetic_axes.heading);
        }
        //
        ESP_LOGI(APP_TAG, "######################## HMC5883L - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE / 2 );
    }
    //
    // free resources
    hmc5883l_delete( dev_hdl );
    vTaskDelete( NULL );
}