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
 * @file veml7700_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <veml7700_task.h>


static inline void print_registers( veml7700_handle_t handle ) {
    veml7700_configuration_register_t       cfg_reg;
    veml7700_power_saving_mode_register_t   psm_reg;

    /* attempt to read configuration register */
    veml7700_get_configuration_register(handle, &cfg_reg);

    /* attempt to read power saving mode register */
    veml7700_get_power_saving_mode_register(handle, &psm_reg);

    ESP_LOGI(APP_TAG, "Configuration Register:               0x%04x (%s)", cfg_reg.reg, uint16_to_binary(cfg_reg.reg));
    ESP_LOGI(APP_TAG, "Power Saving Mode Register:           0x%04x (%s)", psm_reg.reg, uint16_to_binary(psm_reg.reg));
}

void i2c0_veml7700_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    veml7700_config_t dev_cfg       = VEML7700_CONFIG_DEFAULT;
    veml7700_handle_t dev_hdl;
    //
    // init device
    veml7700_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "veml7700 handle init failed");
        assert(dev_hdl);
    }
    //
    //
    print_registers( dev_hdl );
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## VEML7700 - START #########################");
        //
        // handle sensor
        //float ambient_light;
        uint16_t als_counts, wcs_counts;
        //esp_err_t result = veml7700_get_ambient_light(dev_hdl, &ambient_light);
        esp_err_t result = veml7700_get_ambient_light_counts(dev_hdl, &als_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "veml7700 device read failed (%s)", esp_err_to_name(result));
        } else {
            //ESP_LOGI(APP_TAG, "ambient light:     %.2f lux", ambient_light);
            ESP_LOGI(APP_TAG, "ambient light:     %u counts", als_counts);
        }
        result = veml7700_get_white_channel_counts(dev_hdl, &wcs_counts);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "veml7700 device read failed (%s)", esp_err_to_name(result));
        } else {
            //ESP_LOGI(APP_TAG, "white channel:     %.2f lux", ambient_light);
            ESP_LOGI(APP_TAG, "white channel:     %u counts", wcs_counts);
        }
        //
        ESP_LOGI(APP_TAG, "######################## VEML7700 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    veml7700_delete( dev_hdl );
    vTaskDelete( NULL );
}