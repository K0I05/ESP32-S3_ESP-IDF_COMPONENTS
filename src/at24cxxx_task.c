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
 * @file at24cxxx_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <string.h>
#include <at24cxxx_task.h>

void i2c0_at24cxxx_setup( at24cxxx_handle_t handle) {
    char *char_0 = "Hello World, this is a test!";
    char *char_1 = "This is just a random sentence!";

    at24cxxx_write_page(handle, 0, (uint8_t*)char_0, strlen(char_0) + 1);    // page 1 - 64-bytes

    at24cxxx_write_page(handle, 64+1, (uint8_t*)char_1, strlen(char_1) + 1);  // page 2 - 64-bytes
    
    at24cxxx_write_byte(handle, 128+1, 0x55);

    at24cxxx_write_byte(handle, 128+2, 0x45);
}


void i2c0_at24cxxx_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t        last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    at24cxxx_config_t dev_cfg          = AT24C256_CONFIG_DEFAULT;
    at24cxxx_handle_t dev_hdl;
    //
    // init device
    at24cxxx_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "at24cxxx handle init failed");
        assert(dev_hdl);
    }
    //
    i2c0_at24cxxx_setup(dev_hdl);

    uint8_t count = 0;
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AT24CXXX - START #########################");

        esp_err_t ret = ESP_OK;
        uint16_t size = 0;
        uint8_t b;
        uint8_t c;
        at24cxxx_memory_mapping_t memory_map;
        ret = at24cxxx_get_memory_map(dev_hdl, &memory_map);
        if(ret != ESP_OK) {   
            ESP_LOGE(APP_TAG, "at24cxxx device get memory map failed (%s)", esp_err_to_name(ret));
        } else {
            ESP_LOGI(APP_TAG, "at24cxxx device memory size: %d Kbits", memory_map.memory_size_kbits);
            ESP_LOGI(APP_TAG, "at24cxxx device page size: %d bytes", memory_map.page_size_bytes);
            ESP_LOGI(APP_TAG, "at24cxxx device number of pages: %d", memory_map.number_of_pages);
        }

        if(count == 5) {
            ret = at24cxxx_erase(dev_hdl);
            if(ret != ESP_OK) {   
                ESP_LOGE(APP_TAG, "at24cxxx device erase failed (%s)", esp_err_to_name(ret));
            }
        }

        if(count == 6) {
            count = 0;
            i2c0_at24cxxx_setup(dev_hdl);
        }

        char *data_0 = (char*)calloc(1, memory_map.page_size_bytes);
        ret = at24cxxx_read_page(dev_hdl, 0, (uint8_t*)data_0, &size);
        if(ret != ESP_OK) {   
            ESP_LOGE(APP_TAG, "at24cxxx device read page failed (%s)", esp_err_to_name(ret));
        }
        ESP_LOGI(APP_TAG, "%s", data_0);

        char *data_1 = (char*)calloc(1, memory_map.page_size_bytes);
        ret = at24cxxx_read_page(dev_hdl, 64+1, (uint8_t*)data_1, &size);
        if(ret != ESP_OK) {   
            ESP_LOGE(APP_TAG, "at24cxxx device read page failed (%s)", esp_err_to_name(ret));
        }
        ESP_LOGI(APP_TAG, "%s", data_1);

        ret = at24cxxx_read_random_byte(dev_hdl, 128+1, &b);
        if(ret != ESP_OK) {   
            ESP_LOGE(APP_TAG, "at24cxxx device read failed (%s)", esp_err_to_name(ret));
        }
        ret = at24cxxx_read_random_byte(dev_hdl, 128+2, &c);
        if(ret != ESP_OK) {   
            ESP_LOGE(APP_TAG, "at24cxxx device read failed (%s)", esp_err_to_name(ret));
        }

        ESP_LOGI(APP_TAG, "%d (0x%02x)", b, b);
        ESP_LOGI(APP_TAG, "%d (0x%02x)", c, c);

        count += 1;

        ESP_LOGI(APP_TAG, "######################## AT24CXXX - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, 60 );
    }
    //
    // free resources
    at24cxxx_delete( dev_hdl );
    vTaskDelete( NULL );
}