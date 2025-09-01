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
 * @file bme680_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <bme680_task.h>

static inline void print_registers(bme680_handle_t handle) {
    /* configuration registers */
    bme680_control_measurement_register_t ctrl_meas_reg;
    bme680_control_humidity_register_t    ctrl_humi_reg;
    bme680_config_register_t              config_reg;
    bme680_control_gas0_register_t        ctrl_gas0_reg;
    bme680_control_gas1_register_t        ctrl_gas1_reg;

    /* attempt to read control humidity register */
    bme680_get_control_humidity_register(handle, &ctrl_humi_reg);

    /* attempt to read control measurement register */
    bme680_get_control_measurement_register(handle, &ctrl_meas_reg);

    /* attempt to read configuration register */
    bme680_get_configuration_register(handle, &config_reg);

    /* attempt to read control gas 0 register */
    bme680_get_control_gas0_register(handle, &ctrl_gas0_reg);

    /* attempt to read control gas 1 register */
    bme680_get_control_gas1_register(handle, &ctrl_gas1_reg);

    //ESP_LOGI(APP_TAG, "Variant Id          (0x%02x): %s", handle->variant_id,uint8_to_binary(handle->variant_id));
    ESP_LOGI(APP_TAG, "Configuration       (0x%02x): %s", config_reg.reg,    uint8_to_binary(config_reg.reg));
    ESP_LOGI(APP_TAG, "Control Measurement (0x%02x): %s", ctrl_meas_reg.reg, uint8_to_binary(ctrl_meas_reg.reg));
    ESP_LOGI(APP_TAG, "Control Humidity    (0x%02x): %s", ctrl_humi_reg.reg, uint8_to_binary(ctrl_humi_reg.reg));
    ESP_LOGI(APP_TAG, "Control Gas 0       (0x%02x): %s", ctrl_gas0_reg.reg, uint8_to_binary(ctrl_gas0_reg.reg));
    ESP_LOGI(APP_TAG, "Control Gas 1       (0x%02x): %s", ctrl_gas1_reg.reg, uint8_to_binary(ctrl_gas1_reg.reg));
}

void i2c0_bme680_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    
    // initialize i2c device configuration
    bme680_config_t dev_cfg         = I2C_BME680_FORCED_CONFIG_DEFAULT;
    //bme680_config_t dev_cfg         = I2C_BME680_SEQUENTIAL_CONFIG_DEFAULT;
    bme680_handle_t dev_hdl;
    
    // init device
    bme680_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bme680 handle init failed");
        assert(dev_hdl);
    }
    
    print_registers(dev_hdl);

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BME680 - START #########################");
        
        // handle sensor

        esp_err_t result;
        
        bme680_data_t data;
        
        result = bme680_get_data(dev_hdl, &data);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", data.air_temperature);
            ESP_LOGI(APP_TAG, "dewpoint temperature:%.2f °C", data.dewpoint_temperature);
            ESP_LOGI(APP_TAG, "relative humidity:   %.2f %%", data.relative_humidity);
            ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", data.barometric_pressure/100);
            ESP_LOGI(APP_TAG, "gas resistance(%u):   %.2f kΩ", data.gas_index, data.gas_resistance/1000);
            ESP_LOGI(APP_TAG, "iaq score:           %u (%s)", data.iaq_score, bme680_air_quality_to_string(data.iaq_score));
            ESP_LOGI(APP_TAG, "heater is stable:    %s", data.heater_stable ? "yes" : "no");
            ESP_LOGI(APP_TAG, "gas range:           %u", data.gas_range);
            ESP_LOGI(APP_TAG, "gas valid:           %s", data.gas_valid ? "yes" : "no");
        }
        

        //bme680_data_t data[BME680_HEATER_PROFILE_SIZE];
        //result = bme680_get_data2(dev_hdl, data);
        //if(result != ESP_OK) {
        //    ESP_LOGE(APP_TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
        //}

        //ESP_LOGI(APP_TAG, "Index Air(°C) Dew-Point(°C) Humidity(%%) Pressure(hPa) Gas-Resistance(kΩ) Gas-Range Gas-Valid Gas-Index Heater-Stable IAQ-Score");

        /*
        for(uint8_t i = 0; i < dev_hdl->dev_config.heater_profile_size; i++) {
            bme680_data_t data;
            result = bme680_get_data_by_heater_profile(dev_hdl, i, &data);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "bme680 device read failed (%s)", esp_err_to_name(result));
            }
            ESP_LOGI(APP_TAG, "%u    %.2f    %.2f          %.2f         %.2f          %.2f               %u        %s        %u        %s            %u (%s)",
                i,
                data.air_temperature,
                data.dewpoint_temperature,
                data.relative_humidity,
                data.barometric_pressure/100,
                data.gas_resistance/1000,
                data.gas_range,
                data.gas_valid ? "yes" : "no",
                data.gas_index,
                data.heater_stable ? "yes" : "no",
                data.iaq_score, bme680_air_quality_to_string(data.iaq_score));

            ESP_LOGI(APP_TAG, "(%u) air temperature:     %.2f °C", i, data.air_temperature);
            ESP_LOGI(APP_TAG, "(%u) dewpoint temperature:%.2f °C", i, data.dewpoint_temperature);
            ESP_LOGI(APP_TAG, "(%u) relative humidity:   %.2f %%", i, data.relative_humidity);
            ESP_LOGI(APP_TAG, "(%u) barometric pressure: %.2f hPa", i, data.barometric_pressure/100);
            ESP_LOGI(APP_TAG, "(%u) gas resistance(%u):   %.2f kΩ", i, data.gas_index, data.gas_resistance/1000);
            ESP_LOGI(APP_TAG, "(%u) gas range(%u):        %u", i, data.gas_index, data.gas_range);
            ESP_LOGI(APP_TAG, "(%u) gas valid(%u):        %s", i, data.gas_index, data.gas_valid ? "yes" : "no");
            ESP_LOGI(APP_TAG, "(%u) heater is stable:    %s", i, data.heater_stable ? "yes" : "no");
            ESP_LOGI(APP_TAG, "(%u) iaq score:           %u (%s)", i, data.iaq_score, bme680_air_quality_to_string(data.iaq_score));
            
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        */
        
        ESP_LOGI(APP_TAG, "######################## BME680 - END ###########################");

        // pause the task per defined wait period
        //vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
        vTaskDelaySecUntil( &last_wake_time, 60 );
    }
    //
    // free resources
    bme680_delete( dev_hdl );
    vTaskDelete( NULL );
}
