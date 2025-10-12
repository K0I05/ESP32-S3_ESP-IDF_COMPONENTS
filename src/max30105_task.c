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

#define AVG_SAMPLES_MAX 50

// Raw scattering counts (or ADC values) indicative of high particle concentration
#define ALARM_RAW_THRESHOLD 500

// Thresholds for Particle Classification (Ratio Analysis: IR / Green)
// These are float values representing the expected ratio ranges for different smoke types.
#define SMOLDERING_RATIO_MIN 1.5
#define SMOLDERING_RATIO_MAX 3.0

#define FLAMING_RATIO_MIN 0.5
#define FLAMING_RATIO_MAX 1.5

#define NUISANCE_RATIO_MIN 3.0

// --- SMOKE DETECTION & CLASSIFICATION ALGORITHM ---
const char* detect_smoke(uint32_t red_light, uint32_t ir_light, uint32_t green_light) {

    // Step 1: Check for high particle concentration (PM Alarm)
    // Use the IR reading as a primary indicator for a general alarm
    if (ir_light < ALARM_RAW_THRESHOLD) {
        return "✅ Clear Air: Particle concentration is within normal limits.";
    }

    ESP_LOGE(APP_TAG, "🔴 ALARM PRE-CONDITION MET: High Particulate Matter Detected!");

    // Step 2: Classify the particle type using the scattering ratio (IR/Green)
    float ir_green_ratio = 0.0;

    // Avoid division by zero
    if (green_light > 0) {
        ir_green_ratio = (float)ir_light / (float)green_light;
    } else {
        // Assign a high value to handle zero reading (indicating potentially very large, non-fire particles)
        ir_green_ratio = 999.0;
    }

    ESP_LOGE(APP_TAG, " Calculated IR/Green Ratio: %.2f", ir_green_ratio);

    // Classify based on pre-defined thresholds
    if (ir_green_ratio >= SMOLDERING_RATIO_MIN && ir_green_ratio <= SMOLDERING_RATIO_MAX) {
        return "🔥 FIRE ALARM: Smoldering Fire (Large Particle Smoke)";
    }

    if (ir_green_ratio < FLAMING_RATIO_MIN) {
        // While small particles scatter Green strongly, the IR signal must still be significant for fire
        return "🔥 FIRE ALARM: Flaming Fire (Small Particle Smoke)";
    }

    if (ir_green_ratio > NUISANCE_RATIO_MIN) {
        return "⚠️ Nuisance Detected: High ratio suggests Steam or Dust (Very Large Particles)";
    }

    // If it passed the raw threshold but didn't fit any ratio profile
    return "❓ WARNING: High PM Detected, Particle Type Unclassified.";
}

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
    uint32_t red_light_avg = 0, ir_light_avg = 0, green_light_avg = 0;
    uint8_t samples_count = 0;
    //
    //
    //
    /*
    //
    // presence detection example- start
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
    // presence detection example- end
    */
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## MAX30105 - START #########################");
        //
        // handle sensor
        esp_err_t                                  result = ESP_OK;


        /*
        // presence detection example - start
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
        // presence detection example- end

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
                if(samples_count + 1 <= AVG_SAMPLES_MAX) { 
                    red_light_avg   += red;
                    ir_light_avg    += ir;
                    green_light_avg += green;
                    samples_count++;
                    ESP_LOGW(APP_TAG, "samples (%d)", samples_count);
                }
                max30105_fifo_next_sample(dev_hdl); // Move to the next sample in the FIFO
                vTaskDelay(1);
            }
        }

        if(samples_count == AVG_SAMPLES_MAX) { 
            /* avoid dividing by zero and calculate average */
            if (samples_count > 0) {
                red_light_avg   /= samples_count;
                ir_light_avg    /= samples_count;
                green_light_avg /= samples_count;

                /* detect smoke */
                ESP_LOGW(APP_TAG, "max30105 detect smoke: %s", detect_smoke(red_light_avg, ir_light_avg, green_light_avg));

                /* reset averages and sampling count */
                red_light_avg   = 0;
                ir_light_avg    = 0;
                green_light_avg = 0;
                samples_count   = 0;
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