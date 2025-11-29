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
 * @file pressure_tendency.c
 *
 * Air pressure tendency libary
 * 
 * A air pressure tendency appears after three (3) hours of operation. The tendency 
 * codes and change are based on the 3-hr variance from the previous 3-hr history.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 * 
 */
#include <esp_check.h>
#include <esp_log.h>
#include <esp_types.h>

#include <math.h>

#include "include/pressure_tendency.h"

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define ESP_HRS_TO_SEC(HRS) ((uint32_t)((HRS) * 60 * 60))


/**
 * @brief Pressure tendency context descriptor structure definition.
 */
typedef struct pressure_tendency_context_s {
    uint16_t                    sampling_count;     /*!< pressure tendency sampling count, state machine variable */
    uint16_t                    sampling_size;      /*!< pressure tendency sampling size, state machine variable */
    float*                      sampling_buffer;    /*!< pressure tendency sampling buffer array, state machine variable */
    pressure_tendency_periods_t sampling_period;    /*!< pressure tendency sampling period (3-hrs or 6-hrs) */
    uint16_t                    sampling_interval;  /*!< pressure tendency sampling interval in seconds */
} pressure_tendency_context_t;

/*
* static constant declarations
*/
static const char *TAG = "pressure_tendency";

/**
 * @brief Evaluates pressure change and determines tendency code.
 * 
 * @param delta The pressure change value
 * @param code Pointer to store the resulting tendency code
 * @param change Pointer to store the pressure change value
 */
static inline void evaluate_pressure_delta(const float delta, pressure_tendency_codes_t *const code, float *const change) {
    /* evaluate delta aka 3-hr or 6-hr change in pressure */
    /* if the absolute variance is less than 1 hPa, air pressure is steady */
    /* if the delta is negative, and absolute variance is greater than 1 hPa, air pressure is falling */
    /* if the delta is positive, and absolute variance is greater than 1 hPa, air pressure is rising */
    if(fabs(delta) < 1) {
        // steady
        *code   = PRESSURE_TENDENCY_CODE_STEADY;
        *change = delta;
    } else {
        if(delta < 0 && fabs(delta) > 1) {
            /* falling */
            *code   = PRESSURE_TENDENCY_CODE_FALLING;
            *change = delta;
        } else if(delta > 0 && fabs(delta) > 1) {
            /* rising */
            *code   = PRESSURE_TENDENCY_CODE_RISING;
            *change = delta;
        } else {
            /* unknown condition */
            *code   = PRESSURE_TENDENCY_CODE_UNKNOWN;
            *change = NAN;
        }
    }
}

const char* pressure_tendency_code_to_string(const pressure_tendency_codes_t code) {
    switch(code) {
        case PRESSURE_TENDENCY_CODE_UNKNOWN:
            return "Unkown";
        case PRESSURE_TENDENCY_CODE_RISING:
            return "Rising";
        case PRESSURE_TENDENCY_CODE_STEADY:
            return "Steady";
        case PRESSURE_TENDENCY_CODE_FALLING:
            return "Falling";
        case PRESSURE_TENDENCY_CODE_TRAINING:
            return "Training";
        default:
            return "Unkown";
    }
}

esp_err_t pressure_tendency_init(const uint16_t sampling_interval, pressure_tendency_periods_t sampling_period, pressure_tendency_handle_t *pressure_tendency_handle) {
    esp_err_t ret           = ESP_OK;
    uint16_t  sampling_size = 0;

    /* validate arguments */
    ESP_GOTO_ON_FALSE( sampling_interval > 0, ESP_ERR_INVALID_ARG, err, TAG, "sampling interval must be greater than 0, pressure tendency handle initialization failed" );

    /* validate memory availability for pressure tendency handle */
    pressure_tendency_context_t* ctxt = (pressure_tendency_context_t*)calloc(1, sizeof(pressure_tendency_context_t));
    ESP_GOTO_ON_FALSE( ctxt, ESP_ERR_NO_MEM, err, TAG, "no memory for pressure tendency handle, pressure tendency handle initialization failed" );

    /* validate sampling period */
    if(sampling_period == PRESSURE_TENDENCY_3HR_PERIOD) {
        /* convert 3-hr period to seconds and calculate buffer size from sampling interval */
        sampling_size = ESP_HRS_TO_SEC(3) / sampling_interval;
    } else {
        /* convert 6-hr period to seconds and calculate buffer size from sampling interval */
        sampling_size = ESP_HRS_TO_SEC(6) / sampling_interval;
    }

    /* validate memory availability for samples array */
    ctxt->sampling_buffer = (float*)calloc(sampling_size, sizeof(float));
    ESP_GOTO_ON_FALSE( ctxt->sampling_buffer, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for pressure tendency handle samples, pressure tendency handle initialization failed" );

    /* copy configuration */
    ctxt->sampling_size     = sampling_size;
    ctxt->sampling_period   = sampling_period;
    ctxt->sampling_interval = sampling_interval;

    /* set output instance */
    *pressure_tendency_handle = (pressure_tendency_handle_t)ctxt;

    return ESP_OK;

    err_out_handle:
        free(ctxt->sampling_buffer);
        free(ctxt);
    err:
        return ret;
}

esp_err_t pressure_tendency_analysis(pressure_tendency_handle_t handle, 
                                    const float sample, 
                                    pressure_tendency_codes_t *const code,
                                    float *const change) {
    pressure_tendency_context_t* pressure_tendency_context = (pressure_tendency_context_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK(pressure_tendency_context);

    // have we filled the array?
    if (pressure_tendency_context->sampling_count < pressure_tendency_context->sampling_size) {
        // no! add this observation to the array
        pressure_tendency_context->sampling_buffer[pressure_tendency_context->sampling_count] = sample;

        // bump n
        pressure_tendency_context->sampling_count++;
    } else {
        // yes! the array is full so we have to make space
        for (uint16_t i = 1; i < pressure_tendency_context->sampling_size; i++) {
            pressure_tendency_context->sampling_buffer[i-1] = pressure_tendency_context->sampling_buffer[i];
        }

        // now we can fill in the last slot
        pressure_tendency_context->sampling_buffer[pressure_tendency_context->sampling_size-1] = sample;
    }

    /* validate sampling period */
    if(pressure_tendency_context->sampling_period == PRESSURE_TENDENCY_3HR_PERIOD) {
        // does the array have an hour of samples
        if (pressure_tendency_context->sampling_count < (pressure_tendency_context->sampling_size / 3)) {
            // no! we are still training
            *code   = PRESSURE_TENDENCY_CODE_TRAINING;
            *change = NAN;

            return ESP_OK;
        }
    } else {
        // does the array have an hour of samples
        if (pressure_tendency_context->sampling_count < (pressure_tendency_context->sampling_size / 6)) {
            // no! we are still training
            *code   = PRESSURE_TENDENCY_CODE_TRAINING;
            *change = NAN;

            return ESP_OK;
        }
    }

    /* subtract oldest pressure sample from latest pressure sample */
    const float delta = sample - pressure_tendency_context->sampling_buffer[0];

    /* evaluate delta aka 3-hr or 6-hr change in pressure */
    /* if the absolute variance is less than 1 hPa, air pressure is steady */
    /* if the delta is negative, and absolute variance is greater than 1 hPa, air pressure is falling */
    /* if the delta is positive, and absolute variance is greater than 1 hPa, air pressure is rising */
    evaluate_pressure_delta(delta, code, change);

    return ESP_OK;
}

esp_err_t pressure_tendency_reset(pressure_tendency_handle_t handle) {
    pressure_tendency_context_t* pressure_tendency_context = (pressure_tendency_context_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK(pressure_tendency_context);

    /* purge samples */
    for(uint16_t i = 0; i < pressure_tendency_context->sampling_size; i++) {
        pressure_tendency_context->sampling_buffer[i] = NAN;
    }

    /* reset samples counter */
    pressure_tendency_context->sampling_count = 0;

    return ESP_OK;
}

esp_err_t pressure_tendency_delete(pressure_tendency_handle_t handle) {
    pressure_tendency_context_t* pressure_tendency_context = (pressure_tendency_context_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK(pressure_tendency_context);

    if(pressure_tendency_context->sampling_buffer) 
        free(pressure_tendency_context->sampling_buffer);
    free(handle);
    
    return ESP_OK;
}

const char* pressure_tendency_get_fw_version(void) {
    return (const char*)PRESSURE_TENDENCY_FW_VERSION_STR;
}

int32_t pressure_tendency_get_fw_version_number(void) {
    return (int32_t)PRESSURE_TENDENCY_FW_VERSION_INT32;
}