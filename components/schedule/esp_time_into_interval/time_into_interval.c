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
 * @file time_into_interval.c
 *
 * ESP-IDF FreeRTOS task extension
 *
 * Ported from esp-open-rtos
 * 
 * https://www.epochconverter.com/
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/time_into_interval.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TIME_INTO_INTERVAL_NAME_MAX_LEN         (25)        //!< 25-characters for user-defined time-into-interval name

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief Time-into-interval context descriptor structure definition.
 */
typedef struct time_into_interval_context_s {
    const char*                      name;               /*!< time-into-interval, name, maximum of 25-characters */
    uint64_t                         epoch_timestamp;    /*!< time-into-interval, next event unix epoch timestamp (UTC) in milli-seconds */
    time_into_interval_types_t       interval_type;      /*!< time-into-interval, interval type setting */
    uint16_t                         interval_period;    /*!< time-into-interval, a non-zero interval period setting per interval type setting */
    uint16_t                         interval_offset;    /*!< time-into-interval, interval offset setting, per interval type setting, that must be less than the interval period */
    SemaphoreHandle_t                mutex_handle;       /*!< mutex handle of the time-into-interval handle */
} time_into_interval_context_t;

/*
* static constant declarations
*/
static const char *TAG = "time_into_interval";


/**
 * @brief Delays task until period has elapsed.
 * 
 * @param ms Delay period in milliseconds.
 */
static inline void time_into_interval_task_delay_until(const uint64_t ms) {
    const TickType_t delay_ticks = (ms / portTICK_PERIOD_MS);
    TickType_t previous_wake_time = xTaskGetTickCount();
    if(xTaskDelayUntil(&previous_wake_time, delay_ticks ) != pdTRUE) {
        ESP_LOGE(TAG, "Unable to delay task, time-into-interval task delay until failed");
        esp_restart(); 
    }
}

/**
 * @brief Initializes the next tm structure time-parts based on interval-type.
 * 
 * @param[in] interval_type Time into interval type (seconds, minutes, hours, etc.).
 * @param[in] interval_period_msec Time into interval period in milliseconds.
 * @param[in] now_tm Current time tm structure.
 * @param[out] next_tm Next time tm structure.
 */
static inline void time_into_interval_init_next_tm(const time_into_interval_types_t interval_type, const uint64_t interval_period_msec, const struct tm now_tm, struct tm *next_tm) {
    struct tm out_tm;

    // initialize next tm structure time-parts localtime based on interval-type
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            out_tm.tm_year = now_tm.tm_year;
            out_tm.tm_mon  = now_tm.tm_mon;
            out_tm.tm_mday = now_tm.tm_mday;
            out_tm.tm_hour = now_tm.tm_hour;
            out_tm.tm_min  = now_tm.tm_min;
            out_tm.tm_sec  = 0;
            break;
        case TIME_INTO_INTERVAL_MIN:
            out_tm.tm_year = now_tm.tm_year;
            out_tm.tm_mon  = now_tm.tm_mon;
            out_tm.tm_mday = now_tm.tm_mday;
            out_tm.tm_hour = now_tm.tm_hour;
            out_tm.tm_min  = 0;
            out_tm.tm_sec  = 0;
            break;
        case TIME_INTO_INTERVAL_HR:
            out_tm.tm_year = now_tm.tm_year;
            out_tm.tm_mon  = now_tm.tm_mon;
            out_tm.tm_mday = now_tm.tm_mday;
            out_tm.tm_hour = 0;
            out_tm.tm_min  = 0;
            out_tm.tm_sec  = 0;
            break;
    }

    /* handle interval period by tm structure time-part timespan exceedance */
    if(interval_period_msec > (60U * 1000U)) {
        /* over 60-seconds, set minute time-part to 0 */
        out_tm.tm_min  = 0;
        out_tm.tm_sec  = 0;
    } else if(interval_period_msec > (60U * 60U * 1000U)) {
        /* over 60-minutes, set hour time-part to 0 */
        out_tm.tm_hour = 0;
        out_tm.tm_min  = 0;
        out_tm.tm_sec  = 0;
    } else if(interval_period_msec > (24U * 60U * 60U * 1000U)) {
        /* over 24-hours, set day time-part to 0 */
        out_tm.tm_mday = 0;
        out_tm.tm_hour = 0;
        out_tm.tm_min  = 0;
        out_tm.tm_sec  = 0;
    }

    /* set output parameter */
    *next_tm = out_tm;
}

/**
 * @brief Sets the next epoch event timestamp in milliseconds from system clock based on 
 * the time interval type, period, and offset. 
 * 
 * The interval should be divisible by 60 i.e. no remainder if the interval type and period
 * is every 10-seconds, the event will trigger on-time with the system clock i.e. 09:00:00, 
 * 09:00:10, 09:00:20, etc.
 * 
 * The interval offset is used to offset the start of the interval period.  If the interval type
 * and period is every 5-minutes with a 1-minute offset, the event will trigger on-time with the
 * system clock i.e. 09:01:00, 09:06:00, 09:11:00, etc.
 * 
 * @param[in] interval_type Time into interval type (seconds, minutes, hours, etc.).
 * @param[in] interval_period Time into interval period for interval type.
 * @param[in] interval_offset Time into interval offset for interval type.
 * @param[out] epoch_timestamp Unix epoch timestamp (UTC) of next event in milliseconds.
 */
static inline esp_err_t time_into_interval_set_epoch_timestamp_event(const time_into_interval_types_t interval_type, const uint16_t interval_period, const uint16_t interval_offset, uint64_t *epoch_timestamp) {
    struct timeval  now_tv;
    struct tm       now_tm;
    struct tm       next_tm;

    /* validate interval period argument */
    ESP_RETURN_ON_FALSE( (interval_period > 0), ESP_ERR_INVALID_ARG, TAG, "interval period cannot be 0, time-into-interval set epoch time event failed" );

    /* normalize interval period and offset to milli-seconds */
    uint64_t interval_period_msec = time_into_interval_normalize_interval_to_msec(interval_type, interval_period);
    uint64_t interval_offset_msec = time_into_interval_normalize_interval_to_msec(interval_type, interval_offset);

    /* validate interval period argument on total days */
    ESP_RETURN_ON_FALSE( (interval_period_msec <= (28U * 24U * 60U * 60U * 1000U)), ESP_ERR_INVALID_ARG, TAG, "interval period cannot be greater than 28-days, time-into-interval set epoch time event failed" );

    /* validate period and offset intervals */
    ESP_RETURN_ON_FALSE( ((interval_period_msec - interval_offset_msec) > 0), ESP_ERR_INVALID_ARG, TAG, "interval period must be larger than the interval offset, time-into-interval set epoch time event failed" );

    // get system unix epoch time (gmt)
    gettimeofday(&now_tv, NULL);

    // extract system unix time (seconds and milli-seconds)
    time_t now_unix_time = now_tv.tv_sec;

    // set epoch timestamp in milliseconds
    uint64_t now_unix_time_msec = (uint64_t)now_tv.tv_sec * 1000U + ((uint64_t)now_tv.tv_usec / 1000U);

    // convert now tm to time-parts localtime from unix time
    localtime_r(&now_unix_time, &now_tm);

    // initialize next tm structure time-parts localtime based on interval-type
    time_into_interval_init_next_tm(interval_type, interval_period_msec, now_tm, &next_tm);

    // initialize next unix time by adding the task event interval period and offset in milliseconds
    uint64_t next_unix_time_msec = (uint64_t)(mktime(&next_tm) * 1000U) + interval_period_msec + interval_offset_msec;

    // compute the delta between next unix and now times
    int64_t delta_time_msec = next_unix_time_msec - now_unix_time_msec;

    // ensure next task event is ahead in time
    if(delta_time_msec <= 0) {
        // next task event is not ahead in time
        do {
            // keep adding task event intervals until next task event is ahead in time
            next_unix_time_msec = next_unix_time_msec + interval_period_msec;
            
            // compute the delta between next unix and now times
            delta_time_msec = next_unix_time_msec - now_unix_time_msec;
        } while(delta_time_msec < 0);
    }

    // set next task event epoch time
    *epoch_timestamp = next_unix_time_msec - 1;

    return ESP_OK;
}

esp_err_t time_into_interval_get_interval(time_into_interval_handle_t handle, time_into_interval_types_t *const interval_type, uint16_t *const interval_period) {
    time_into_interval_context_t* ctxt = (time_into_interval_context_t*)handle;

    // validate arguments
    ESP_ARG_CHECK( ctxt );

    *interval_type   = ctxt->interval_type;
    *interval_period = ctxt->interval_period;

    return ESP_OK;
}

uint64_t time_into_interval_normalize_interval_to_sec(const time_into_interval_types_t interval_type, const uint16_t interval) {
    uint64_t interval_sec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            interval_sec = interval;
            break;
        case TIME_INTO_INTERVAL_MIN:
            interval_sec = (interval * 60U);  // 1-minute has 60-seconds
            break;
        case TIME_INTO_INTERVAL_HR:
            interval_sec = (interval * (60U * 60U)); // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_sec;
}

uint64_t time_into_interval_normalize_interval_to_msec(const time_into_interval_types_t interval_type, const uint16_t interval) {
    uint64_t interval_msec = 0;

    // normalize interval to sec
    switch(interval_type) {
        case TIME_INTO_INTERVAL_SEC:
            interval_msec = interval * 1000U;
            break;
        case TIME_INTO_INTERVAL_MIN:
            interval_msec = (interval * 60U) * 1000U; // 1-minute has 60-seconds
            break;
        case TIME_INTO_INTERVAL_HR:
            interval_msec = ((interval * 60U) * 60U) * 1000U; // 1-hour has 60-minutes, 1-minute has 60-seconds
            break;
    }

    return interval_msec;
}

uint64_t time_into_interval_get_epoch_timestamp(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;
 
    // extract unix epoch utc timestamp and convert to seconds
    return (uint64_t)tv_utc_timestamp.tv_sec;
}

uint64_t time_into_interval_get_epoch_timestamp_msec(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;

    // extract unix epoch utc timestamp and convert to milli-seconds
    return (uint64_t)tv_utc_timestamp.tv_sec * 1000U + (uint64_t)tv_utc_timestamp.tv_usec / 1000U;
}

uint64_t time_into_interval_get_epoch_timestamp_usec(void) {
    // get current time as 'struct timeval'.
    // see https://linux.die.net/man/2/gettimeofday
    struct timeval tv_utc_timestamp;

    // get unix utc timestamp and validate results
    if(gettimeofday(&tv_utc_timestamp, NULL) == -1) return 0;

    // extract unix epoch utc timestamp and convert to micro-seconds
    return (uint64_t)tv_utc_timestamp.tv_sec * 1000000U + (uint64_t)tv_utc_timestamp.tv_usec;
}

esp_err_t time_into_interval_init(const time_into_interval_config_t *time_into_interval_config, 
                                 time_into_interval_handle_t *time_into_interval_handle) {
    esp_err_t                   ret = ESP_OK;
    
    /* validate task-schedule arguments */
    ESP_GOTO_ON_FALSE( (strnlen(time_into_interval_config->name, TIME_INTO_INTERVAL_NAME_MAX_LEN + 1) < TIME_INTO_INTERVAL_NAME_MAX_LEN), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval name cannot exceed 20-characters, time-into-interval handle initialization failed" );
    ESP_GOTO_ON_FALSE( (time_into_interval_config->interval_period > 0), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval interval period cannot be 0, time-into-interval handle initialization failed" );

    /* validate period and offset intervals */
    int64_t interval_delta = time_into_interval_normalize_interval_to_msec(time_into_interval_config->interval_type, time_into_interval_config->interval_period) - 
                             time_into_interval_normalize_interval_to_msec(time_into_interval_config->interval_type, time_into_interval_config->interval_offset); 
    ESP_GOTO_ON_FALSE( (interval_delta > 0), ESP_ERR_INVALID_ARG, err, TAG, "time-into-interval interval period must be larger than the interval offset, time-into-interval handle initialization failed" );
    
    /* validate memory availability for time into interval handle */
    time_into_interval_context_t* ctxt = (time_into_interval_context_t*)calloc(1, sizeof(time_into_interval_context_t));
    ESP_GOTO_ON_FALSE( ctxt, ESP_ERR_NO_MEM, err, TAG, "no memory for time-into-interval handle, time-into-interval handle initialization failed" );

    /* initialize task schedule state object parameters */
    ctxt->name            = time_into_interval_config->name;
    ctxt->epoch_timestamp = 0;
    ctxt->interval_type   = time_into_interval_config->interval_type;
    ctxt->interval_period = time_into_interval_config->interval_period;
    ctxt->interval_offset = time_into_interval_config->interval_offset;
    ctxt->mutex_handle    = xSemaphoreCreateMutex();

    /* set epoch timestamp of the next scheduled time-into-interval event */
    ESP_GOTO_ON_ERROR( time_into_interval_set_epoch_timestamp_event(ctxt->interval_type, 
                                                                ctxt->interval_period, 
                                                                ctxt->interval_offset, 
                                                                &ctxt->epoch_timestamp), 
                                                                err_out_handle, TAG, "unable to set epoch timestamp, time-into-interval handle initialization failed" );

    /* set output handle */
    *time_into_interval_handle = (time_into_interval_handle_t)ctxt;

    return ESP_OK;

    err_out_handle:
        free(ctxt);
    err:
        return ret;
}

bool time_into_interval(time_into_interval_handle_t handle) {
    time_into_interval_context_t* ctxt = (time_into_interval_context_t*)handle;
    bool state = false;

    /* validate arguments */
    if(!(ctxt)) {
        return state;
    }

    /* lock the mutex */
    xSemaphoreTake(ctxt->mutex_handle, portMAX_DELAY);

    // get system unix epoch timestamp (UTC)
    uint64_t now_unix_msec = time_into_interval_get_epoch_timestamp_msec();

    // compute time delta until next time into interval condition
    int64_t delta_msec = ctxt->epoch_timestamp - now_unix_msec;

    //ESP_LOGW(TAG, "time_into_interval(%s):delta_msec: %lli", ctxt->name, delta_msec);

    // validate time delta, when delta is <= 0, time has elapsed
    if(delta_msec <= 0) {
        
        // set time-into-interval state to true - intervale has lapsed
        state = true;

        /* set next event timestamp (UTC) */
        time_into_interval_set_epoch_timestamp_event(ctxt->interval_type, 
                                                    ctxt->interval_period, 
                                                    ctxt->interval_offset, 
                                                    &ctxt->epoch_timestamp);
    }

    /* unlock the mutex */
    xSemaphoreGive(ctxt->mutex_handle);
    
    return state;
}

esp_err_t time_into_interval_delay(time_into_interval_handle_t handle) {
    time_into_interval_context_t* ctxt = (time_into_interval_context_t*)handle;

    // validate arguments
    ESP_ARG_CHECK( ctxt );

    /* lock the mutex */
    xSemaphoreTake(ctxt->mutex_handle, portMAX_DELAY);

    // get system unix epoch timestamp (UTC)
    uint64_t now_unix_msec = time_into_interval_get_epoch_timestamp_msec();

    // compute time delta until next scan event
    int64_t delta_msec = ctxt->epoch_timestamp - now_unix_msec;

    // validate time is into the future, otherwise, reset next epoch time
    if(delta_msec < 0) {

        // set epoch timestamp of the next scheduled task
        time_into_interval_set_epoch_timestamp_event(ctxt->interval_type, 
                                                    ctxt->interval_period, 
                                                    ctxt->interval_offset, 
                                                    &ctxt->epoch_timestamp);

        // compute time delta for next event
        delta_msec = ctxt->epoch_timestamp - now_unix_msec;
    }

    //ESP_LOGW(TAG, "time_into_interval_delay(%s):delta_msec: %lli", ctxt->name, delta_msec);

    /* unlock the mutex */
    xSemaphoreGive(ctxt->mutex_handle);

    /* wait for the next cycle */
    time_into_interval_task_delay_until(delta_msec);

    /* lock the mutex */
    xSemaphoreTake(ctxt->mutex_handle, portMAX_DELAY);

    // set epoch timestamp of the next scheduled task
    time_into_interval_set_epoch_timestamp_event(ctxt->interval_type, 
                                                ctxt->interval_period, 
                                                ctxt->interval_offset, 
                                                &ctxt->epoch_timestamp);
                                        
    /* unlock the mutex */
    xSemaphoreGive(ctxt->mutex_handle);

    return ESP_OK;
}

esp_err_t time_into_interval_get_last_event(time_into_interval_handle_t handle, uint64_t *epoch_timestamp) {
    time_into_interval_context_t* ctxt = (time_into_interval_context_t*)handle;

    // validate arguments
    ESP_ARG_CHECK( ctxt );

    /* lock the mutex */
    xSemaphoreTake(ctxt->mutex_handle, portMAX_DELAY);

    /* convert interval into msec */
    uint64_t interval_msec = time_into_interval_normalize_interval_to_msec(ctxt->interval_type, ctxt->interval_period);

    /* set last event epoch timestamp */
    *epoch_timestamp = ctxt->epoch_timestamp - interval_msec;

    /* unlock the mutex */
    xSemaphoreGive(ctxt->mutex_handle);

    return ESP_OK;
}

esp_err_t time_into_interval_get_next_event(time_into_interval_handle_t handle, uint64_t *epoch_timestamp) {
    time_into_interval_context_t* ctxt = (time_into_interval_context_t*)handle;

    // validate arguments
    ESP_ARG_CHECK( ctxt );

    /* lock the mutex */
    xSemaphoreTake(ctxt->mutex_handle, portMAX_DELAY);

    /* set next event epoch timestamp */
    *epoch_timestamp = ctxt->epoch_timestamp;

    /* unlock the mutex */
    xSemaphoreGive(ctxt->mutex_handle);

    return ESP_OK;
}


esp_err_t time_into_interval_delete(time_into_interval_handle_t handle) {
    /* free resource */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* time_into_interval_get_fw_version(void) {
    return (const char*)TIME_INTO_INTERVAL_FW_VERSION_STR;
}

int32_t time_into_interval_get_fw_version_number(void) {
    return (int32_t)TIME_INTO_INTERVAL_FW_VERSION_INT32;
}