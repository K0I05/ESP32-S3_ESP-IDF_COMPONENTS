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
 * @file time_into_interval_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <math.h>
#include <time_into_interval_task.h>
#include <sys/time.h>
#include <esp_timer.h>

#define TIME_FORMAT_BUFFER_SIZE            (64)

void print_system_time(const char* info) {
    time_t now; struct tm timeinfo; static char strftime_buf[TIME_FORMAT_BUFFER_SIZE];
    struct timeval ts; gettimeofday(&ts, NULL); 
    time(&now); localtime_r(&now, &timeinfo); strftime(strftime_buf, sizeof(strftime_buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    ESP_LOGI(APP_TAG, "%s.%lu: %s", strftime_buf, ts.tv_usec, info);
}

void sch_time_into_interval_task( void *pvParameters ) {
    /* time-into-interval with a 1-min period and 10-sec offset */
    time_into_interval_handle_t tii_1min10sec_hdl;
    const time_into_interval_config_t tii_1min10sec_cfg = {
        .name               = "tti_1min10sec",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 60,
        .interval_offset    = 10
    };

    /* time-into-interval with a 1-min period */
    time_into_interval_handle_t tii_10sec_hdl;
    const time_into_interval_config_t tii_10sec_cfg = {
        .name               = "tti_60sec",
        .interval_type      = TIME_INTO_INTERVAL_SEC,
        .interval_period    = 60,
        .interval_offset    = 0
    };

    /* attempt to initialize a time-into-interval tii_1min10sec_hdl handle */
    time_into_interval_init(&tii_1min10sec_cfg, &tii_1min10sec_hdl);
    if (tii_1min10sec_hdl == NULL) ESP_LOGE(APP_TAG, "tii_1min10sec_hdl, init time-into-interval handle failed");

    /* attempt to initialize a time-into-interval tii_10sec_hdl handle */
    time_into_interval_init(&tii_10sec_cfg, &tii_10sec_hdl);
    if (tii_10sec_hdl == NULL) ESP_LOGE(APP_TAG, "tii_10sec_hdl, init time-into-interval handle failed");

    ESP_LOGI(APP_TAG, "######################## TIME-INTO-INTERVAL - START #########################");

    // task loop entry point
    for ( ;; ) {
        //ESP_LOGI(APP_TAG, "######################## TIME-INTO-INTERVAL - START #########################");
        
        /* print time when period has elapsed */
        //if(time_into_interval(tii_1min10sec_hdl) == true) {
        //    print_system_time("tii_1min10sec_hdl conditional execution");
        //}
        ESP_LOGI(APP_TAG, "Firmware Version: %s", time_into_interval_get_fw_version());

        /* delay task */
        time_into_interval_delay(tii_10sec_hdl);

        print_system_time("tii_60sec_hdl delayed execution");

        //ESP_LOGI(APP_TAG, "######################## TIME-INTO-INTERVAL - END ###########################");
    }
    
    // free resources
    time_into_interval_delete(tii_1min10sec_hdl);
    time_into_interval_delete(tii_10sec_hdl);
    vTaskDelete( NULL );
}