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
 * @file scalar_trend_task.c
 * @defgroup 
 * @{
 *
 *
 * Copyright (c) 2025 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include <math.h>
#include <time.h>
#include <scalar_trend_task.h>

/* Generate Gaussian noise (mean 0, stddev 1) using Box-Muller */
static double rand_normal_unit(void) {
    static int have_spare = 0;
    static double spare;
    if (have_spare) {
        have_spare = 0;
        return spare;
    }

    double u, v, s;
    do {
        u = (rand() + 1.0) / (RAND_MAX + 2.0); /* in (0,1) */
        v = (rand() + 1.0) / (RAND_MAX + 2.0);
        u = 2.0 * u - 1.0;
        v = 2.0 * v - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);

    s = sqrt(-2.0 * log(s) / s);
    spare = v * s;
    have_spare = 1;
    return u * s;
}

/*
 * simulate_temperatures
 *
 * out: pointer to double array of length num_samples (output)
 * num_samples: number of samples to generate
 * start_temp: starting temperature in degrees (e.g., 20.0)
 * amplitude: amplitude of sinusoidal variation in degrees (e.g., 1.5)
 * period_minutes: period of the sinusoid in minutes (e.g., 1440 for daily)
 * noise_std: standard deviation of Gaussian noise in degrees (e.g., 0.2)
 * drift_per_minute: linear drift added per minute in degrees (can be negative)
 *
 * The function fills out[i] for i=0..num_samples-1 corresponding to samples taken
 * each minute.
 */
void simulate_temperatures(double *out, int num_samples,
                           double start_temp, double amplitude, double period_minutes,
                           double noise_std, double drift_per_minute)
{
    if (out == NULL || num_samples <= 0) return;

    /* seed rand() once (caller may seed earlier if deterministic behavior desired) */
    /* we don't re-seed here to allow caller control if needed */

    for (int i = 0; i < num_samples; ++i) {
        const double minute = (double)i;
        double sinusoid = 0.0;
        if (period_minutes > 0.0) {
            sinusoid = amplitude * sin(2.0 * M_PI * minute / period_minutes);
        }
        const double drift = drift_per_minute * minute;
        const double noise = noise_std * rand_normal_unit();
        out[i] = start_temp + drift + sinusoid + noise;
    }
}

void utils_scalar_trend_task( void *pvParameters ) {
    TickType_t              last_wake_time = xTaskGetTickCount ();
    scalar_trend_handle_t scalar_trend_hdl = NULL;

    uint16_t samples_idx = 0;
    const uint16_t num_samples = 3 * 60; /* 3 hours * 60 minutes = 180 samples */
    double samples[num_samples];

    /* Example parameters:
     * - Start at 20.0 °C
     * - Small sinusoidal wiggle of 1.0 °C with a 3-hour period (so you see a hump)
     * - Measurement noise std dev 0.15 °C
     * - Slight upward drift 0.001 °C per minute (≈0.18 °C over 3 hours)
     */
    const double start_temp       = 20.0;
    const double amplitude        = 1.0;
    const double period_minutes   = 180.0;    /* 3-hour sinusoid (visible over the simulated window) */
    const double noise_std        = 0.15;
    const double drift_per_minute = 0.001;

    /* Seed RNG for noise */
    srand((unsigned int)time(NULL));

    /* Simulate */
    simulate_temperatures(samples, num_samples, start_temp, amplitude, period_minutes,
                          noise_std, drift_per_minute);

    // initialize scalar trend handle
    scalar_trend_init(60, &scalar_trend_hdl);

    // push 60 samples onto scalar trend handle
    for (uint16_t i = 0; i < 60; i++) {
        scalar_trend_codes_t trend_code = SCALAR_TREND_CODE_UNKNOWN;
        scalar_trend_analysis(scalar_trend_hdl, samples[i], &trend_code);
        samples_idx++;
    }
    
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SCALAR TREND - START #########################");
        
        // analyze next sample
        scalar_trend_codes_t trend_code = SCALAR_TREND_CODE_UNKNOWN;
        scalar_trend_analysis(scalar_trend_hdl, samples[samples_idx], &trend_code);
        samples_idx++;

        // log results
        ESP_LOGI(APP_TAG, "Sample %u: Temperature = %.2f °C, Trend = %s",
                 samples_idx,
                 samples[samples_idx - 1],
                 scalar_trend_code_to_string(trend_code) );

        // check if all samples processed
        if(samples_idx >= num_samples) {
            ESP_LOGW(APP_TAG, "All samples processed, exiting scalar trend task");
            break;
        }

        ESP_LOGI(APP_TAG, "######################## SCALAR TREND - END ###########################");
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, UTILS_TASK_SAMPLING_RATE );
    }
    
    // free resources
    scalar_trend_delete(scalar_trend_hdl);
    vTaskDelete( NULL );
}