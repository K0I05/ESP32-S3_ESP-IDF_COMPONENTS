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
 * @file wx_utils.c
 *
 * ESP-IDF weather utilities (utils)
 *
 * References
 * https://docs.vaisala.com/r/M212417EN-H/en-US/GUID-EBE2B115-3C98-4C8C-9F2D-A2FF1EFECFCC
 * https://earthscience.stackexchange.com/questions/16366/weather-forecast-based-on-pressure-temperature-and-humidity-only-for-implement
 * https://web.archive.org/web/20110610213848/http://www.meteormetrics.com/zambretti.htm
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */

#include "include/wx_utils.h"
#include <string.h>
#include <stdio.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * weather utilities definitions
*/

#define WX_TEMPERATURE_MAX       (float)(125.0)  //!< maximum temperature range
#define WX_TEMPERATURE_MIN       (float)(-40.0)  //!< minimum temperature range
#define WX_HUMIDITY_MAX          (float)(100.0)  //!< maximum humidity range
#define WX_HUMIDITY_MIN          (float)(0.0)    //!< minimum humidity range


/*
 * macro definitions
*/

#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* weather utilities static global declarations
*/

/**
 * @brief Weather utilities temperature maximum and minimum ranges in degrees Celsius (default maximum 125 and minimum -40).
 */
static inline wx_scalar_range_t wx_temperature_range = { .maximum = WX_TEMPERATURE_MAX, .minimum = WX_TEMPERATURE_MIN };

/**
 * @brief Weather utilities humidity maximum and minimum ranges in percent (default maximum 100 and minimum 0).
 */
static inline wx_scalar_range_t wx_humidity_range = { .maximum = WX_HUMIDITY_MAX, .minimum = WX_HUMIDITY_MIN };


/*
* static constant declarations
*/

static const char *TAG = "wx_utils";


/*
* functions and subroutines
*/

esp_err_t wx_set_temperature_range(const float maximum, const float minimum) {
    /* validate max and min arguments */
    if(minimum > maximum || maximum < minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "minimum temperature must be smaller than the maximum temperature, set temperature range failed");
    }

    /* set max and min range */
    wx_temperature_range = (wx_scalar_range_t){ .maximum = maximum, .minimum = minimum };

    return ESP_OK;
}

esp_err_t wx_set_humidity_range(const float maximum, const float minimum) {
    /* validate max and min arguments */
    if(minimum > maximum || maximum < minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "minimum humidity must be smaller than the maximum humidity, set humidity range failed");
    }

    /* set max and min ranges */
    wx_humidity_range = (wx_scalar_range_t){ .maximum = maximum, .minimum = minimum };

    return ESP_OK;
}

esp_err_t wx_td_range(const float temperature, const float humidity, float *const dewpoint) {
    /* validate arguments */
    ESP_ARG_CHECK( dewpoint );

    // validate range of temperature parameter
    if(temperature > wx_temperature_range.maximum || temperature < wx_temperature_range.minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "temperature is out of range, calculate dewpoint failed");
    }

    // validate range of humidity parameter
    if(humidity > wx_humidity_range.maximum || humidity < wx_humidity_range.minimum) {
        ESP_RETURN_ON_FALSE( false, ESP_ERR_INVALID_ARG, TAG, "humidity is out of range, calculate dewpoint failed");
    }

    // calculate dew-point temperature
    //double H = (log10(humidity)-2)/0.4343 + (17.62*temperature)/(243.12+temperature);
    //*dewpoint = 243.12*H/(17.62-H);

    *dewpoint = wx_td(temperature, humidity);
    
    return ESP_OK;
}




const double wx_c_to_k(const double t) {
    return t + 273.15;
}

const double wx_k_to_c(const double t) {
    return t - 273.15;
}

const double wx_v(const double t) {
    const double c0 = 0.4931358;
    const double c1 = -0.0046094296;
    const double c2 = 0.000013746454;
    const double c3 = -0.000000012743214;

    return t - c0 - c1 * t - c2 * pow(t, 2) - c3 * pow(t, 3);
}

const double wx_pwi(const double t) {
    const double a0 = -5674.5359;
    const double a1 = 6.3925247;
    const double a2 = -9.677843E-03;
    const double a3 = 0.00000062215701;
    const double a4 = 2.0747825E-09;
    const double a5 = -9.484024E-13;
    const double a6 = 4.1635019;

    return exp( (a0 / t) + a1 + (a2 * t) + (a3 * pow(t, 2)) + (a4 * pow(t, 3)) + (a5 * pow(t, 4)) + (a6 * log(t)) ) * 0.01;
}

const double wx_pws(const double t) {
    const double b_1 = -5800.2206;
    const double b0 = 1.3914993;
    const double b1 = -0.048640239;
    const double b2 = 0.000041764768;
    const double b3 = -0.000000014452093;
    const double b4 = 6.5459673;
    const double v = wx_v(t);

    return exp( (b_1 / v) + b0 + (b1 * v) + (b2 * pow(v, 2)) + (b3 * pow(v, 3)) + (b4 * log(v)) ) * 0.01;
}


const double wx_pressure_at_sea_level(const double ph, const double a, const double t) {
    // see https://keisan.casio.com/exec/system/1224575267

    // precalculate altitude, corrected for the temperature lapse rate
    const double hl = a * 0.0065;

    return ph / pow(1.0-hl/(t+hl+273.15),5.257);
}

const double wx_td(const double ta, const double hr) {
    const double a1 = 243.12;
    const double a2 = 17.62;
    const double H = log( hr * exp( (a2 * ta) / (a1 + ta) ) / 100 );

    return (a1 * H) / (a2 - H);
}

static inline const double wx_phit(const double ta, const double tw, const double pa) {
    const double pws_tw = wx_pws(wx_c_to_k(tw));

    return pws_tw - 0.000662 * pa * (ta - tw);
}

static inline const double wx_pder(const double ta, const double tw, const double phit) {
    return NAN;
}

const double wx_tw(const double ta, const double hr, const double td, const double pa) {
    const double pws_td = wx_pws(wx_c_to_k(td));

    double twet_init = 243.12 * ( log(pws_td / 6.112) / (17.62 - log(pws_td / 6.112)) );
    return NAN;
}