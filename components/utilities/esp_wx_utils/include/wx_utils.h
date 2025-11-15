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
 * @file wx_utils.h
 * @defgroup wx_utils
 * @{
 *
 * ESP-IDF weather utilities
 * 
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __WX_UTILS_H__
#define __WX_UTILS_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


/*
 * weather utilities enumerator and structure declarations
*/

typedef struct {
    float maximum;
    float minimum;
} wx_scalar_range_t;



/*
* function and subroutine declarations
*/

/**
 * @brief Sets the global weather utilities maximum and minimum temperature range (`wx_temperature_range`) in degrees Celsius.
 * 
 * @param maximum Maximum temperature range in degrees Celsius.
 * @param minimum Minimum temperature range in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t wx_set_temperature_range(const float maximum, const float minimum);

/**
 * @brief Sets the global weather utilities humidity maximum and minimum range (`wx_humidity_range`) in percent.
 * 
 * @param maximum Maximum humidity range in percent.
 * @param minimum Minimum humidity range in percent.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t wx_set_humidity_range(const float maximum, const float minimum);

/**
 * @brief Calculates dewpoint temperature from air temperature and relative humidity with range validation.  The 
 * default temperature range is from -40 to 125 degrees Celsius and default humidity range is from 0 to 100 percent.  
 * The default ranges can be adjusted through the set range functions.
 *
 * @param[in] temperature Air temperature in degrees Celsius.
 * @param[in] humidity Relative humidity in percent.
 * @param[out] dewpoint Calculated dewpoint temperature in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t wx_td_range(const float temperature, const float humidity, float *const dewpoint);




/**
 * @brief Converts degrees celsius to kelvin.
 * 
 * @param t Temperature in degrees celsius.
 * @return double Temperature in kelvin.
 */
const double wx_c_to_k(const double t);

/**
 * @brief Converts kelvin to degrees celsius
 * 
 * @param t Temperature in kelvin.
 * @return double Temperature in degrees celsius.
 */
const double wx_k_to_c(const double t);

/**
 * @brief Calculates v at temperature.
 * 
 * @param t Temperature in kelvin.
 * @return double v at temperature.
 */
const double wx_v(const double t);

/**
 * @brief Calculates aqueous vapor pressure of ice at temperature.
 * 
 * @param t Temperature in kelvin.
 * @return double Aqueous vapor pressure of ice
 */
const double wx_pwi(const double t);

/**
 * @brief Calculates aqueous vapor pressure of water at temperature.
 * 
 * @param t Temperature in kelvin.
 * @return double Aqueous vapor pressure of water.
 */
const double wx_pws(const double t);

/**
 * @brief Calculates the reduced air pressure QFF (pressure at sea level) without QFE.
 * 
 * @param pa Air pressure at this altitude in hecto-pascal.
 * @param l Reduction level at this altitude in meters.
 * @param ta Air temperature at this altitude in degrees celsius.
 * @return float Reduced air pressure QFF in hecto-pascal.
 */
const double wx_pressure_at_sea_level(const double pa, const double l, const double ta);

/**
 * @brief Calculates the reduced air pressure QFE (pressure at a certain level).
 * 
 * @note The reduction level for QFE processing is the elevation difference of 
 * the pressure sensor and the QFE level into which the pressure will be reduced. 
 * If the pressure sensor is above the QFE level, the reduction level to get the 
 * QFE level pressure is a positive value. If the sensor is below the QFE level, 
 * the reduction level is a negative value.
 * 
 * @param pa Air pressure at this altitude in hecto-pascal.
 * @param l Reduction level in meters.
 * @param ta Air temperature at this altitude in degrees celsius.
 * @return float Reduced air pressure QFE in hecto-pascal.
 */
const double wx_qfe(const double pa, const double l, const double ta);

/**
 * @brief Calculates the reduced air pressure QFF (pressure at sea level).
 * 
 * @note The reduction level is the elevation difference of the station altitude 
 * and the mean sea level.
 * 
 * @param qfe Field elevation aire pressure in hecto-pascal.
 * @param l Reduction level at this altitude in meters.
 * @param ta Air temperature at this altitude in degrees celsius.
 * @return float Reduced air pressure QFF in hecto-pascal.
 */
const double wx_qff(const double qfe, const double l, const double ta);

/**
 * @brief Calculates the reduced air pressure QNH (pressure at sea 
 * level according to ICAO standard atmospheric).
 * 
 * @param qfe Staion level air pressure in hecto-pascal.
 * @param h Elevation of pressure QFE in International Standard Atmosphere (ISA).
 * @param a Station altitude in meters.
 * @return float Reduced air pressure QNH in hecto-pascal.
 */
const double wx_qnh(const double qfe, const double h, const double a);

/**
 * @brief Calculates dewpoint temperature from air temperature and relative
 * humidity.
 * 
 * @param ta Air temperature in degrees celsius.
 * @param hr Relative humidity in percent.
 * @return double Dewpoint temperature in degrees celsius.
 */
const double wx_td(const double ta, const double hr);

/**
 * @brief Calculates wetbulb temperature from air temperature, relative
 * humidity, and dewpoint temperature.
 * 
 * @param ta Air temperature in degrees celsius.
 * @param hr Relative humidity in percent.
 * @param td Dewpoint temperature in degrees celsius.
 * @param pa Air pressure in hecto-pascal.
 * @return double Wetbulb temperature in degrees celsius.
 */
const double wx_tw(const double ta, const double hr, const double td, const double pa);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __WX_UTILS_H__
