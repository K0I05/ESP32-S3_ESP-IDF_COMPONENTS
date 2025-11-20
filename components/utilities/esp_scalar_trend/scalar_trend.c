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
 * @file scalar_trend.c
 *
 * Scalar trend libary
 * 
 * Scalar Trend appears after one (1) hour of operation. The trend codes are a 
 * forecast of the 3-hr change based on the previous 1-hour history.
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 * 
 * Original sources
 * https://gist.github.com/Paraphraser/c5609f85cc7ee6ecd03ce179fb7f7edb
 * https://github.com/oyve/barometer-trend/blob/main/predictions/byPressureTrend.js
 * 
 */
#include <esp_check.h>
#include <esp_log.h>
#include <esp_types.h>

#include <math.h>

#include <scalar_trend.h>

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define SQR(x) ((x) * (x))

/**
 * @brief Scalar trend context descriptor structure definition.
 */
typedef struct scalar_trend_context_s {
    double      critical_t;    /*!< scalar trend samples absolute critical t value, state machine variable */
    uint16_t    samples_count; /*!< scalar trend samples count, state machine variable */
    uint16_t    samples_size;  /*!< scalar trend samples size, state machine variable */
    double*     samples;       /*!< scalar trend samples array, state machine variable */
} scalar_trend_context_t;

/*
* static constant declarations
*/
static const char *TAG = "scalar_trend";

/**
 * @brief Calculates q-distribution by α (probability).
 * 
 * @note Original source: https://my.originlab.com/forum/topic.asp?TOPIC_ID=5776
 * 
 * @param x 
 * @return double 
 */
static inline double q_inv(const double x) {
	double result;
	double sum1, sum2;
	double tempo, xp, xa = x;
	int i;
	
	const double c[4] = {2.515517, 0.802853, 0.010328, 0.0};
	const double d[4] = {1.0, 1.432788, 0.189269, 0.001308};
	
	if(xa<=0.0)
		xa = 0.0001;
	else if(xa>=1.0)
		xa = 0.9999;
	
	if (xa<0.5)
		tempo = sqrt(log(1.0/SQR(xa)));
	else
		tempo = sqrt(log(1.0/SQR(1.0-xa)));
	
	sum1 = 0.0;
	sum2 = 0.0;
	xp = 1.0;
	
	for(i = 0;i<4;i++) {
		sum1 += c[i] * xp;
		sum2 += d[i] * xp;
		xp *= tempo;
	}

	result = tempo - sum1 / sum2;
	result = (xa > 0.5)? -result : result;
	return result;
}

/**
 * @brief Calculates the left-tailed inverse student's t-distribution by α (probability) and v (degree of freedom).
 * 
 * Source: https://my.originlab.com/forum/topic.asp?TOPIC_ID=5776
 * 
 * @param x α argument of the left-tailed inverse student's t-distribution.
 * @param df v argument of the left-tailed inverse student's t-distribution
 * @return double Left-tailed inverse student's t-distribution value.
 */
static inline double t_inv(const double x, const double df) {
	double sum, xp, xq, xa = x;
	double Pwr[10];
	double term[5];
	int i;
	
	if(xa <= 0.0)
		xa = 0.0001;
	else if (xa >= 1.0)
		xa = 0.9999;
	
	xq = q_inv(xa);
	Pwr[1] = xq;
	for (i = 2;i<=9;i++)
		Pwr[i] = Pwr[i-1] * xq;
	
	term[1] = 0.25 * ( Pwr[3] + Pwr[1] );
	term[2] = (5*Pwr[5] + 16*Pwr[3] + 3*Pwr[1])/96;
	term[3] = (3*Pwr[7] + 19*Pwr[7] + 17*Pwr[3] - 15*Pwr[1])/384;
	term[4] = (79*Pwr[9] + 776*Pwr[7] + 1482*Pwr[5] - 1920*Pwr[3] - 945*Pwr[1])/92160.0;
	
	sum=xq;
	xp = 1;
	for(i = 1;i<=4;i++) {
		xp *=df;
		sum += term[i] / xp;
	}
	return sum;
}

const char* scalar_trend_code_to_string(const scalar_trend_codes_t code) {
    switch(code) {
        case SCALAR_TREND_CODE_UNKNOWN:
            return "Unkown";
        case SCALAR_TREND_CODE_RISING:
            return "Rising";
        case SCALAR_TREND_CODE_STEADY:
            return "Steady";
        case SCALAR_TREND_CODE_FALLING:
            return "Falling";
        default:
            return "Unkown";
    }
}

esp_err_t scalar_trend_init(const uint16_t samples_size, scalar_trend_handle_t *scalar_trend_handle) {
    esp_err_t  ret = ESP_OK;

    /* validate arguments */
    ESP_GOTO_ON_FALSE( samples_size > 2, ESP_ERR_INVALID_ARG, err, TAG, "samples size must be greater than 2, scalar trend handle initialization failed" );

    /* validate memory availability for scalar trend handle */
    scalar_trend_context_t* ctxt = (scalar_trend_context_t*)calloc(1, sizeof(scalar_trend_context_t)); 
    ESP_GOTO_ON_FALSE( ctxt, ESP_ERR_NO_MEM, err, TAG, "no memory for scalar trend handle, scalar trend handle initialization failed" );

    /* validate memory availability for samples array */
    ctxt->samples = (double*)calloc(samples_size, sizeof(double));
    ESP_GOTO_ON_FALSE( ctxt->samples, ESP_ERR_NO_MEM, err_out_handle, TAG, "no memory for scalar trend handle samples, scalar trend handle initialization failed" );

    /* calculate absolute critical t value and copy configuration */
    ctxt->critical_t   = fabs(t_inv(0.05/2, samples_size - 2));
    ctxt->samples_size = samples_size;

    /* set output instance */
    *scalar_trend_handle = (scalar_trend_handle_t)ctxt;

    return ESP_OK;

    err_out_handle:
        free(ctxt);
    err:
        return ret;
}

esp_err_t scalar_trend_analysis(scalar_trend_handle_t handle, 
                                const double sample, 
                                scalar_trend_codes_t *const code) {
    scalar_trend_context_t* ctxt = (scalar_trend_context_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK(ctxt);

    // have we filled the array?
    if (ctxt->samples_count < ctxt->samples_size) {
        // no! add this observation to the array
        ctxt->samples[ctxt->samples_count] = sample;

        // bump n
        ctxt->samples_count++;
    } else {
        // yes! the array is full so we have to make space
        for (uint16_t i = 1; i < ctxt->samples_size; i++) {
            ctxt->samples[i-1] = ctxt->samples[i];
        }

        // now we can fill in the last slot
        ctxt->samples[ctxt->samples_size-1] = sample;
    }

    // is the array full yet?
    if (ctxt->samples_count < ctxt->samples_size) {
        // no! we are still training
        *code = SCALAR_TREND_CODE_UNKNOWN;

        return ESP_OK;
    }

    /*
     * Step 1 : calculate the straight line of best fit
     *          (least-squares linear regression)
     */

    double sum_x  = 0.0;     // ∑(x)
    double sum_xx = 0.0;    // ∑(x²)
    double sum_y  = 0.0;     // ∑(y)
    double sum_xy = 0.0;    // ∑(xy)
    
    // we need n in lots of places and it's convenient as a double
    const double n = 1.0 * ctxt->samples_size;

    // iterate to calculate the above values
    for (uint16_t i = 0; i < ctxt->samples_size; i++) {
        const double x = 1.0 * i;
        const double y = ctxt->samples[i];

        sum_x = sum_x + x;
        sum_xx = sum_xx + x * x;
        sum_y = sum_y + y;
        sum_xy = sum_xy + x * y;
    }

    // calculate the slope and intercept
    const double slope = (sum_x*sum_y - n*sum_xy) / (sum_x*sum_x - n*sum_xx);
    const double intercept = (sum_y -slope*sum_x) / n;

    /*
     * Step 2 : Perform an hypothesis test on the equation of the linear
     *          model to see whether, statistically, the available data
     *          contains sufficient evidence to conclude that the slope
     *          is non-zero.
     *          
     *          Let beta1 = the slope of the regression line between
     *          fixed time intervals and pressure observations.
     *          
     *          H0: β₁ = 0    (the slope is zero)
     *          H1: β₁ ≠ 0    (the slope is not zero)
     *          
     *          The level of significance: α is 5% (0.05)
     *          
     *          The test statistic is:
     *          
     *              tObserved = (b₁ - β₁) / s_b₁
     *              
     *          In this context, b₁ is the estimated slope of the linear
     *          model and β₁ the reference value from the hypothesis
     *          being tested. s_b₁ is the standard error of b₁.
     *
     *          From H0, β₁ = 0 so the test statistic simplifies to:
     * 
     *              tObserved = b₁ / s_b₁
     *      
     *          This is a two-tailed test so half of α goes on each side
     *          of the T distribution.
     *          
     *          The degrees-of-freedom, ν, for the test is:
     *          
     *              ν = n-2 = 6 - 2 = 4
     *              
     *          The critical value (calculated externally using Excel or
     *          a graphics calculator) is:
     * 
     *              -tCritical = invt(0.05/2,4) = -2.776445105
     *      
     *          By symmetry:
     * 
     *              +tCritical = abs(-tCritical)
     *              
     *          The decision rule is:
     * 
     *              reject H0 if tObserved < -tCritical or 
     *                           tObserved > +tCritical
     *      
     *          which can be simplified to:
     * 
     *              reject H0 if abs(tObserved) > +tCritical
     * 
     *          Note that the value of +tCritical is carried in the
     *          global variable:
     *          
     *              Critical_t_value
     *              
     *          The next step is to calculate the test statistic but one
     *          of the inputs to that calculation is SSE, so we need
     *          that first.
     *      
     */

    double SSE = 0.0;        // ∑((y-ŷ)²)

    // iterate
    for (uint16_t i = 0; i < ctxt->samples_size; i++) {
        const double y = ctxt->samples[i];
        const double residual = y - (intercept + slope * i);
        SSE = SSE + residual * residual;
    }

    /*    
     *          Now we can calculate the test statistic. Note the use
     *          of the fabs() function below to force the result into
     *          the positive domain for comparison with Critical_t_value
     */
    const double tObserved =
        fabs(
           slope/(sqrt(SSE / (n-2.0)) / sqrt(sum_xx - sum_x*sum_x/n))
        );

    /*    
     *          Finally, make the decision and return a string
     *          summarizing the conclusion.
     */

    // is tObserved further to the left or right than tCritical?
    if (tObserved > ctxt->critical_t) {
    
        // yes! what is the sign of the slope?
        if (slope < 0.0) {
            /* falling */
            *code = SCALAR_TREND_CODE_FALLING;
            return ESP_OK;
        } else {
            /* rising */
            *code = SCALAR_TREND_CODE_RISING;
            return ESP_OK;
        }
    }

    // otherwise, the slope may be zero (statistically)
    *code = SCALAR_TREND_CODE_STEADY;

    return ESP_OK;
}

esp_err_t scalar_trend_reset(scalar_trend_handle_t handle) {
    scalar_trend_context_t* ctxt = (scalar_trend_context_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK(ctxt);

    /* purge samples */
    for(uint16_t i = 0; i < ctxt->samples_size; i++) {
        ctxt->samples[i] = NAN;
    }

    /* reset samples counter */
    ctxt->samples_count = 0;

    return ESP_OK;
}

esp_err_t scalar_trend_delete(scalar_trend_handle_t handle) {
    scalar_trend_context_t* ctxt = (scalar_trend_context_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK(ctxt);

    if(ctxt->samples) 
        free(ctxt->samples);
    free(handle);
    
    return ESP_OK;
}

const char* scalar_trend_get_fw_version(void) {
    return (const char*)SCALAR_TREND_FW_VERSION_STR;
}

int32_t scalar_trend_get_fw_version_number(void) {
    return (int32_t)SCALAR_TREND_FW_VERSION_INT32;
}