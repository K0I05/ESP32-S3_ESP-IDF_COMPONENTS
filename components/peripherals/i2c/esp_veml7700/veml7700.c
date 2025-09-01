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
 * @file veml7700.c
 *
 * ESP-IDF driver for VEML7700 illuminance sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/veml7700.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * VEML7700 definitions
*/
#define VEML7700_DEVICE_ID          UINT8_C(0x81)   //!< veml7700 device identifier (fixed)
#define VEML7700_POLY_COEF_A        (6.0135e-13)
#define VEML7700_POLY_COEF_B        (-9.3924e-9)
#define VEML7700_POLY_COEF_C        (8.1488e-5)
#define VEML7700_POLY_COEF_D        (1.0023)

#define VEML7700_CMD_ALS_CONF       UINT8_C(0x00)
#define VEML7700_CMD_ALS_WH         UINT8_C(0x01)
#define VEML7700_CMD_ALS_WL         UINT8_C(0x02)
#define VEML7700_CMD_POWER_SAVING   UINT8_C(0x03)
#define VEML7700_CMD_ALS            UINT8_C(0x04)
#define VEML7700_CMD_WHITE          UINT8_C(0x05)
#define VEML7700_CMD_ALS_INT        UINT8_C(0x06)
#define VEML7700_CMD_ID             UINT8_C(0x07)

#define VEML7700_POWERUP_DELAY_MS   UINT16_C(5)     /*!< veml7700 delay on power-up before attempting I2C transactions */
#define VEML7700_APPSTART_DELAY_MS  UINT16_C(10)    /*!< veml7700 delay after initialization before application start-up */
#define VEML7700_CMD_DELAY_MS       UINT16_C(5)     /*!< veml7700 delay before attempting I2C transactions after a command is issued */
#define VEML7700_RETRY_DELAY_MS     UINT16_C(2)     /*!< veml7700 delay between an I2C receive transaction retry */
#define VEML7700_TX_RX_DELAY_MS     UINT16_C(10)    /*!< veml7700 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define VEML7700_GAIN_OPTIONS_COUNT UINT8_C(4)	    /*!< Possible gain values count */
#define VEML7700_IT_TIMES_COUNT     UINT8_C(6)      /*!< Possible integration time values count */
#define VEML7700_IT_OPTIONS_COUNT   UINT8_C(2)      /*!< Possible integration time values count */
#define VEML7700_PSM_TIMES_COUNT    UINT8_C(24) 
#define VEML7700_PSM_OPTIONS_COUNT  UINT8_C(4)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief VEML7700 device descriptor structure definition.
 */
typedef struct veml7700_device_s {
    veml7700_config_t                       config;                 /*!< veml7700 device configuration */
    i2c_master_dev_handle_t                 i2c_handle;             /*!< veml7700 i2c device handle */
    //float                                 resolution;			    /*!< Current resolution and multiplier */
    //uint32_t                              maximum_lux;		    /*!< Current maximum lux limit */
} veml7700_device_t;

/*
* static constant declarations
*/
static const char *TAG = "veml7700";

/**
 * @brief List of all possible values for configuring sensor gain.
 * 
 */
static const uint8_t veml7700_gains[VEML7700_GAIN_OPTIONS_COUNT] = {
    VEML7700_GAIN_DIV_8,
    VEML7700_GAIN_DIV_4,
    VEML7700_GAIN_1,
    VEML7700_GAIN_2
};

/**
 * @brief List of all possible values for configuring sensor integration time.
 * 
 */
static const uint8_t veml7700_integration_times[VEML7700_IT_TIMES_COUNT] = {
    VEML7700_INTEGRATION_TIME_25MS,  
    VEML7700_INTEGRATION_TIME_50MS,  
    VEML7700_INTEGRATION_TIME_100MS, 
    VEML7700_INTEGRATION_TIME_200MS, 
    VEML7700_INTEGRATION_TIME_400MS, 
    VEML7700_INTEGRATION_TIME_800MS,
};


static const uint16_t veml7700_integration_time_map[VEML7700_IT_TIMES_COUNT][VEML7700_IT_OPTIONS_COUNT] = {
    {VEML7700_INTEGRATION_TIME_25MS,  24},
    {VEML7700_INTEGRATION_TIME_50MS,  50},
    {VEML7700_INTEGRATION_TIME_100MS, 100},
    {VEML7700_INTEGRATION_TIME_200MS, 200},
    {VEML7700_INTEGRATION_TIME_400MS, 400},
    {VEML7700_INTEGRATION_TIME_800MS, 800}
};

static const uint16_t veml7700_psm_refresh_time_map[VEML7700_PSM_TIMES_COUNT][VEML7700_PSM_OPTIONS_COUNT] = {
    {VEML7700_POWER_SAVING_MODE_1, VEML7700_INTEGRATION_TIME_25MS, 25, 525},
    {VEML7700_POWER_SAVING_MODE_2, VEML7700_INTEGRATION_TIME_25MS, 25, 1025},
    {VEML7700_POWER_SAVING_MODE_3, VEML7700_INTEGRATION_TIME_25MS, 25, 2025},
    {VEML7700_POWER_SAVING_MODE_4, VEML7700_INTEGRATION_TIME_25MS, 25, 4025},
    {VEML7700_POWER_SAVING_MODE_1, VEML7700_INTEGRATION_TIME_50MS, 50, 550},
    {VEML7700_POWER_SAVING_MODE_2, VEML7700_INTEGRATION_TIME_50MS, 50, 1050},
    {VEML7700_POWER_SAVING_MODE_3, VEML7700_INTEGRATION_TIME_50MS, 50, 2050},
    {VEML7700_POWER_SAVING_MODE_4, VEML7700_INTEGRATION_TIME_50MS, 50, 4050},
    {VEML7700_POWER_SAVING_MODE_1, VEML7700_INTEGRATION_TIME_100MS, 100, 600},
    {VEML7700_POWER_SAVING_MODE_2, VEML7700_INTEGRATION_TIME_100MS, 100, 1100},
    {VEML7700_POWER_SAVING_MODE_3, VEML7700_INTEGRATION_TIME_100MS, 100, 2100},
    {VEML7700_POWER_SAVING_MODE_4, VEML7700_INTEGRATION_TIME_100MS, 100, 4100},
    {VEML7700_POWER_SAVING_MODE_1, VEML7700_INTEGRATION_TIME_200MS, 200, 700},
    {VEML7700_POWER_SAVING_MODE_2, VEML7700_INTEGRATION_TIME_200MS, 200, 1200},
    {VEML7700_POWER_SAVING_MODE_3, VEML7700_INTEGRATION_TIME_200MS, 200, 2200},
    {VEML7700_POWER_SAVING_MODE_4, VEML7700_INTEGRATION_TIME_200MS, 200, 4200},
    {VEML7700_POWER_SAVING_MODE_1, VEML7700_INTEGRATION_TIME_400MS, 400, 900},
    {VEML7700_POWER_SAVING_MODE_2, VEML7700_INTEGRATION_TIME_400MS, 400, 1400},
    {VEML7700_POWER_SAVING_MODE_3, VEML7700_INTEGRATION_TIME_400MS, 400, 2400},
    {VEML7700_POWER_SAVING_MODE_4, VEML7700_INTEGRATION_TIME_400MS, 400, 4400},
    {VEML7700_POWER_SAVING_MODE_1, VEML7700_INTEGRATION_TIME_800MS, 800, 1300},
    {VEML7700_POWER_SAVING_MODE_2, VEML7700_INTEGRATION_TIME_800MS, 800, 1800},
    {VEML7700_POWER_SAVING_MODE_3, VEML7700_INTEGRATION_TIME_800MS, 800, 2800},
    {VEML7700_POWER_SAVING_MODE_4, VEML7700_INTEGRATION_TIME_800MS, 800, 4800}
};


/**
 * @brief Proper resolution multipliers mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 17-Jan-2024
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const float veml7700_resolution_map[VEML7700_IT_TIMES_COUNT][VEML7700_GAIN_OPTIONS_COUNT] = {
    {0.0042, 0.0084, 0.0336, 0.0672},
    {0.0084, 0.0168, 0.0672, 0.1344},
    {0.0168, 0.0336, 0.1344, 0.2688},
    {0.0336, 0.0672, 0.2688, 0.5376},
    {0.0672, 0.1344, 0.5376, 1.0752},
    {0.1344, 0.2688, 1.0752, 2.1504}
};

/**
 * @brief Maximum luminocity mapped to gain-integration time combination.
 * 
 * @note Source: Official Vishay VEML7700 Application Note, rev. 17-Jan-2024
 * 
 * @link https://www.vishay.com/docs/84323/designingveml7700.pdf
 */
static const uint32_t veml7700_maximums_map[VEML7700_IT_TIMES_COUNT][VEML7700_GAIN_OPTIONS_COUNT] = {
    {275,   550,    2202,   4404},
    {550,   1101,   4404,   8808},
    {1101,  2202,   8808,   17616},
    {2202,  4404,   17616,  35232},
    {4404,  8808,   35232,  70463},
    {8808,  17616,  70463,  140926}
};

/**
 * @brief Finds the index of a given element within an array.
 * 
 * This is a standard implementation of a commonly used function which can be
 * found online.
 * 
 * @param elm		Value of the element we are searching for
 * @param ar 		The array in which to search
 * @param len 		Length of the given array
 * 
 * @return uint8_t 
 * 		- n Index within the array
 * 		- -1 Element not found.
 */
static inline uint8_t veml7700_index_of(const uint8_t elm, const uint8_t *ar, const uint8_t len) {
    uint8_t size = len;
    while (size--) { if(ar[size] == elm) { return size; } } return -1;
}

/**
 * @brief Gets the index of the integration time value within the list of possible
 * integration time values.
 * 
 * @param integration_time The integration time value to search for.
 * 
 * @return int The index within the array of possible integration times.
 */
static inline int veml7700_get_it_index(const veml7700_integration_times_t integration_time) {
	return veml7700_index_of(integration_time, veml7700_integration_times, VEML7700_IT_TIMES_COUNT);
}

/**
 * @brief Gets the index of the gain value within the list of possible
 * gain values.
 * 
 * @param gain The gain value to search for.
 * 
 * @return int The index within the array of possible gains.
 */
static inline int veml7700_get_gain_index(const veml7700_gains_t gain) {
	return veml7700_index_of(gain, veml7700_gains, VEML7700_GAIN_OPTIONS_COUNT);
}

/**
 * @brief Gets the index of the power saving mode and integration time value within the list of possible values.
 * 
 * @param psm_mode The power saving mode value to search for.
 * @param integration_time The integration time value to search for.
 * @return int The index within the array of possible values.
 */
static inline int veml7700_get_psm_mode_index(const veml7700_power_saving_modes_t psm_mode, const veml7700_integration_times_t integration_time) {
    uint8_t size = VEML7700_PSM_TIMES_COUNT;
    while (size--) { 
        if(veml7700_psm_refresh_time_map[size][0] == psm_mode && veml7700_psm_refresh_time_map[size][1] == integration_time) { 
            return size; 
        } 
    } 
    return -1;
}

/**
 * @brief The maximum possible lux value for any configuration on this
 * sensor.
 * 
 * @return uint32_t The maximum lux value.
 */
static inline uint32_t veml7700_get_maximum_lux(void) {
	return veml7700_maximums_map[VEML7700_IT_TIMES_COUNT - 1][VEML7700_GAIN_OPTIONS_COUNT - 1];
}

/**
 * @brief Gets the smallest possible maximum lux value for this sensor.
 * 
 * @return uint32_t The smallest maximum lux value.
 */
static inline uint32_t veml7700_get_lowest_maximum_lux(void) {
	return veml7700_maximums_map[0][0];
}

/**
 * @brief Gets the next smallest maximum lux limit value.
 * 
 * Used to identify if a better range is possible for the current 
 * light levels.
 * 
 * @param device VEML7700 device descriptor.
 * 
 * @return uint32_t The next smallest maximum lux value.
 */
static inline uint32_t veml7700_get_lower_maximum_lux(veml7700_device_t *const device) {
	int gain_index = veml7700_get_gain_index(device->config.gain);
	int it_index = veml7700_get_it_index(device->config.integration_time);

	// find the next smallest 'maximum' value in the mapped maximum luminosities
	if ((gain_index > 0) && (it_index > 0)) {
		if (veml7700_maximums_map[it_index][gain_index - 1] >= veml7700_maximums_map[it_index - 1][gain_index]) {
			return veml7700_maximums_map[it_index][gain_index - 1];
		} else {
			return veml7700_maximums_map[it_index - 1][gain_index];
		}
	} else if ((gain_index > 0) && (it_index == 0)) {
		return veml7700_maximums_map[it_index][gain_index - 1];
	} else {
		return veml7700_maximums_map[it_index - 1][gain_index];
	}
}

/**
 * @brief Reads the maximum lux for the current configuration.
 * 
 * @param device VEML7700 device descriptor.
 * @return uint32_t The maximum lux value.
 */
static inline uint32_t veml7700_get_current_maximum_lux(veml7700_device_t *const device) {
	int gain_index = veml7700_get_gain_index(device->config.gain);
	int it_index = veml7700_get_it_index(device->config.integration_time);

	return veml7700_maximums_map[it_index][gain_index];
}

/**
 * @brief Reads gain and integration time to calculate resolution.
 * 
 * @param device VEML7700 device descriptor.
 * @return float Calculated resolution.
 */
static inline float veml7700_get_resolution(veml7700_device_t *const device)
{
	int gain_index = veml7700_get_gain_index(device->config.gain);
	int it_index = veml7700_get_it_index(device->config.integration_time);

	return veml7700_resolution_map[it_index][gain_index];
}

esp_err_t veml7700_optimize_configuration(veml7700_handle_t handle) {
    uint16_t als_counts;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* enable power */
    ESP_RETURN_ON_ERROR( veml7700_enable(handle), TAG, "enable power failed" );

    /* set baseline integration time and gain values */
    veml7700_configuration_register_t config_reg;
    int it_index                     = veml7700_get_it_index(VEML7700_INTEGRATION_TIME_100MS); /* set baseline integration time to 100ms */
    int gain_index                   = veml7700_get_gain_index(VEML7700_GAIN_DIV_8);           /* set baseline gain to 1/8 */
    config_reg.bits.integration_time = veml7700_integration_times[it_index];
    config_reg.bits.gain             = veml7700_gains[gain_index];

    /* set baseline configuration */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config_reg), TAG, "write configuration register failed" );

    /* read ambient light counts */
    ESP_RETURN_ON_ERROR( veml7700_get_ambient_light_counts(handle, &als_counts), TAG, "read ambient light counts failed" );

    if (als_counts <= 100) {
        // increase gain and then integration time as needed
        while ((als_counts <= 100) && !((gain_index == 3) && (it_index == 5))) {
            if (gain_index < 3) {
                // increase and set gain
                ESP_RETURN_ON_ERROR(veml7700_set_gain(handle, veml7700_gains[++gain_index]), TAG, "write gain failed" );
            } else if (it_index < 5) {
                // increase and set integration time
                ESP_RETURN_ON_ERROR(veml7700_set_integration_time(handle, veml7700_integration_times[++it_index]), TAG, "write integration time failed" );
            }
            /* read ambient light counts */
            ESP_RETURN_ON_ERROR( veml7700_get_ambient_light_counts(handle, &als_counts), TAG, "read ambient light counts failed" );
        }

        ESP_LOGI(TAG, "IT     %d", dev->config.integration_time);
        ESP_LOGI(TAG, "Gain   %d", dev->config.gain);
    } else {
        // decrease integration time as needed
        while ((als_counts > 10000) && (it_index > 0)) {
            // decrease and set integration time
            ESP_RETURN_ON_ERROR(veml7700_set_integration_time(handle, veml7700_integration_times[--it_index]), TAG, "write integration time failed" );

            /* read ambient light counts */
            ESP_RETURN_ON_ERROR( veml7700_get_ambient_light_counts(handle, &als_counts), TAG, "read ambient light counts failed" );
        }

        ESP_LOGI(TAG, "IT     %d", dev->config.integration_time);
        ESP_LOGI(TAG, "Gain   %d", dev->config.gain);
    }

    return ESP_OK;
}

/**
 * @brief VEML7700 I2C HAL read word from register address transaction.
 * 
 * @param device VEML7700 device descriptor.
 * @param reg_addr VEML7700 register address to read from.
 * @param word VEML7700 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t veml7700_i2c_read_word_from(veml7700_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "veml7700_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief VEML7700 I2C HAL write halfword to register address transaction.
 * 
 * @param device VEML7700 device descriptor.
 * @param reg_addr VEML7700 register address to write to.
 * @param word VEML7700 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t veml7700_i2c_write_word_to(veml7700_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

esp_err_t veml7700_get_configuration_register(veml7700_handle_t handle, veml7700_configuration_register_t *const reg) {
    uint16_t config = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_ALS_CONF, &config), TAG, "read configuration register failed" );

    /* set output parameter */
    reg->reg = config;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_set_configuration_register(veml7700_handle_t handle, const veml7700_configuration_register_t reg) {
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    veml7700_configuration_register_t config = { .reg = reg.reg };

    /* set reserved values */
    config.bits.reserved1 = 0;
    config.bits.reserved2 = 0;
    config.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_write_word_to(dev, VEML7700_CMD_ALS_CONF, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_get_threshold_registers(veml7700_handle_t handle, uint16_t *const hi_threshold, uint16_t *const lo_threshold) {
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_ALS_WH, hi_threshold), TAG, "read high threshold register failed" );
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_ALS_WL, lo_threshold), TAG, "read low threshold register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_set_threshold_registers(veml7700_handle_t handle, const uint16_t hi_threshold, const uint16_t lo_threshold) {
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_write_word_to(dev, VEML7700_CMD_ALS_WH, hi_threshold), TAG, "write high threshold register failed" );
    
    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_write_word_to(dev, VEML7700_CMD_ALS_WL, lo_threshold), TAG, "write low threshold register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_get_power_saving_mode_register(veml7700_handle_t handle, veml7700_power_saving_mode_register_t *const reg) {
    uint16_t psm = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_POWER_SAVING, &psm), TAG, "read power saving mode register failed" );

    /* set output parameter */
    reg->reg = psm;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_set_power_saving_mode_register(veml7700_handle_t handle, const veml7700_power_saving_mode_register_t power_saving_mode_reg) {
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    veml7700_power_saving_mode_register_t power_saving_mode = { .reg = power_saving_mode_reg.reg };

    /* set reserved values */
    power_saving_mode.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_write_word_to(dev, VEML7700_CMD_POWER_SAVING, power_saving_mode.reg), TAG, "write power saving mode register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_get_interrupt_status_register(veml7700_handle_t handle, veml7700_interrupt_status_register_t *const reg) {
    uint16_t irq = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_ALS_INT, &irq), TAG, "read interrupt status register failed" );

    /* set handle register */
    reg->reg = irq;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t i2c_veml7700_get_identifier_register(veml7700_handle_t handle, veml7700_identifier_register_t *const reg) {
    uint16_t ident = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_ID, &ident), TAG, "read identifier register failed" );

    /* set handle register */
    reg->reg = ident;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t veml7700_init(i2c_master_bus_handle_t master_handle, const veml7700_config_t *veml7700_config, veml7700_handle_t *veml7700_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && veml7700_config );

    /* power-up delay */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, veml7700_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, veml7700 device handle initialization failed", veml7700_config->i2c_address);

    /* validate memory availability for handle */
    veml7700_device_t* dev = (veml7700_device_t*)calloc(1, sizeof(veml7700_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for device, veml7700 device handle initialization failed");

    /* copy configuration */
    dev->config = *veml7700_config;

    /* set device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c0 new bus failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));

    /* attempt to read initialization registers */
    veml7700_configuration_register_t       cfg_reg;
    veml7700_power_saving_mode_register_t   psm_reg;
    
    /* attempt to read configuration register */
    ESP_GOTO_ON_ERROR(veml7700_get_configuration_register((veml7700_handle_t)dev, &cfg_reg), err_handle, TAG, "read configuration register failed");

    /* attempt to read power saving mode register */
    ESP_GOTO_ON_ERROR(veml7700_get_power_saving_mode_register((veml7700_handle_t)dev, &psm_reg), err_handle, TAG, "read power saving mode register failed");

    /* set configuration register */
    cfg_reg.bits.gain                   = dev->config.gain;
    cfg_reg.bits.integration_time       = dev->config.integration_time;
    cfg_reg.bits.persistence_protect    = dev->config.persistence_protect;
    cfg_reg.bits.irq_enabled            = dev->config.irq_enabled;
    cfg_reg.bits.shutdown               = dev->config.power_disabled;

    /* set power saving register */
    psm_reg.bits.power_saving_enabled   = veml7700_config->power_saving_enabled;
    psm_reg.bits.power_saving_mode      = veml7700_config->power_saving_mode;

    /* validate thresholds configuration */
    if(dev->config.set_thresholds == true) {
        /* attempt to write threshold registers */
        ESP_GOTO_ON_ERROR(veml7700_set_threshold_registers((veml7700_handle_t)dev, dev->config.hi_threshold, dev->config.lo_threshold), err_handle, TAG, "read threshold registers failed");
    }

    /* attempt to write configuration register */
    ESP_GOTO_ON_ERROR(veml7700_set_configuration_register((veml7700_handle_t)dev, cfg_reg), err_handle, TAG, "write configuration register failed");

    /* attempt to write power saving mode register */
    ESP_GOTO_ON_ERROR(veml7700_set_power_saving_mode_register((veml7700_handle_t)dev, psm_reg), err_handle, TAG, "write power saving mode register failed");

    /* set device handle */
    *veml7700_handle = (veml7700_handle_t)dev;

    /* application start delay  */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t veml7700_get_ambient_light_counts(veml7700_handle_t handle, uint16_t *const counts) {
    uint16_t als_counts = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && counts);

    /* handle measurement refresh time */
    if(dev->config.power_saving_enabled) {
        int psm_index = veml7700_get_psm_mode_index(dev->config.power_saving_mode, dev->config.integration_time);
        vTaskDelay(pdMS_TO_TICKS(veml7700_psm_refresh_time_map[psm_index][3]));
    } else {
        int it_index = veml7700_get_it_index(dev->config.integration_time);
        vTaskDelay(pdMS_TO_TICKS(veml7700_integration_time_map[it_index][1]));
    }
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_ALS, &als_counts), TAG, "read ambient light counts failed" );

    /* set output parameter */
    *counts = als_counts;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t veml7700_get_ambient_light(veml7700_handle_t handle, float *const ambient_light) {
    uint16_t als_counts = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && ambient_light);
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_ambient_light_counts(handle, &als_counts), TAG, "read ambient light counts failed" );

    /* apply resolution correction */
    float comp_lux = (float)(als_counts) * veml7700_get_resolution(dev);

    /* apply correction formula for illumination > 1000 lux */
    if(comp_lux > 1000) {
        /* polynomial correction and set output parameter */
        *ambient_light = (VEML7700_POLY_COEF_A * powf(comp_lux, 4)) + (VEML7700_POLY_COEF_B * powf(comp_lux, 3)) + (VEML7700_POLY_COEF_C * powf(comp_lux, 2)) + (VEML7700_POLY_COEF_D * comp_lux);
    } else {
        /* set output parameter */
        *ambient_light = comp_lux;
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t veml7700_get_ambient_light_auto(veml7700_handle_t handle, float *const ambient_light) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && ambient_light);

    // calculate and automatically reconfigure the optimal sensor configuration
    ESP_RETURN_ON_ERROR( veml7700_optimize_configuration(handle), TAG, "optimize configuration for read ambient light auto failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_ambient_light(handle, ambient_light), TAG, "read ambient light auto failed" );

	//ESP_LOGD(TAG, "Configured maximum luminosity: %" PRIu32 "", veml7700_handle->maximum_lux);
	//ESP_LOGD(TAG, "Configured resolution: %0.4f", veml7700_handle->resolution);

	return ESP_OK;
}

esp_err_t veml7700_get_white_channel_counts(veml7700_handle_t handle, uint16_t *const counts) {
    uint16_t als_counts = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && counts);

    /* handle measurement refresh time */
    if(dev->config.power_saving_enabled) {
        int psm_index = veml7700_get_psm_mode_index(dev->config.power_saving_mode, dev->config.integration_time);
        vTaskDelay(pdMS_TO_TICKS(veml7700_psm_refresh_time_map[psm_index][3]));
    } else {
        int it_index = veml7700_get_it_index(dev->config.integration_time);
        vTaskDelay(pdMS_TO_TICKS(veml7700_integration_time_map[it_index][1]));
    }
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_i2c_read_word_from(dev, VEML7700_CMD_WHITE, &als_counts), TAG, "read white channel counts failed" );

    /* set output parameter */
    *counts = als_counts;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t veml7700_get_white_channel(veml7700_handle_t handle, float *const white_light) {
    uint16_t als_counts = 0;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && white_light);
    
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_white_channel_counts(handle, &als_counts), TAG, "read white channel failed" );

    /* apply resolution correction */
    float comp_lux = (float)(als_counts) * veml7700_get_resolution(dev);

    /* apply correction formula for illumination > 1000 lux */
    if(comp_lux > 1000) {
        /* polynomial correction and set output parameter */
        *white_light = (VEML7700_POLY_COEF_A * powf(comp_lux, 4)) + (VEML7700_POLY_COEF_B * powf(comp_lux, 3)) + (VEML7700_POLY_COEF_C * powf(comp_lux, 2)) + (VEML7700_POLY_COEF_D * comp_lux);
    } else {
        /* set output parameter */
        *white_light = comp_lux;
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t veml7700_get_white_channel_auto(veml7700_handle_t handle, float *const white_light) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && white_light);

    // calculate and automatically reconfigure the optimal sensor configuration
    ESP_RETURN_ON_ERROR( veml7700_optimize_configuration(handle), TAG, "optimize configuration for read white channel auto failed" );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_white_channel(handle, white_light), TAG, "read white channel auto failed" );

	//ESP_LOGD(TAG, "Configured maximum luminocity: %" PRIu32 "\n", veml7700_handle->maximum_lux);
	//ESP_LOGD(TAG, "Configured resolution: %0.4f\n", veml7700_handle->resolution);

	return ESP_OK;
}

esp_err_t veml7700_get_thresholds(veml7700_handle_t handle, uint16_t *const hi_threshold, uint16_t *const lo_threshold) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_threshold_registers(handle, hi_threshold, lo_threshold), TAG, "read threshold registers for get thresholds failed" );

    return ESP_OK;
}

esp_err_t veml7700_set_thresholds(veml7700_handle_t handle, const uint16_t hi_threshold, const uint16_t lo_threshold) {
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_threshold_registers(handle, hi_threshold, lo_threshold), TAG, "write threshold registers for set thresholds failed" );

    /* set config parameters */
    dev->config.hi_threshold = hi_threshold;
    dev->config.lo_threshold = lo_threshold;

    return ESP_OK;
}

esp_err_t veml7700_get_gain(veml7700_handle_t handle, veml7700_gains_t *const gain) {
    veml7700_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    *gain = config.bits.gain;

    return ESP_OK;
}

esp_err_t veml7700_set_gain(veml7700_handle_t handle, const veml7700_gains_t gain) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "write configuration register for set gain failed" );

    /* set parameters */
    config.bits.gain = gain;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for set gain failed" );

    /* set config parameter */
    dev->config.gain = gain;


    return ESP_OK;
}

esp_err_t veml7700_get_integration_time(veml7700_handle_t handle, veml7700_integration_times_t *const integration_time) {
    veml7700_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "write configuration register for set gain failed" );

    /* set output parameter */
    *integration_time = config.bits.integration_time;

    return ESP_OK;
}

esp_err_t veml7700_set_integration_time(veml7700_handle_t handle, const veml7700_integration_times_t integration_time) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "write configuration register for set gain failed" );

    /* set parameters */
    config.bits.integration_time = integration_time;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for set integration time failed" );

    /* set config parameter */
    dev->config.integration_time = config.bits.integration_time;

    return ESP_OK;
}

esp_err_t veml7700_get_persistence_protection(veml7700_handle_t handle, veml7700_persistence_protections_t *const persistence_protection) {
    veml7700_configuration_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    /* set output parameters */
    *persistence_protection = config.bits.persistence_protect;

    return ESP_OK;
}

esp_err_t veml7700_set_persistence_protection(veml7700_handle_t handle, const veml7700_persistence_protections_t persistence_protection) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    /* set parameter */
    config.bits.persistence_protect = persistence_protection;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for set persistence protection failed" );

    /* set config parameter */
    dev->config.persistence_protect = persistence_protection;

    return ESP_OK;
}

esp_err_t veml7700_get_power_saving_mode(veml7700_handle_t handle, veml7700_power_saving_modes_t *const power_saving_mode, bool *const power_saving_enabled) {
    veml7700_power_saving_mode_register_t psm;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_power_saving_mode_register(handle, &psm), TAG, "read power saving mode register for get power saving mode failed" );

    /* set output parameters */
    *power_saving_mode    = psm.bits.power_saving_mode;
    *power_saving_enabled = psm.bits.power_saving_enabled;

    return ESP_OK;
}

esp_err_t veml7700_set_power_saving_mode(veml7700_handle_t handle, const veml7700_power_saving_modes_t power_saving_mode, const bool power_saving_enabled) {
    veml7700_power_saving_mode_register_t psm;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_power_saving_mode_register(handle, &psm), TAG, "read power saving mode register for get power saving mode failed" );

    /* set parameters */
    psm.bits.power_saving_mode    = power_saving_mode;
    psm.bits.power_saving_enabled = power_saving_enabled;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_power_saving_mode_register(handle, psm), TAG, "write power saving mode register for set power saving mode failed" );

    /* set config parameter */
    dev->config.power_saving_mode    = power_saving_mode;
    dev->config.power_saving_enabled = power_saving_enabled;

    return ESP_OK;
}

esp_err_t veml7700_get_interrupt_status(veml7700_handle_t handle, bool *const hi_threshold_exceeded, bool *const lo_threshold_exceeded) {
    veml7700_interrupt_status_register_t irq;
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_interrupt_status_register(handle, &irq), TAG, "read interrupt status register for interrupt status failed" );

    /* set output parameters */
    *hi_threshold_exceeded = irq.bits.hi_threshold_exceeded;
    *lo_threshold_exceeded = irq.bits.lo_threshold_exceeded;

    return ESP_OK;
}

esp_err_t veml7700_enable_irq(veml7700_handle_t handle) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    /* set parameters */
    config.bits.irq_enabled = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for enable irq failed" );

    /* set config parameter */
    dev->config.irq_enabled = true;

    return ESP_OK;
}

esp_err_t veml7700_disable_irq(veml7700_handle_t handle) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    /* set parameters */
    config.bits.irq_enabled = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for disable irq failed" );

    /* set config parameter */
    dev->config.irq_enabled = false;

    return ESP_OK;
}

esp_err_t veml7700_disable(veml7700_handle_t handle) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    /* shutdown device */
    config.bits.shutdown = true;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for shutdown failed" );

    /* set config parameter */
    dev->config.power_disabled = true;

    return ESP_OK;
}

esp_err_t veml7700_enable(veml7700_handle_t handle) {
    veml7700_configuration_register_t config;
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( veml7700_get_configuration_register(handle, &config), TAG, "read configuration register for get gain failed" );

    /* wakeup device */
    config.bits.shutdown = false;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( veml7700_set_configuration_register(handle, config), TAG, "write configuration register for wake-up failed" );

    /* set config parameter */
    dev->config.power_disabled = false;

    return ESP_OK;
}

esp_err_t veml7700_remove(veml7700_handle_t handle) {
    veml7700_device_t* dev = (veml7700_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t veml7700_delete(veml7700_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( veml7700_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* veml7700_get_fw_version(void) {
    return (const char*)VEML7700_FW_VERSION_STR;
}

int32_t veml7700_get_fw_version_number(void) {
    return (int32_t)VEML7700_FW_VERSION_INT32;
}