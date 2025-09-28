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
 * @file max30105.h
 * @defgroup drivers max30105
 * @{
 *
 * ESP-IDF driver for max30105 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MAX30105_H__
#define __MAX30105_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*
 * MAX30105 definitions
 */
#define I2C_MAX30105_DEV_CLK_SPD           UINT32_C(100000) //!< max30105 I2C default clock frequency (100KHz)

#define I2C_MAX30105_DEV_ADDR              UINT8_C(0x57) //!< max30105 I2C address

#define MAX30105_CH_SMP_MAX                UINT8_C(32) //!< max30105 maximum number of samples by channel (i.e. red, ir, green)

/*
 * MAX30105 macro definitions
 */
#define MAX30105_CONFIG_DEFAULT {                                  \
    .i2c_address                   = I2C_MAX30105_DEV_ADDR,        \
    .i2c_clock_speed               = I2C_MAX30105_DEV_CLK_SPD,     \
    .fifo_almost_full_threshold    = 0x0,                          \
    .fifo_rollover_enabled         = true,                         \
    .fifo_sample_average           = MAX30105_SMP_AVG_8,           \
    .control_mode                  = MAX30105_CM_GREEN_RED_IR_LED, \
    .particle_sample_rate          = MAX30105_SRC_800SPS,          \
    .particle_adc_resolution       = MAX30105_ARC_7_81LSB_2048FS,  \
    .particle_led_pulse_width      = MAX30105_LPWC_411US_18BITS,   \
    .multi_led_mode_slot1          = MAX30105_MLCM_RED_LED1_PA,    \
    .multi_led_mode_slot2          = MAX30105_MLCM_IR_LED2_PA,     \
    .multi_led_mode_slot3          = MAX30105_MLCM_GREEN_LED3_PA,  \
    .multi_led_mode_slot4          = MAX30105_MLCM_NONE,           \
    .red_led_pulse_amplitude       = MAX30105_LPA_6_4MA,           \
    .ir_led_pulse_amplitude        = MAX30105_LPA_6_4MA,           \
    .green_led_pulse_amplitude     = MAX30105_LPA_6_4MA,           \
    .proximity_led_pulse_amplitude = MAX30105_LPA_6_4MA,           \
    }

#define MAX30105_PARTICLE_CONFIG_DEFAULT {                              \
    .i2c_address                   = I2C_MAX30105_DEV_ADDR,             \
    .i2c_clock_speed               = I2C_MAX30105_DEV_CLK_SPD,          \
    .fifo_almost_full_threshold    = 0x0,                               \
    .fifo_rollover_enabled         = true,                              \
    .fifo_sample_average           = MAX30105_SMP_AVG_8,                \
    .control_mode                  = MAX30105_CM_RED_IR_LED,            \
    .particle_sample_rate          = MAX30105_SRC_800SPS,               \
    .particle_adc_resolution       = MAX30105_ARC_7_81LSB_2048FS,       \
    .particle_led_pulse_width      = MAX30105_LPWC_411US_18BITS,        \
    .multi_led_mode_slot1          = MAX30105_MLCM_RED_LED1_PA,         \
    .multi_led_mode_slot2          = MAX30105_MLCM_IR_LED2_PA,          \
    .multi_led_mode_slot3          = MAX30105_MLCM_NONE,                \
    .multi_led_mode_slot4          = MAX30105_MLCM_NONE,                \
    .red_led_pulse_amplitude       = MAX30105_LPA_6_4MA,                \
    .ir_led_pulse_amplitude        = MAX30105_LPA_6_4MA,                \
    .green_led_pulse_amplitude     = MAX30105_LPA_6_4MA,                \
    .proximity_led_pulse_amplitude = MAX30105_LPA_6_4MA,                \
    }


/*
* MAX30105 enumerator and structure declarations
*/

/**
 * @brief MAX30105 particle-sensing ADC range controls enumerator (register 0x0a).
 */
typedef enum max30105_adc_range_controls_e {
    MAX30105_ARC_7_81LSB_2048FS  = (0x00), /*!< max30105 7.81 LSB size (pA), 2048 full-scale (nA) */
    MAX30105_ARC_15_63LSB_4096FS = (0x01), /*!< max30105 15.63 LSB size (pA), 4096 full-scale (nA) */
    MAX30105_ARC_31_25LSB_8192FS = (0x02), /*!< max30105 31.25 LSB size (pA), 8192 full-scale (nA) */  
    MAX30105_ARC_62_5LSB_16384FS = (0x03), /*!< max30105 62.5 LSB size (pA), 16384 full-scale (nA) */
} max30105_adc_range_controls_t;

/**
 * @brief MAX30105 particle-sensing sample rate control enumerator (register 0x0a).
 */
typedef enum max30105_sample_rate_controls_e {
    MAX30105_SRC_50SPS   = (0b000), /*!< max30105 50 samples per second */
    MAX30105_SRC_100SPS  = (0b001), /*!< max30105 100 samples per second */
    MAX30105_SRC_200SPS  = (0b010), /*!< max30105 200 samples per second */  
    MAX30105_SRC_400SPS  = (0b011), /*!< max30105 400 samples per second */ 
    MAX30105_SRC_800SPS  = (0b100), /*!< max30105 800 samples per second */ 
    MAX30105_SRC_1000SPS = (0b101), /*!< max30105 1000 samples per second */ 
    MAX30105_SRC_1600SPS = (0b110), /*!< max30105 1600 samples per second */ 
    MAX30105_SRC_3200SPS = (0b111), /*!< max30105 3200 samples per second */ 
} max30105_sample_rate_controls_t;

/**
 * @brief MAX30105 LED pulse width controls enumerator (register 0x0a).
 */
typedef enum max30105_led_pulse_width_controls_e {
    MAX30105_LPWC_69US_15BITS  = (0b00), /*!< max30105 68.95us pulse width with 15-bit ADC resolution */
    MAX30105_LPWC_118US_16BITS = (0b01), /*!< max30105 117.78us pulse width with 16-bit ADC resolution */
    MAX30105_LPWC_215US_17BITS = (0b10), /*!< max30105 215.44us pulse width with 17-bit ADC resolution */  
    MAX30105_LPWC_411US_18BITS = (0b11), /*!< max30105 410.75us pulse width with 18-bit ADC resolution */  
} max30105_led_pulse_width_controls_t;

/**
 * @brief MAX30105 control modes enumerator.
 */
typedef enum max30105_control_modes_e {
    MAX30105_CM_RED_LED          = (0b010), /*!< max30105 particle sensing mode with 1 LED, red only */
    MAX30105_CM_RED_IR_LED       = (0b011), /*!< max30105 particle sensing mode with 2 LEDs, red and IR */
    MAX30105_CM_GREEN_RED_IR_LED = (0b111), /*!< max30105 multiple LED mode, green, red, and/or IR */  
} max30105_control_modes_t;

/**
 * @brief MAX30105 multi-LED control modes enumerator (registers 0x11-0x12).
 */
typedef enum max30105_multi_led_control_modes_e {
    MAX30105_MLCM_DISABLED            = (0b000), /*!< max30105 none, time slot is disabled */
    MAX30105_MLCM_RED_LED1_PA         = (0b001), /*!< max30105 red LED1 pulse amplitude */
    MAX30105_MLCM_IR_LED2_PA          = (0b010), /*!< max30105 infrared LED2 pulse amplitude */  
    MAX30105_MLCM_GREEN_LED3_PA       = (0b011), /*!< max30105 green LED3 pulse amplitude */ 
    MAX30105_MLCM_NONE                = (0b100), /*!< max30105 none */
    MAX30105_MLCM_RED_LED1_PILOT_PA   = (0b101), /*!< max30105 red LED1 proximity mode pulse amplitude */
    MAX30105_MLCM_IR_LED2_PILOT_PA    = (0b110), /*!< max30105 infrared LED2 proximity mode pulse amplitude  */  
    MAX30105_MLCM_GREEN_LED3_PILOT_PA = (0b111), /*!< max30105 green LED3 proximity mode pulse amplitude */ 
} max30105_multi_led_control_modes_t;

/**
 * @brief MAX30105 LED pulse amplitudes enumerator (registers 0x0c-0x10).
 */
typedef enum max30105_led_pulse_amplitudes_e {
    MAX30105_LPA_0_0MA  = (0x00), /*!< max30105 LED pulse amplitude at 0.0 mA */
    MAX30105_LPA_0_2MA  = (0x01), /*!< max30105 LED pulse amplitude at 0.2 mA */
    MAX30105_LPA_0_4MA  = (0x02), /*!< max30105 LED pulse amplitude at 0.4 mA */ 
    MAX30105_LPA_3_1MA  = (0x0f), /*!< max30105 LED pulse amplitude at 3.1 mA */
    MAX30105_LPA_6_4MA  = (0x1f), /*!< max30105 LED pulse amplitude at 6.4 mA */ 
    MAX30105_LPA_12_5MA = (0x3f), /*!< max30105 LED pulse amplitude at 12.5 mA */ 
    MAX30105_LPA_25_4MA = (0x7f), /*!< max30105 LED pulse amplitude at 25.4 mA */ 
    MAX30105_LPA_50_0MA = (0xff), /*!< max30105 LED pulse amplitude at 50.0 mA */ 
} max30105_led_pulse_amplitudes_t;

/**
 * @brief MAX30105 FIFO sample averaging enumerator (register 0x08).
 */
typedef enum max30105_sample_averages_e {
    MAX30105_SMP_AVG_1    = (0b000),    /*!< max30105 sample averaging of 1 */
    MAX30105_SMP_AVG_2    = (0b001),    /*!< max30105 sample averaging of 2 */
    MAX30105_SMP_AVG_4    = (0b010),    /*!< max30105 sample averaging of 4 */
    MAX30105_SMP_AVG_8    = (0b011),    /*!< max30105 sample averaging of 8 */
    MAX30105_SMP_AVG_16   = (0b100),    /*!< max30105 sample averaging of 16 */
    MAX30105_SMP_AVG_32   = (0b101)     /*!< max30105 sample averaging of 32 */
} max30105_sample_averages_t;

/**
 * @brief MAX30105 pulse amplitude LED types enumerator.
 */
typedef enum max30105_pulse_amplitude_led_types_e {
    MAX30105_PALT_RED_LED1   = 0,
    MAX30105_PALT_IR_LED2    = 1,
    MAX30105_PALT_GREEN_LED3 = 2,
    MAX30105_PALT_PROX_LED   = 3
} max30105_pulse_amplitude_led_types_t;

/**
 * @brief MAX30105 multi-LED control mode slots enumerator.
 */
typedef enum max30105_multi_led_mode_slots_e {
    MAX30105_MLM_SLOT_1 = 0,
    MAX30105_MLM_SLOT_2 = 1,
    MAX30105_MLM_SLOT_3 = 2,
    MAX30105_MLM_SLOT_4 = 3
} max30105_multi_led_mode_slots_t;

/**
 * @brief MAX30105 ADC channels (Red, IR, Green) count data structure definition.
 */
typedef struct max30105_adc_channels_count_data_s {
    uint32_t red_count[MAX30105_CH_SMP_MAX];
    uint32_t ir_count[MAX30105_CH_SMP_MAX];
    uint32_t green_count[MAX30105_CH_SMP_MAX];
    uint8_t  sample_size;
} max30105_adc_channels_count_data_t;

/**
 * @brief MAX30105 interrupt status flags structure definition.
 */
typedef struct max30105_irq_status_flags_s {
    bool    power_ready;        /*!< max30105 triggers the power-ready interrupt to signal that the module is powered-up and ready to collect data */
    bool    proximity;          /*!< max30105 triggers the proximity interrupt when the proximity threshold is reached, and particle-sensing mode has begun. */
    bool    alc_overflow;       /*!< max30105 triggers the alc overflow interrupt when the ambient light cancellation function of the particle-sensing photo-diode has reached its maximum limit, and ambient light is affecting the output o the ADC. */
    bool    data_ready;         /*!< max30105 triggers the data-ready interrupt when there is a new sample in the data FIFO. */
    bool    fifo_almost_full;   /*!< max30105 triggers the FIFO almost full interrupt when the FIFO write pointer has a certain number of free spaces remaining. */
    bool    temperature_ready;  /*!< max30105 triggers the temperature-ready interrupt when an internal die temperature conversion is finished. */
} max30105_irq_status_flags_t;

/**
 * @brief MAX30105 configuration structure definition.
 */
typedef struct max30105_config_s {
    uint16_t                            i2c_address;                    /*!< max30105 i2c device address */
    uint32_t                            i2c_clock_speed;                /*!< max30105 i2c device scl clock speed in hz */
    max30105_control_modes_t            control_mode;                   /*!< max30105 LED control mode */
    max30105_adc_range_controls_t       particle_adc_resolution;        /*!< max30105 particle-sensing ADC range or resolution */
    max30105_sample_rate_controls_t     particle_sample_rate;           /*!< max30105 particle-sensing sample rate */
    max30105_led_pulse_width_controls_t particle_led_pulse_width;       /*!< max30105 particle-sensing LED pulse width and ADC resolution */
    max30105_sample_averages_t          fifo_sample_average;            /*!< max30105 FIFO sample averaging */
    bool                                fifo_rollover_enabled;          /*!< max30105 FIFO rollover enabled */
    uint8_t                             fifo_almost_full_threshold;     /*!< max30105 FIFO almost full interrupt threshold (number of samples: 0x0h = 32, 0x1h = 31, 0xFh = 17) */
    max30105_led_pulse_amplitudes_t     red_led_pulse_amplitude;        /*!< max30105 red LED pulse amplitude */
    max30105_led_pulse_amplitudes_t     ir_led_pulse_amplitude;         /*!< max30105 infrared LED pulse amplitude */
    max30105_led_pulse_amplitudes_t     green_led_pulse_amplitude;      /*!< max30105 green LED pulse amplitude */
    max30105_led_pulse_amplitudes_t     proximity_led_pulse_amplitude;  /*!< max30105 proximity LED pulse amplitude */
    max30105_multi_led_control_modes_t  multi_led_mode_slot1;           /*!< max30105 multi-LED mode time slot 1 */
    max30105_multi_led_control_modes_t  multi_led_mode_slot2;           /*!< max30105 multi-LED mode time slot 2 */
    max30105_multi_led_control_modes_t  multi_led_mode_slot3;           /*!< max30105 multi-LED mode time slot 3 */
    max30105_multi_led_control_modes_t  multi_led_mode_slot4;           /*!< max30105 multi-LED mode time slot 4 */
} max30105_config_t;

/**
 * @brief MAX30105 opaque handle structure definition.
 */
typedef void* max30105_handle_t;


/**
 * @brief Initializes an MAX30105 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] max30105_config MAX30105 device configuration.
 * @param[out] max30105_handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_init(i2c_master_bus_handle_t master_handle, const max30105_config_t *max30105_config, max30105_handle_t *max30105_handle);

/**
 * @brief Reads red, IR, and green LED ADC counts from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @param data Data structure that contains Red LED ADC, IR LED, and Green LED counts.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_optical_counts(max30105_handle_t handle, max30105_adc_channels_count_data_t *const data);

/**
 * @brief Reads FIFO data status from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @param[out] ready MAX30105 FIFO data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_data_status(max30105_handle_t handle, bool *const ready);

/**
 * @brief Reads temperature data status from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @param[out] ready MAX30105 temperature data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_temperature_status(max30105_handle_t handle, bool *const ready);

/**
 * @brief Reads interrupt status from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param flags MAX30105 status flags data structure .
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_interrupt_status(max30105_handle_t handle, max30105_irq_status_flags_t *const flags);

/**
 * @brief Reads control mode setting from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @param mode MAX30105 control mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_control_mode(max30105_handle_t handle, max30105_control_modes_t *const mode);

/**
 * @brief Writes control mode setting to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param mode MAX30105 control mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_control_mode(max30105_handle_t handle, const max30105_control_modes_t mode);

/**
 * @brief Reads LED pulse amplitude by type (Red, IR, Green, or Proximity) from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param type MAX30105 pulse amplitude LED type (Red, IR, Green, or Proximity).
 * @param amplitude MAX30105 LED pulse amplitude setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_led_pulse_amplitude(max30105_handle_t handle, const max30105_pulse_amplitude_led_types_t type, max30105_led_pulse_amplitudes_t *const amplitude);

/**
 * @brief Writes LED pulse amplitude by type (Red, IR, Green, or Proximity) to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param type MAX30105 pulse amplitude LED type (Red, IR, Green, or Proximity).
 * @param amplitude MAX30105 LED pulse amplitude setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_led_pulse_amplitude(max30105_handle_t handle, const max30105_pulse_amplitude_led_types_t type, const max30105_led_pulse_amplitudes_t amplitude);

/**
 * @brief Reads multi-LED control mode by slot (1 to 4) from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param slot MAX30105 multi-LED control mode slot (1 to 4).
 * @param mode MAX30105 multi-LED control mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_multi_led_mode(max30105_handle_t handle, const max30105_multi_led_mode_slots_t slot, max30105_multi_led_control_modes_t *const mode);

/**
 * @brief Writes multi-LED control mode by slot (1 to 4) to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param slot MAX30105 multi-LED control mode slot (1 to 4).
 * @param mode MAX30105 multi-LED control mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_multi_led_mode(max30105_handle_t handle, const max30105_multi_led_mode_slots_t slot, const max30105_multi_led_control_modes_t mode);

/**
 * @brief Reads particle-sensing LED pulse width from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param pulse_width MAX30105 LED pulse width setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_particle_led_pulse_width(max30105_handle_t handle, max30105_led_pulse_width_controls_t *const pulse_width);

/**
 * @brief Writes particle-sensing LED pulse width to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param pulse_width MAX30105 LED pulse width setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_particle_led_pulse_width(max30105_handle_t handle, const max30105_led_pulse_width_controls_t pulse_width);

/**
 * @brief Reads particle-sensing adc resolution from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param resolution MAX30105 particle adc resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_particle_adc_resolution(max30105_handle_t handle, max30105_adc_range_controls_t *const resolution);

/**
 * @brief Writes particle-sensing adc resolution to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param resolution MAX30105 particle adc resolution setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_particle_adc_resolution(max30105_handle_t handle, const max30105_adc_range_controls_t resolution);

/**
 * @brief Reads particle-sensing sample rate from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param rate MAX30105 particle sample rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_particle_sample_rate(max30105_handle_t handle, max30105_sample_rate_controls_t *const rate);

/**
 * @brief Writes particle-sensing sample rate to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param rate MAX30105 particle sample rate setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_particle_sample_rate(max30105_handle_t handle, const max30105_sample_rate_controls_t rate);

/**
 * @brief Reads FIFO sample averaging from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param average MAX30105 FIFO sample averaging setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_fifo_sample_averaging(max30105_handle_t handle, max30105_sample_averages_t *const average);

/**
 * @brief Writes FIFO sample averaging to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param average MAX30105 FIFO sample averaging setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_fifo_sample_averaging(max30105_handle_t handle, const max30105_sample_averages_t average);

/**
 * @brief Reads FIFO almost full threshold from MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param threshold MAX30105 FIFO almost full threshold setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_fifo_almost_full_threshold(max30105_handle_t handle, uint8_t *const threshold);

/**
 * @brief Writes FIFO almost full threshold to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param threshold MAX30105 FIFO almost full threshold setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_fifo_almost_full_threshold(max30105_handle_t handle, const uint8_t threshold);

/**
 * @brief Enables power on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_power(max30105_handle_t handle);

/**
 * @brief Disables power on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_power(max30105_handle_t handle);

/**
 * @brief Enables FIFO rollover on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_fifo_rollover(max30105_handle_t handle);

/**
 * @brief Disables FIFO rollover on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_fifo_rollover(max30105_handle_t handle);

/**
 * @brief Enables ambient light cancellation overflow interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_alc_overflow_interrupt(max30105_handle_t handle);

/**
 * @brief Disables ambient light cancellation overflow interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_alc_overflow_interrupt(max30105_handle_t handle);

/**
 * @brief Enables data ready interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_data_ready_interrupt(max30105_handle_t handle);

/**
 * @brief Disables data ready interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_data_ready_interrupt(max30105_handle_t handle);

/**
 * @brief Enables FIFO almost full interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_fifo_almost_full_interrupt(max30105_handle_t handle);

/**
 * @brief Disables FIFO almost full interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_fifo_almost_full_interrupt(max30105_handle_t handle);

/**
 * @brief Enables proximity interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_proximity_interrupt(max30105_handle_t handle);

/**
 * @brief Disables proximity interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_proximity_interrupt(max30105_handle_t handle);

/**
 * @brief Enables die temperature sensor interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_enable_temperature_interrupt(max30105_handle_t handle);

/**
 * @brief Disables die temperature sensor interrupt on MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_disable_temperature_interrupt(max30105_handle_t handle);

/**
 * @brief Writes a soft-reset to the MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_reset(max30105_handle_t handle);

/**
 * @brief Removes an MAX30105 device from master bus.
 *
 * @param[in] handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_remove(max30105_handle_t handle);

/**
 * @brief Removes an MAX30105 device from master bus and frees handle.
 * 
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_delete(max30105_handle_t handle);

/**
 * @brief Converts MAX30105 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* MAX30105 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* max30105_get_fw_version(void);

/**
 * @brief Converts MAX30105 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t MAX30105 firmware version number.
 */
int32_t max30105_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __MAX30105_H__
