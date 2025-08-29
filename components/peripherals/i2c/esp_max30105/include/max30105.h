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

#define I2C_MAX30105_DEV_ADDR              UINT8_C(0x38) //!< max30105 I2C address


/*
 * MAX30105 macro definitions
 */
#define I2C_MAX30105_CONFIG_DEFAULT {                   \
    .i2c_address            = I2C_MAX30105_DEV_ADDR,    \
    .i2c_clock_speed        = I2C_MAX30105_DEV_CLK_SPD, \
    }


/*
* MAX30105 enumerator and structure declarations
*/

/**
 * @brief MAX30105 particle-sensing ADC range controls (18-bit resolution) enumerator (register 0x0a).
 */
typedef enum max30105_adc_range_controls_e {
    MAX30105_ARC_7_81LSB  = (0x00), /*!< max30105 7.81 LSB size (pA), 2048 full-scale (nA) */
    MAX30105_ARC_15_63LSB = (0x01), /*!< max30105 15.63 LSB size (pA), 4096 full-scale (nA) */
    MAX30105_ARC_31_25LSB = (0x02), /*!< max30105 31.25 LSB size (pA), 8192 full-scale (nA) */  
    MAX30105_ARC_62_5LSB  = (0x03), /*!< max30105 62.5 LSB size (pA), 16384 full-scale (nA) */
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
    MAX30105_CM_PARTICLE_SENSING_1LED = (0b010), /*!< max30105 particle sensing mode with 1 LED */
    MAX30105_CM_PARTICLE_SENSING_2LED = (0b011), /*!< max30105 particle sensing mode with 2 LEDs */
    MAX30105_CM_MULTIPLE_LED          = (0b111), /*!< max30105 multiple LED mode */  
} max30105_control_modes_t;

/**
 * @brief MAX30105 multi-LED control modes enumerator (registers 0x11-0x12).
 */
typedef enum max30105_multi_led_control_modes_e {
    MAX30105_MLCM_DISABLED    = (0b000), /*!< max30105  */
    MAX30105_MLCM_RED_LED     = (0b001), /*!< max30105  */
    MAX30105_MLCM_IR_LED      = (0b010), /*!< max30105  */  
    MAX30105_MLCM_GREEN_LED   = (0b011), /*!< max30105  */ 
    MAX30105_MLCM_NONE        = (0b100), /*!< max30105  */
    MAX30105_MLCM_RED_PILOT   = (0b101), /*!< max30105  */
    MAX30105_MLCM_IR_PILOT    = (0b110), /*!< max30105  */  
    MAX30105_MLCM_GREEN_PILOT = (0b111), /*!< max30105  */ 
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
 * @brief MAX30105 interrupt status 1 register (0x00, read-only | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_interrupt_status1_register_u {
    struct {
        bool    irq_power_ready:1;   /*!< max30105 On power-up or after a brownout condition, when the supply voltage VDD transitions from below the undervoltage-lockout (UVLO) voltage to above the UVLO voltage, a power-ready interrupt is triggered to signal that the module is powered-up and ready to collect data.                       (bit:0)  */
        uint8_t reserved:3;          /*!< reserved                       (bit:1-3) */
        bool    irq_proximity:1;     /*!< max30105 the proximity interrupt is triggered when the proximity threshold is reached, and particle-sensing mode has begun. This lets the host processor know to begin running the particle-sensing algorithm and collect data. The interrupt is cleared by reading the Interrupt Status 1 register (0x00).    (bit:4) */
        bool    irq_alc_overflow:1;  /*!< max30105 this interrupt triggers when the ambient light cancellation function of the particle-sensing photodiode has reached its maximum limit, The interrupt is cleared by reading the Interrupt Status 1 register (0x00).     (bit:5) */
        bool    irq_data_ready:1;    /*!< max30105 in particle-sensing mode, this interrupt triggers when there is a new sample in the data FIFO. The interrupt is cleared by reading the Interrupt Status 1 register (0x00), or by reading the FIFO_DATA register. (bit:6) */
        bool    irq_fifo_almost_full:1; /*!< max30105 in particle-sensing mode, this interrupt triggers when the FIFO write pointer has a certain number of free spaces remaining. The interrupt is cleared by reading the Interrupt Status 1 register (0x00).  (bit:7) */
    } bits;
    uint8_t reg;
} max30105_interrupt_status1_register_t;

/**
 * @brief MAX30105 interrupt status 2 register (0x01, read-only | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_interrupt_status2_register_u {
    struct {
        uint8_t reserved1:1;          /*!< reserved                       (bit:0) */
        bool    irq_die_temperature_ready:1; /*!<  max30105 when an internal die temperature conversion is finished, this interrupt is triggered so the processor can read the temperature data registers. The interrupt is cleared by reading either the Interrupt Status 2 register (0x01) or the TFRAC register (0x20).          (bit:1)  */
        uint8_t reserved2:6;          /*!< reserved                       (bit:2-7) */
    } bits;
    uint8_t reg;
} max30105_interrupt_status2_register_t;

/**
 * @brief MAX30105 interrupt enable 1 register (0x02, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_interrupt_enable1_register_u {
    struct {
        uint8_t reserved:4;          /*!< reserved    (bit:0-3) */
        bool    proximity_irq_enabled:1;     /*!< max30105 proximity interrupt is asserted when enabled (bit:4) */
        bool    alc_overflow_irq_enabled:1;  /*!< max30105 ambient light cancellation interrupt is asserted when enabled (bit:5) */
        bool    data_ready_irq_enabled:1;    /*!< max30105 data ready interrupt is asserted when enabled (bit:6) */
        bool    fifo_almost_full_irq_enabled:1; /*!< max30105   interrupt is asserted when enabled (bit:7) */
    } bits;
    uint8_t reg;
} max30105_interrupt_enable1_register_t;

/**
 * @brief MAX30105 interrupt enable 2 register (0x03, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_interrupt_enable2_register_u {
    struct {
        uint8_t reserved1:4;          /*!< reserved    (bit:0) */
        bool    irq_die_temperature_ready_enabled:1;  /*!< max30105 internal temperature ready interrupt is asserted when enabled  (bit:1) */
        uint8_t reserved2:4;          /*!< reserved    (bit:2-7) */
    } bits;
    uint8_t reg;
} max30105_interrupt_enable2_register_t;

/**
 * @brief MAX30105 mode configuration register (0x09, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_mode_configuration_register_u {
    struct {
        max30105_control_modes_t control_mode:3;     /*!< max30105 These bits set the operating state of the MAX30105. Changing modes does not change any other setting, nor does it erase any previously stored data inside the data registers. (bit:0-2) */
        uint8_t                  reserved:3;  /*!< reserved (bit:5-3) */
        bool                     reset_enabled:1;    /*!< max30105 soft-reset is asserted when enabled, When the RESET bit is set to one, all configuration, threshold, and data registers are reset to their power-on-state through a power-on reset. (bit:6) */
        bool                     shutdown_enabled:1; /*!< max30105 shutdown is asserted when enabled, The part can be put into a power-save mode by setting this bit to one. While in power-save mode, all registers retain their values, and write/read operations function as normal. All interrupts are cleared to zero in this mode. (bit:7) */
    } bits;
    uint8_t reg;
} max30105_mode_configuration_register_t;

/**
 * @brief MAX30105 multi-LED mode control registers (0x11-0x12, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_multi_led_mode_control_register_u {
    struct {
        max30105_multi_led_control_modes_t slot_1:3;    /*!< max30105 multi-LED mode time slot 1 or 3 (bit:0-2) */
        uint8_t                            reserved1:1; /*!< reserved (bit:3) */
        max30105_multi_led_control_modes_t slot_2:3;    /*!< max30105 multi-LED mode time slot 2 or 4 (bit:4-6) */
        uint8_t                            reserved2:1; /*!< reserved (bit:7) */
    } bits;
    uint8_t reg;
} max30105_multi_led_mode_control_register_t;

/**
 * @brief MAX30105 FIFO write pointer register (0x04, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_fifo_write_pointer_register_u {
    struct {
        uint8_t fifo_write_pointer:5;  /*!< max30105   (bit:0-4) */
        uint8_t reserved:3;            /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} max30105_fifo_write_pointer_register_t;

/**
 * @brief MAX30105 FIFO overflow counter register (0x05, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_fifo_overflow_counter_register_u {
    struct {
        uint8_t fifo_overflow_counter:5;  /*!< max30105   (bit:0-4) */
        uint8_t reserved:3;            /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} max30105_fifo_overflow_counter_register_t;

/**
 * @brief MAX30105 FIFO read pointer register (0x06, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_fifo_read_pointer_register_u {
    struct {
        uint8_t fifo_read_pointer:5;  /*!< max30105   (bit:0-4) */
        uint8_t reserved:3;            /*!< reserved   (bit:5-7) */
    } bits;
    uint8_t reg;
} max30105_fifo_read_pointer_register_t;

/**
 * @brief MAX30105 FIFO data register (0x07, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_fifo_data_register_u {
    struct {
        uint8_t fifo_data:5;  /*!< max30105   (bit:0-7) */
    } bits;
    uint8_t reg;
} max30105_fifo_data_register_t;

/**
 * @brief MAX30105 configuration structure.
 */
typedef struct max30105_config_s {
    uint16_t                    i2c_address;    /*!< max30105 i2c device address */
    uint32_t                    i2c_clock_speed;/*!< max30105 i2c device scl clock speed in hz */
    max30105_control_modes_t    control_mode;   /*!< max30105 LED control mode */
} max30105_config_t;

/**
 * @brief MAX30105 context structure.
 */
struct max30105_context_t {
    max30105_config_t                      dev_config;  /*!< max30105 device configuration */
    i2c_master_dev_handle_t                i2c_handle;  /*!< max30105 I2C device handle */
   // max30105_mode_configuration_register_t mode_config_reg; /*!< max30105 mode configuration register (read-write) */
    //max30105_interrupt_status1_register_t  irq_status1_reg; /*!< max30105 interrupt status 1 register (read-only) */
    //max30105_interrupt_status2_register_t  irq_status2_reg; /*!< max30105 interrupt status 2 register (read-only) */
   //max30105_interrupt_enable1_register_t  irq_enable1_reg; /*!< max30105 interrupt enable 1 register (read-write) */
    //max30105_interrupt_enable2_register_t  irq_enable2_reg; /*!< max30105 interrupt enable 2 register (read-write) */
};

/**
 * @brief MAX30105 context structure definition.
 */
typedef struct max30105_context_t max30105_context_t;

/**
 * @brief MAX30105 handle structure definition.
 */
typedef struct max30105_context_t *max30105_handle_t;



/**
 * @brief Reads interrupt status 1 register from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_interrupt_status1_register(max30105_handle_t handle, max30105_interrupt_status1_register_t *const reg);

/**
 * @brief Reads interrupt status 2 register from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_interrupt_status2_register(max30105_handle_t handle, max30105_interrupt_status2_register_t *const reg);

/**
 * @brief Reads interrupt enable 1 register from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_interrupt_enable1_register(max30105_handle_t handle, max30105_interrupt_enable1_register_t *const reg);

/**
 * @brief Writes interrupt enable 1 register to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param reg Interrupt enable 1 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_interrupt_enable1_register(max30105_handle_t handle, const max30105_interrupt_enable1_register_t reg);

/**
 * @brief Reads interrupt enable 2 register from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_interrupt_enable2_register(max30105_handle_t handle, max30105_interrupt_enable2_register_t *const reg);

/**
 * @brief Writes interrupt enable 2 register to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param reg Interrupt enable 2 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_interrupt_enable2_register(max30105_handle_t handle, const max30105_interrupt_enable2_register_t reg);

/**
 * @brief Reads mode configuration register from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_mode_configuration_register(max30105_handle_t handle, max30105_mode_configuration_register_t *const reg);

/**
 * @brief Writes mode configuration register to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param reg Mode configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_mode_configuration_register(max30105_handle_t handle, const max30105_mode_configuration_register_t reg);

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
 * @param red_count Red LED ADC count.
 * @param ir_count IR LED ADC count.
 * @param green_count Green LED ADC count.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_optical_counts(max30105_handle_t handle, float *const red_count, float *const ir_count, float *const green_count);

/**
 * @brief Reads data status from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @param[out] ready MAX30105 data is ready when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_data_status(max30105_handle_t handle, bool *const ready);

/**
 * @brief Reads control mode setting from MAX30105.
 *
 * @param handle MAX30105 device handle.
 * @param control_mode MAX30105 control mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_get_control_mode(max30105_handle_t handle, max30105_control_modes_t *const control_mode);

/**
 * @brief Writes control mode setting to MAX30105.
 * 
 * @param handle MAX30105 device handle.
 * @param control_mode MAX30105 control mode setting.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t max30105_set_control_mode(max30105_handle_t handle, const max30105_control_modes_t control_mode);

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
