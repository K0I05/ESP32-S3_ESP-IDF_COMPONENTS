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
 * @file tcs3472.h
 * @defgroup drivers tcs3472
 * @{
 *
 * ESP-IDF driver for OSRAM TCS3472 RGBC sensor
 * 

 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __TCS3472_H__
#define __TCS3472_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "tcs3472_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * TCS3472 definitions
*/
#define I2C_TCS3472_DEV_CLK_SPD            UINT32_C(100000)    //!< vl53l4cx I2C default clock frequency (100KHz)

#define I2C_TCS3472_DEV_ADDR               UINT8_C(0x52)       //!< vl53l4cx I2C address


/*
 * TCS3472 macro definitions
*/

/**
 * @brief TCS3472 device configuration initialization default.
 */
#define I2C_TCS3472_CONFIG_DEFAULT {                                                \
            .i2c_address                = I2C_TCS3472_DEV_ADDR,                     \
            .i2c_clock_speed            = I2C_TCS3472_DEV_CLK_SPD,                  \
            .power_enabled              = true,                                     \
            .irq_enabled                = false,                                    \
            .irq_control_rate           = TCS3472_IRQ_RATE_EVERY_CYCLE,             \
            .gain_control               = TCS3472_GAIN_CONTROL_16X,                 \
            .integration_time           = 43.2f,                                    \
            .wait_time                  = 43.2f,                                    \
            .wait_time_enabled          = true,                                     \
            .long_wait_enabled          = false                                     \
}

/*
 * TCS3472 enumerator and structure declarations
*/

/**
 * @brief TCS3472 channels enumerator definition.
 */
typedef enum tcs3472_channels_e {
    TCS3472_CHANNEL_RED   = 1,  /*!< red channel  */
    TCS3472_CHANNEL_GREEN = 2,  /*!< green channel */
    TCS3472_CHANNEL_BLUE  = 3,  /*!< blue channel  */
    TCS3472_CHANNEL_CLEAR = 4   /*!< clear channel */
} tcs3472_channels_t;

/**
 * @brief TCS3472 interrupt rates enumerator definition.
 */
typedef enum tcs3472_irq_rates_e {
    TCS3472_IRQ_RATE_EVERY_CYCLE   = (0b0000),  /*!< every RGBC cycle generates an interrupt */
    TCS3472_IRQ_RATE_1_CLEAR_CHAN  = (0b0001),  /*!< 1 clear channel value outside of threshold range */
    /* TODO */
    TCS3472_IRQ_RATE_60_CLEAR_CHAN = (0b1111)   /*!< 1 clear channel consecutive values outside out of range */
} tcs3472_irq_rates_t;

/**
 * @brief TCS3472 RGBC gain control enumerator definition.
 */
typedef enum tcs3472_gain_controls_e {
    TCS3472_GAIN_CONTROL_1X   = (0b00),  /*!< RGBC gain of 1x */
    TCS3472_GAIN_CONTROL_4X   = (0b01),  /*!< RGBC gain of 4x */
    TCS3472_GAIN_CONTROL_16X  = (0b10),  /*!< RGBC gain of 16x */
    TCS3472_GAIN_CONTROL_60X  = (0b11),  /*!< RGBC gain of 60x */
} tcs3472_gain_controls_t;

/**
 * @brief TCS3472 enable register (0x00) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) tcs3472_enable_register_u {
    struct {
        bool    power_enabled:1; /*!< power on when enabled. (bit:0)  */
        bool    adc_enabled:1;   /*!< adc on when enabled which activates the RGBC. (bit:1)  */
        uint8_t reserved1:1;     /*!< reserved (bit:2) */
        bool    wait_enabled:1;  /*!< wait timer is active when enabled (bit:3) */
        bool    irq_enabled:1;   /*!< interrupt is active when enabled (bit:4) */
        uint8_t reserved2:3;     /*!< reserved (bit:5-7) */
    } bits;
    uint8_t reg;
} tcs3472_enable_register_t;

/**
 * @brief TCS3472 persistence register (0x01) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) tcs3472_persistence_register_u {
    struct {
        tcs3472_irq_rates_t irq_control_rate:4; /*!< interrupt persistence, controls rate of interrupt to the host processor (bit:0-3)  */
        uint8_t reserved1:4;                   /*!< reserved (bit:4-7) */
    } bits;
    uint8_t reg;
} tcs3472_persistence_register_t;

/**
 * @brief TCS3472 control register (0x0F) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) tcs3472_control_register_u {
    struct {
        tcs3472_gain_controls_t gain:2; /*!< RGBC gain control value. (bit:0-1)  */
        uint8_t reserved1:6;            /*!< reserved (bit:2-7) */
    } bits;
    uint8_t reg;
} tcs3472_control_register_t;

/**
 * @brief TCS3472 configuration register (0x0D) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) tcs3472_config_register_u {
    struct {
        uint8_t reserved1:1;     /*!< reserved (bit:0) */
        bool long_wait_enabled:1;/*!< When enabled, the wait cycles are increased by a factor 12× from that programmed in the WTIME register. (bit: 1)*/
        uint8_t reserved2:6;     /*!< reserved (bit:2-7) */
    } bits;
    uint8_t reg;
} tcs3472_config_register_t;

/**
 * @brief TCS3472 status register (0x13) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) tcs3472_status_register_u {
    struct {
        bool data_valid:1;       /*!< RGBC data is valid. (bit: 0) */
        uint8_t reserved1:3;     /*!< reserved (bit:1-3) */
        bool irq_clear_ch:1;     /*!< RGBC clear channel interrupt (bit: 4) */
        uint8_t reserved2:3;     /*!< reserved (bit:5-7) */
    } bits;
    uint8_t reg;
} tcs3472_status_register_t;


/**
 * @brief TCS3472 device configuration structure definition.
 */
typedef struct tcs3472_config_s {
    uint16_t                    i2c_address;        /*!< tcs3472 i2c device address */
    uint32_t                    i2c_clock_speed;    /*!< tcs3472 i2c device scl clock speed  */
    bool                        power_enabled;      /*!< tcs3472 power on when enabled  */
    tcs3472_irq_rates_t         irq_control_rate;   /*!< tcs3472 interrupt persistence, controls rate of interrupt to the host processor */ 
    tcs3472_gain_controls_t     gain_control;       /*!< tcs3472 RGBC gain control value */
    float                       integration_time;   /*!< tcs3472 RGBC integration time in 2.4-ms increments */
    float                       wait_time;          /*!< tcs3472 RGBC wait time in 2.4-ms increments */
    bool                        wait_time_enabled;  /*!< tcs3472 RGBC wait time is active when enabled */
    bool                        long_wait_enabled;  /*!< tcs3472 long wait timer is active when enabled */
    bool                        irq_enabled;        /*!< tcs3472 interrupt is active when enabled */
    bool                        set_irq_thresholds; /*!< tcs3472 sets RGBC clear channel high and low thresholds */
    uint16_t                    irq_high_threshold; /*!< tcs3472 RGBC clear channel high threshold */
    uint16_t                    irq_low_threshold;  /*!< tcs3472 RGBC clear channel low threshold */
} tcs3472_config_t;

/**
 * @brief TCS3472 channel data structure definition.
 */
typedef struct tcs3472_channels_data_s {
    uint16_t                    red;                /*!< red light channel count */
    uint16_t                    green;              /*!< green light channel count */
    uint16_t                    blue;               /*!< blue light channel count */
    uint16_t                    clear;              /*!< clear light channel count */
} tcs3472_channels_data_t;

/**
 * @brief TCS3472 opaque handle structure definition.
 */
typedef void* tcs3472_handle_t;

/**
 * @brief Reads enable register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg Enable register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_enable_register(tcs3472_handle_t handle, tcs3472_enable_register_t *const reg);

/**
 * @brief Writes enable register to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] reg Enable register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_enable_register(tcs3472_handle_t handle, const tcs3472_enable_register_t reg);

/**
 * @brief Reads RGBC integration time register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg RGBC time register controls the internal integration time of the RGBC clear and IR channel ADCs in 2.4-ms increments.
 *                 Max RGBC Count = (256 − ATIME) × 1024 up to a maximum of 65535.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_atime_register(tcs3472_handle_t handle, uint8_t *const reg);

/**
 * @brief Writes RGBC integration time register to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] reg RGBC time register controls the internal integration time of the RGBC clear and IR channel ADCs in 2.4-ms increments.
 *            Max RGBC Count = (256 − ATIME) × 1024 up to a maximum of 65535.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_atime_register(tcs3472_handle_t handle, const uint8_t reg);

/**
 * @brief Reads wait time register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg Wait time register is set 2.4 ms increments unless the WLONG bit is asserted, in which case the wait times are 12× longer.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_wtime_register(tcs3472_handle_t handle, uint8_t *const reg);

/**
 * @brief Writes wait time register to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] reg Wait time register is set 2.4 ms increments unless the WLONG bit is asserted, in which case the wait times are 12× longer.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_wtime_register(tcs3472_handle_t handle, const uint8_t reg);

/**
 * @brief Reads interrupt persistence filter register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg Persistence register controls the filtering interrupt capabilities of the device.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_persistence_register(tcs3472_handle_t handle, tcs3472_persistence_register_t *const reg);

/**
 * @brief Writes interrupt persistence filter register to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] reg Persistence register controls the filtering interrupt capabilities of the device.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_persistence_register(tcs3472_handle_t handle, const tcs3472_persistence_register_t reg);

/**
 * @brief Reads configuration register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_config_register(tcs3472_handle_t handle, tcs3472_config_register_t *const reg);

/**
 * @brief Writes configuration register to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_config_register(tcs3472_handle_t handle, const tcs3472_config_register_t reg);

/**
 * @brief Reads control register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg Control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_control_register(tcs3472_handle_t handle, tcs3472_control_register_t *const reg);

/**
 * @brief Writes control register to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] reg Control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_control_register(tcs3472_handle_t handle, const tcs3472_control_register_t reg);

/**
 * @brief Reads status register from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] reg Status register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_status_register(tcs3472_handle_t handle, tcs3472_status_register_t *const reg);


/**
 * @brief Initializes an TCS3472 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] tcs3472_config TCS3472 device configuration.
 * @param[out] tcs3472_handle TCS3472 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_init(i2c_master_bus_handle_t master_handle, const tcs3472_config_t *tcs3472_config, tcs3472_handle_t *tcs3472_handle);

/**
 * @brief Reads RGBC channels count data from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] data RGBC channels data structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_channels(tcs3472_handle_t handle, tcs3472_channels_data_t *const data);

/**
 * @brief Reads red channel count data from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] count Red channel count.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_red_channel_count(tcs3472_handle_t handle, uint16_t *const count);

/**
 * @brief Reads green channel count data from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] count Green channel count.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_green_channel_count(tcs3472_handle_t handle, uint16_t *const count);

/**
 * @brief Reads blue channel count data from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] count Blue channel count.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_blue_channel_count(tcs3472_handle_t handle, uint16_t *const count);

/**
 * @brief Reads clear channel count data from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] count Clear channel count.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_clear_channel_count(tcs3472_handle_t handle, uint16_t *const count);

/**
 * @brief Reads RGBC interrupt threshold registers from TCS3472.  The values to be used as the high and low trigger 
 * points for the comparison function for interrupt generation.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] high_threshold RGBC clear channel high threshold.
 * @param[out] low_threshold RGBC clear channel low threshold.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_irq_thresholds(tcs3472_handle_t handle, uint16_t *const high_threshold, uint16_t *const low_threshold);

/**
 * @brief Writes RGBC interrupt threshold registers to TCS3472.  The values to be used as the high and low trigger 
 * points for the comparison function for interrupt generation.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] high_threshold RGBC clear channel high threshold.
 * @param[in] low_threshold RGBC clear channel low threshold.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_irq_thresholds(tcs3472_handle_t handle, const uint16_t high_threshold, const uint16_t low_threshold);

/**
 * @brief Reads RGBC gain control from TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] gain RGBC gain control for the analog block.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_gain_control(tcs3472_handle_t handle, tcs3472_gain_controls_t *const gain);

/**
 * @brief Writes RGBC gain control to TCS3472.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] gain RGBC gain control for the analog block.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_gain_control(tcs3472_handle_t handle, const tcs3472_gain_controls_t gain);

/**
 * @brief Reads integration time of the RGBC clear and IR channel ADCs from the TCS3472 in 2.4-ms increments.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] time Integration time in 2.4-ms increments.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_integration_time(tcs3472_handle_t handle, float *const time);

/**
 * @brief Writes integration time of the RGBC clear and IR channel ADCs to the TCS3472 in 2.4-ms increments.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] time Integration time in 2.4-ms increments.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_integration_time(tcs3472_handle_t handle, const float time);

/**
 * @brief Reads wait time from TCS3472 in 2.4-ms increments.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] time Wait time in 2.4-ms increments unless long wait time is enabled.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_wait_time(tcs3472_handle_t handle, float *const time);

/**
 * @brief Writes wait time to TCS3472 in 2.4-ms increments.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[in] time Wait time in 2.4-ms increments unless long wait time is enabled.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_set_wait_time(tcs3472_handle_t handle, const float time);

/**
 * @brief Reads data flag from TCS3472 device.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] ready RGBC channels have completed an integration cycle when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_data_status(tcs3472_handle_t handle, bool *const ready);

/**
 * @brief Reads data and irq status flags from TCS3472 device.
 * 
 * @param[in] handle TCS3472 device handle.
 * @param[out] data_ready RGBC channels have completed an integration cycle when true.
 * @param[out] irq_asserted RGBC clear channel interrupt when asserted.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_get_status(tcs3472_handle_t handle, bool *const data_ready, bool *const irq_asserted);

/**
 * @brief Enables power from TCS3472 device.
 * 
 * @param[in] handle TCS3472 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_enable_power(tcs3472_handle_t handle);

/**
 * @brief Disables power from TCS3472 device.
 * 
 * @param[in] handle TCS3472 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_disable_power(tcs3472_handle_t handle);

/**
 * @brief Removes an TCS3472 device from master bus.
 *
 * @param[in] handle TCS3472 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_remove(tcs3472_handle_t handle);

/**
 * @brief Removes an TCS3472 device from master I2C bus and delete the handle.
 * 
 * @param[in] handle TCS3472 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_delete(tcs3472_handle_t handle);


/**
 * @brief Converts TCS3472 firmware version numbers (major, minor, patch) into a string.
 * 
 * @return char* TCS3472 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* tcs3472_get_fw_version(void);

/**
 * @brief Converts TCS3472 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t TCS3472 firmware version number.
 */
int32_t tcs3472_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __TCS3472_H__
