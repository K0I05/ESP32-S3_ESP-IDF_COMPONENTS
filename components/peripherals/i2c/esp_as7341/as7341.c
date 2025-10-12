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
 * @file as7341.c
 *
 * ESP-IDF driver for AS7341 11-channel spectrometer (350nm to 1000nm)
 *
 * 
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/as7341.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * AS7341 definitions
*/

#define AS7341_PART_ID              UINT8_C(0x09)   //!< as7341 I2C device part identifier

//#define I2C_AS7341_ASTATUS              (0x60)  //!< as7341 (see i2c_as7341_astatus_register_t)
//#define I2C_AS7341_CH0_ADC_DATA_L       (0x61)  //!< as7341
//#define I2C_AS7341_CH0_ADC_DATA_H       (0x62)  //!< as7341
#define AS7341_ITIME_L              UINT8_C(0x63)  //!< as7341
#define AS7341_ITIME_M              UINT8_C(0x64)  //!< as7341
#define AS7341_ITIME_H              UINT8_C(0x65)  //!< as7341
//#define I2C_AS7341_CH1_ADC_DATA_L       (0x66)  //!< as7341
//#define I2C_AS7341_CH1_ADC_DATA_H       (0x67)  //!< as7341
//#define I2C_AS7341_CH2_ADC_DATA_L       (0x68)  //!< as7341
//#define I2C_AS7341_CH2_ADC_DATA_H       (0x69)  //!< as7341
//#define I2C_AS7341_CH3_ADC_DATA_L       (0x6a)  //!< as7341
//#define I2C_AS7341_CH3_ADC_DATA_H       (0x6b)  //!< as7341
//#define I2C_AS7341_CH4_ADC_DATA_L       (0x6c)  //!< as7341
//#define I2C_AS7341_CH4_ADC_DATA_H       (0x6d)  //!< as7341
//#define I2C_AS7341_CH5_ADC_DATA_L       (0x6e)  //!< as7341
//#define I2C_AS7341_CH5_ADC_DATA_H       (0x6f)  //!< as7341

#define AS7341_CONFIG               UINT8_C(0x70)  //!< as7341 (see i2c_as7341_config_register_t)
#define AS7341_DEV_STATUS           UINT8_C(0x71)  //!< as7341 (see i2c_as7341_device_status_register_t)
#define AS7341_EDGE                 UINT8_C(0x72)  //!< as7341
#define AS7341_GPIO1                UINT8_C(0x73)  //!< as7341 (see i2c_as7341_gpio1_register_t)
#define AS7341_LED                  UINT8_C(0x74)  //!< as7341 (see i2c_as7341_led_register_t)
#define AS7341_ENABLE               UINT8_C(0x80)  //!< as7341 (see i2c_as7341_enable_register_t)
#define AS7341_ATIME                UINT8_C(0x81)  //!< as7341
#define AS7341_WTIME                UINT8_C(0x83)  //!< as7341
#define AS7341_SP_TH_L_LSB          UINT8_C(0x84)  //!< as7341
#define AS7341_SP_TH_L_MSB          UINT8_C(0x85)  //!< as7341
#define AS7341_SP_TH_H_LSB          UINT8_C(0x86)  //!< as7341
#define IAS7341_SP_TH_H_MSB         UINT8_C(0x87)  //!< as7341
#define AS7341_AUXID                UINT8_C(0x90)  //!< as7341
#define AS7341_REVID                UINT8_C(0x91)  //!< as7341
#define AS7341_ID                   UINT8_C(0x92)  //!< as7341

#define AS7341_INT_STATUS           UINT8_C(0x93)  //!< as7341 (see i2c_as7341_interrupt_status_register_t)

#define AS7341_ASTATUS              UINT8_C(0x94)  //!< as7341 (see i2c_as7341_astatus_register_t)
#define AS7341_CH0_ADC_DATA_L       UINT8_C(0x95)  //!< as7341
#define AS7341_CH0_ADC_DATA_H       UINT8_C(0x96)  //!< as7341
#define AS7341_CH1_ADC_DATA_L       UINT8_C(0x97)  //!< as7341
#define AS7341_CH1_ADC_DATA_H       UINT8_C(0x98)  //!< as7341
#define AS7341_CH2_ADC_DATA_L       UINT8_C(0x99)  //!< as7341
#define AS7341_CH2_ADC_DATA_H       UINT8_C(0x9a)  //!< as7341
#define AS7341_CH3_ADC_DATA_L       UINT8_C(0x9b)  //!< as7341
#define AS7341_CH3_ADC_DATA_H       UINT8_C(0x9c)  //!< as7341
#define AS7341_CH4_ADC_DATA_L       UINT8_C(0x9d)  //!< as7341
#define AS7341_CH4_ADC_DATA_H       UINT8_C(0x9e)  //!< as7341
#define AS7341_CH5_ADC_DATA_L       UINT8_C(0x9f)  //!< as7341
#define AS7341_CH5_ADC_DATA_H       UINT8_C(0xa0)  //!< as7341

#define AS7341_STATUS2              UINT8_C(0xa3)  //!< as7341 (see i2c_as7341_status2_register_t)
#define AS7341_STATUS3              UINT8_C(0xa4)  //!< as7341 (see i2c_as7341_status3_register_t)
#define AS7341_STATUS5              UINT8_C(0xa6)  //!< as7341 (see i2c_as7341_status5_register_t)
#define AS7341_STATUS6              UINT8_C(0xa7)  //!< as7341 (see i2c_as7341_status6_register_t)

#define AS7341_CONFIG0              UINT8_C(0xa9)  //!< as7341 (see i2c_as7341_config0_register_t)
#define AS7341_CONFIG1              UINT8_C(0xaa)  //!< as7341 (see i2c_as7341_config1_register_t)
#define AS7341_CONFIG3              UINT8_C(0xac)  //!< as7341 (see i2c_as7341_config3_register_t)
#define AS7341_CONFIG6              UINT8_C(0xaf)  //!< as7341 (see i2c_as7341_config6_register_t)
#define AS7341_CONFIG8              UINT8_C(0xb1)  //!< as7341 (see i2c_as7341_config8_register_t)
#define AS7341_CONFIG9              UINT8_C(0xb2)  //!< as7341 (see i2c_as7341_config9_register_t)
#define AS7341_CONFIG10             UINT8_C(0xb3)  //!< as7341 (see i2c_as7341_config10_register_t)
#define AS7341_CONFIG12             UINT8_C(0xb5)  //!< as7341 (see i2c_as7341_config12_register_t)

#define AS7341_PERS                 UINT8_C(0xbd)  //!< as7341 (see i2c_as7341_pers_register_t)
#define AS7341_GPIO2                UINT8_C(0xbe)  //!< as7341 (see i2c_as7341_gpio2_register_t)
#define AS7341_ASTEP_L              UINT8_C(0xca)  //!< as7341
#define AS7341_ASTEP_H              UINT8_C(0xcb)  //!< as7341
#define AS7341_AGC_GAIN_MAX         UINT8_C(0xcf)  //!< as7341 (see i2c_as7341_agc_gain_register_t)
#define AS7341_AZ_CONFIG            UINT8_C(0xd6)  //!< as7341
#define AS7341_FD_TIME1             UINT8_C(0xd8)  //!< as7341
#define AS7341_FD_TIME2             UINT8_C(0xda)  //!< as7341 (see i2c_as7341_fd_time2_register_t)
#define AS7341_FD_CONFIG0           UINT8_C(0xd7)  //!< as7341 (see i2c_as7341_fd_config0_register_t)
#define AS7341_FD_STATUS            UINT8_C(0xdb)  //!< as7341 (see i2c_as7341_fd_status_register_t)
#define AS7341_INTENAB              UINT8_C(0xf9)  //!< as7341 (see i2c_as7341_interrupt_enable_register_t)
#define AS7341_CONTROL              UINT8_C(0xfa)  //!< as7341 (see i2c_as7341_control_register_t)


#define AS7341_DATA_POLL_TIMEOUT_MS UINT16_C(1000)
#define AS7341_DATA_READY_DELAY_MS  UINT16_C(1)
#define AS7341_POWERUP_DELAY_MS     UINT16_C(200)
#define AS7341_APPSTART_DELAY_MS    UINT16_C(200)
#define AS7341_RESET_DELAY_MS       UINT16_C(25)
#define AS7341_SETUP_DELAY_MS       UINT16_C(15)
#define AS7341_CMD_DELAY_MS         UINT16_C(5)
#define AS7341_TX_RX_DELAY_MS       UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)



/**
 * @brief AS7341 enable register (0x80) structure. 
 * 
 * @note To operate the device enable power and interrupts before enabling spectral measurements.
 * 
 */
typedef union __attribute__((packed)) as7341_enable_register_u {
    struct {
        bool    power_enabled:1;                /*!< as7341 power enabled when true                                                             (bit:0)  */
        bool    spectral_measurement_enabled:1; /*!< as7341 spectral measurement enabled when true                                              (bit:1)  */
        uint8_t reserved1:1;                    /*!< reserved                                                                                   (bit:2) */
        bool    wait_time_enabled:1;            /*!< as7341 wait time between two consecutive spectral measurements enabled when true           (bit:3) */
        bool    smux_enabled:1;                 /*!< as7341 starts SMUX command when true and bit is cleared when SMUX operation is finished    (bit:4) */
        uint8_t reserved2:1;                    /*!< reserved                                                                                   (bit:5) */
        bool    flicker_detection_enabled:1;    /*!< as7341 flicker detection enabled when true                                                 (bit:6)  */
        uint8_t reserved3:1;                    /*!< reserved                                                                                   (bit:7) */
    } bits;
    uint8_t reg;
} as7341_enable_register_t;

/**
 * @brief AS7341 configuration register (0x70) structure.
 */
typedef union __attribute__((packed)) as7341_config_register_u {
    struct {
        as7341_als_modes_t      irq_mode:2;                 /*!< as7341 ambient light sensing mode                              (bit:0-1)  */
        bool                    irq_sync_signal_enabled:1;  /*!< as7341 sync signal applied on output pin interrupt when true   (bit:2) */
        bool                    led_ldr_control_enabled:1;  /*!< as7341 register LED controls LED connected to pin LDR when true (register 0x74), otherwise, exernal LED is not controlled by as7341 (bit:3) */
        uint8_t                 reserved:4;                 /*!< reserved                                                       (bit:4-7) */
    } bits;
    uint8_t reg;
} as7341_config_register_t;

/**
 * @brief AS7341 flicker detection time 1 register (0xD8) structure.  THIS ISN'T NEEDED
 */
typedef union __attribute__((packed)) as7341_flicker_detection_time1_register_u {
    struct {
        uint8_t                 fd_integration_time:8;  /*!< as7341 flicker detection integration time LSB (t = FD_TIME * 2.78us) (bit:0-7) */
    } bits;
    uint8_t reg;
} as7341_flicker_detection_time1_register_t;

/**
 * @brief AS7341 flicker detection time 2 register (0xDA) structure.
 */
typedef union __attribute__((packed)) as7341_flicker_detection_time2_register_u {
    struct {
        uint8_t                          fd_integration_time:3;  /*!< as7341 flicker detection integration time MSB (t = FD_TIME * 2.78us) (bit:0-2) */
        as7341_flicker_detection_gains_t fd_gain:5;              /*!< as7341 flicker detection gain      (bit:3-7) */
    } bits;
    uint8_t reg;
} as7341_flicker_detection_time2_register_t;

/**
 * @brief AS7341 gpio1 register (0x73) structure.
 */
typedef union __attribute__((packed)) as7341_gpio1_register_u {
    struct {
        bool                    photo_diode_irq_enabled:1;  /*!< as7341 photo diode connected to interrupt pin when true (bit:0) */
        bool                    photo_diode_gpio_enabled:1; /*!< as7341 photo diode connected to GPIO pin when true      (bit:1) */
        uint8_t                 reserved:6;                 /*!< reserved                                                (bit:2-7) */
    } bits;
    uint8_t reg;
} as7341_gpio1_register_t;

/**
 * @brief AS7341 gpio2 register (0xbe) structure.
 */
typedef union __attribute__((packed)) as7341_gpio2_register_u {
    struct {
        bool                    gpio_input_state:1;     /*!< as7341 GPIO pin is input when true if gpio input is enabled (bit:0) */
        bool                    gpio_output_state:1;    /*!< as7341 output state of GPIO pin is active when true         (bit:1) */
        bool                    gpio_input_enabled:1;   /*!< as7341 GPIO pin accepts non-floating input when true        (bit:2) */
        bool                    gpio_output_inverted:1; /*!< as7341 output state of GPIO pin is inverted when true       (bit:3) */
        uint8_t                 reserved:4;             /*!< reserved                                                    (bit:4-7) */
    } bits;
    uint8_t reg;
} as7341_gpio2_register_t;

/**
 * @brief AS7341 led register (0x74) structure.
 */
typedef union __attribute__((packed)) as7341_led_register_u {
    struct {
        as7341_led_drive_strengths_t    led_drive_strength:7;   /*!< as7341 LED driving strength (bit:0-6) */
        bool                            led_ldr_enabled:1;      /*!< as7341 external LED connected to LDR pin is enabled when true (bit:7) */
    } bits;
    uint8_t reg;
} as7341_led_register_t;

/**
 * @brief AS7341 interrupt enable register (0xf9) structure.
 */
typedef union __attribute__((packed)) as7341_interrupt_enable_register_u {
    struct {
        bool                    irq_system_enabled:1;           /*!< as7341 interrupt asserted when system interrupts occur when enabled (bit:0) */
        uint8_t                 reserved1:1;                    /*!< reserved         (bit:1) */
        bool                    irq_fifo_enabled:1;             /*!< as7341 interrupt asserted when FIFO LVL exceeds FIFO threshold condition when enabled (bit:2) */
        bool                    irq_spectral_enabled:1;         /*!< as7341 interrupt asserted subject to spectral thresholds and persistence filter when enabled (bit:3) */
        uint8_t                 reserved2:3;                    /*!< reserved         (bit:4-6) */
        bool                    irq_spectral_flicker_enabled:1; /*!< as7341 spectral and flicker detection interrupt asserted when enabled (bit:7) */
    } bits;
    uint8_t reg;
} as7341_interrupt_enable_register_t;

/**
 * @brief AS7341 interrupt status register (0x93) structure.
 */
typedef union __attribute__((packed)) as7341_interrupt_status_register_u {
    struct {
        bool                    irq_system:1;           /*!< as7341 system interrupt asserted when true (bit:0) */
        bool                    irq_calibration:1;      /*!< as7341 calibration interrupt asserted when true         (bit:1) */
        bool                    irq_fifo:1;             /*!< as7341 interrupt asserted when FIFO LVL exceeds FIFO threshold condition when true (bit:2) */
        bool                    irq_spectral:1;         /*!< as7341 interrupt asserted subject to spectral threholds and persistence filter when true (bit:3) */
        uint8_t                 reserved:3;             /*!< reserved         (bit:4-6) */
        bool                    irq_spectral_flicker:1; /*!< as7341 spectral and flicker detection interrupt asserted when true (bit:7) */
    } bits;
    uint8_t reg;
} as7341_interrupt_status_register_t;

/**
 * @brief AS7341 device status register (0x71) structure.
 */
typedef union __attribute__((packed)) as7341_device_status_register_u {
    struct {
        bool    data_ready:1;       /*!< as7341 spectral measurement status is ready when true (bit:0) */
        bool    gpio_wait_sync:1;   /*!< as7341 device waits for sync pulse on GPIO to start integration when true (bit:1) */
        uint8_t reserved:6;         /*!< reserved                       (bit:2-7) */
    } bits;
    uint8_t reg;
} as7341_device_status_register_t;

/**
 * @brief AS7341 astatus register (0x94) structure.
 */
typedef union __attribute__((packed)) as7341_astatus_register_u {
    struct {
        uint8_t again_status:4; /*!< as7341 gain applied for the spectral data latched to this ASTATUS read (bit:3-0) */
        uint8_t reserved:3;     /*!< reserved                                                               (bit:4-6) */
        bool    asat_status:1;  /*!< as7341 latched data is affected by analog or digital saturation        (bit:7) */
    } bits;
    uint8_t reg;
} as7341_astatus_register_t;

/**
 * @brief AS7341 status 2 register (0xa3) structure.
 */
typedef union __attribute__((packed)) as7341_status2_register_u {
    struct {
        bool    flicker_detect_digital_saturation:1;    /*!< as7341 maximum counter value has been reached during flicker detection when true (bit:0) */
        bool    flicker_detect_analog_saturation:1;     /*!< as7341 intensity of ambient light ha exceeded the maximum integration level for analg circuit for flicker detection when true (bit:1) */
        uint8_t reserved1:1;                            /*!< reserved         (bit:2) */
        bool    analog_saturation:1;                    /*!< as7341 intensity of ambient light ha exceeded the maximum integration level for spectral analg circuit when true (bit:3) */
        bool    digital_saturation:1;                   /*!< as7341 maximum counter value has been reached when true, dependent of ATIME register (bit:4) */
        uint8_t reserved2:1;                            /*!< reserved         (bit:5) */
        bool    spectral_valid:1;                       /*!< as7341 spectral measurement has been completed when true (bit:6) */
        uint8_t reserved3:1;                            /*!< reserved         (bit:7) */
    } bits;
    uint8_t reg;
} as7341_status2_register_t;

/**
 * @brief AS7341 status 3 register (0xa4) structure.
 */
typedef union __attribute__((packed)) as7341_status3_register_u {
    struct {
        uint8_t reserved1:4;                    /*!< reserved                                                                   (bit:3-0) */
        bool    irq_spectral_threshold_lo:1;    /*!< as7341 spectral interrupt asserted when data is below the low threshold    (bit:4) */
        bool    irq_spectral_threshold_hi:1;    /*!< as7341 spectral interrupt asserted when data is above the high threshold   (bit:5) */
        uint8_t reserved2:2;                    /*!< reserved                                                                   (bit:6-7) */
    } bits;
    uint8_t reg;
} as7341_status3_register_t;

/**
 * @brief AS7341 status 5 register (0xa6) structure.
 */
typedef union __attribute__((packed)) as7341_status5_register_u {
    struct {
        uint8_t reserved1:2;                /*!< reserved                                                                           (bit:1-0) */
        bool    irq_smux_operation:1;       /*!< as7341 SMUX command execution has finished when true                               (bit:2) */
        bool    irq_flicker_detection:1;    /*!< as7341 FD_STATUS register status has changed when true and if SIEN_FD is enabled   (bit:3) */
        uint8_t reserved2:4;                /*!< reserved                                                                           (bit:4-7) */
    } bits;
    uint8_t reg;
} as7341_status5_register_t;

/**
 * @brief AS7341 status 6 register (0xa7) structure.
 */
typedef union __attribute__((packed)) as7341_status6_register_u {
    struct {
        bool    initializing:1;                 /*!< as7341 device is initializing when true, wait until cleared                (bit:0) */
        bool    sleep_after_irq:1;              /*!< as7341 device is in sleep due to an interrupt when true                    (bit:1) */
        bool    spectral_trigger_error:1;       /*!< as7341 timing error when true, WTIME to short for ATIME                    (bit:2) */
        uint8_t reserved1:1;                    /*!< reserved                                                                   (bit:3) */
        bool    flicker_detect_trigger_error:1; /*!< as7341 timing error that prevents flicker detect from working correctly    (bit:4) */
        bool    over_temperature_detected:1;    /*!< as7341 device temperature is to high when true                             (bit:5) */
        uint8_t reserved2:1;                    /*!< reserved                                                                   (bit:6) */
        bool    fifo_buffer_overflow:1;         /*!< as7341 FIFO buffer overflowed and information is lost when true            (bit:7) */
    } bits;
    uint8_t reg;
} as7341_status6_register_t;

/**
 * @brief AS7341 flicker detection status register (0xdb) structure.
 */
typedef union __attribute__((packed)) as7341_flicker_detection_status_register_u {
    struct {
        bool    fd_100hz_flicker:1;         /*!< as7341 ambient light source is flickering at 100Hz when true                       (bit:0) */
        bool    fd_120hz_flicker:1;         /*!< as7341 ambient light source is flickering at 120Hz when true                       (bit:1) */
        bool    fd_100hz_flicker_valid:1;   /*!< as7341 100Hz flicker detection calculation is valid when true, write true to clear (bit:2) */
        bool    fd_120hz_flicker_valid:1;   /*!< as7341 120Hz flicker detection calculation is valid when true, write true to clear (bit:3) */
        bool    fd_saturation_detected:1;   /*!< as7341 saturation ocurred during the last flicker detection measurement when true,write true to clear  (bit:4) */
        bool    fd_measurement_valid:1;     /*!< as7341 flicker detection measurement is complete when true, write true to clear    (bit:5) */
        uint8_t reserved:2;                 /*!< reserved                                                                           (bit:6-7) */
    } bits;
    uint8_t reg;
} as7341_flicker_detection_status_register_t;

/**
 * @brief AS7341 auxiliary identifier register (0x90) structure.
 */
typedef union __attribute__((packed)) as7341_auxiliary_id_register_u {
    struct {
        uint8_t identifier:4;   /*!< as7341 auxiliary identification    (bit:0-3) */
        uint8_t reserved:4;     /*!< reserved                           (bit:4-7) */
    } bits;
    uint8_t reg;
} as7341_auxiliary_id_register_t;

/**
 * @brief AS7341 revision number identifier register (0x91) structure.
 */
typedef union __attribute__((packed)) as7341_revision_id_register_u {
    struct {
        uint8_t identifier:3;   /*!< as7341 revision number identification  (bit:0-2) */
        uint8_t reserved:5;     /*!< reserved                               (bit:3-7) */
    } bits;
    uint8_t reg;
} as7341_revision_id_register_t;

/**
 * @brief AS7341 part number identifier register (0x92) structure.
 */
typedef union __attribute__((packed)) as7341_part_id_register_u {
    struct {
        uint8_t reserved:2;     /*!< reserved                                   (bit:0-1) */
        uint8_t identifier:6;   /*!< as7341 device part number identification   (bit:2-7) */
    } bits;
    uint8_t reg;
} as7341_part_id_register_t;

/**
 * @brief AS7341 configuration 0 register (0xa9) structure.
 */
typedef union __attribute__((packed)) as7341_config0_register_u {
    struct {
        uint8_t reserved1:2;        /*!< reserved                                                   (bit:1-0) */
        bool    trigger_long:1;     /*!< increase te WTIME setting by a factor of 16 when asserted  (bit:2) */
        uint8_t reserved2:1;        /*!< reserved                                                   (bit:3) */
        bool    reg_bank_access:1;  /*!< bit needs to be set to access registers 0x60 to 0x70       (bit:4) */
        bool    low_power:1;        /*!< device will run in a low power modem when asserted         (bit:5) */
        uint8_t reserved3:2;        /*!< reserved                                                   (bit:6-7) */
    } bits;
    uint8_t reg;
} as7341_config0_register_t;

/**
 * @brief AS7341 configuration 1 register (0xaa) structure.
 */
typedef union __attribute__((packed)) as7341_config1_register_u {
    struct {
        as7341_spectral_gains_t spectral_gain:5;    /*!< spectral sensitivity  (bit:4-0) */
        uint8_t                 reserved:3;         /*!< reserved              (bit:5-7) */
    } bits;
    uint8_t reg;
} as7341_config1_register_t;

/**
 * @brief AS7341 configuration 6 register (0xaf) structure.
 */
typedef union __attribute__((packed)) as7341_config6_register_u {
    struct {
        uint8_t                 reserved1:3;    /*!< reserved                                                   (bit:2-0) */
        as7341_smux_commands_t  smux_command:2; /*!< as7341 SMUX command to execute when smux enabled gets set  (bit:3-4) */
        uint8_t                 reserved2:3;    /*!< reserved                                                   (bit:5-7) */
    } bits;
    uint8_t reg;
} as7341_config6_register_t;

/**
 * @brief AS7341 device descriptor structure definition.
 */
typedef struct as7341_device_s {
    as7341_config_t             config;         /*!< as7341 device configuration */
    i2c_master_dev_handle_t     i2c_handle;     /*!< as7341 i2c device handle */
    uint8_t                     part_id;
    uint8_t                     revision_id;
} as7341_device_t;

/*
* static constant declarations
*/
static const char *TAG = "as7341";



/**
 * @brief AS7341 I2C HAL write byte to register address transaction.
 * 
 * @param device AS7341 device descriptor.
 * @param reg_addr AS7341 register address to write to.
 * @param byte AS7341 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_write_byte_to(as7341_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief AS7341 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device AS7341 device descriptor.
 * @param reg_addr AS7341 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_read_from(as7341_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "as7341_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief AS7341 I2C HAL read halfword from register address transaction.
 * 
 * @param device AS7341 device descriptor.
 * @param reg_addr AS7341 register address to read from.
 * @param word AS7341 read transaction return halfword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_read_word_from(as7341_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "as7341_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief AS7341 I2C HAL read byte from register address transaction.
 * 
 * @param device AS7341 device descriptor.
 * @param reg_addr AS7341 register address to read from.
 * @param byte AS7341 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_read_byte_from(as7341_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "as7341_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}


/**
 * @brief Configures SMUX registers for low channels F1-F4, Clear and NIR.
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_setup_smux_lo_channels(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write config transactions (F1, F2, F3, F4, NIR, CLEAR) */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x00, 0x30), TAG, "write F3 left set to ADC2 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x01, 0x01), TAG, "write F1 left set to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x02, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x03, 0x00), TAG, "write F8 left disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x04, 0x00), TAG, "write F6 left disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x05, 0x42), TAG, "write F4 left connected to ADC3/F2 connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x06, 0x00), TAG, "write F5 left disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x07, 0x00), TAG, "write F7 left disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x08, 0x50), TAG, "write CLEAR connected ADC4 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x09, 0x00), TAG, "write F5 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0a, 0x00), TAG, "write F7 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0b, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0c, 0x20), TAG, "write F2 right connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0d, 0x04), TAG, "write F4 right connected to ADC3 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0e, 0x00), TAG, "write F6/F8 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0f, 0x30), TAG, "write F3 right connected to ADC2 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x10, 0x01), TAG, "write F1 right connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x11, 0x50), TAG, "write CLEAR right connected to ADC4 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x12, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x13, 0x06), TAG, "write NIR connected to ADC5 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for high channels F5-F8, Clear and NIR.
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_setup_smux_hi_channels(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write config transactions (F5, F6, F7, F8, NIR, CLEAR) */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x00, 0x00), TAG, "write F3 left disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x01, 0x00), TAG, "write F1 left disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x02, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x03, 0x40), TAG, "write F8 left connected to ADC3 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x04, 0x02), TAG, "write F6 left connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x05, 0x00), TAG, "write F4/F2 disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x06, 0x10), TAG, "write F5 left connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x07, 0x03), TAG, "write F7 left connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x08, 0x50), TAG, "write CLEAR connected to ADC4 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x09, 0x10), TAG, "write F5 right connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0a, 0x03), TAG, "write F7 right connected to ADC0 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0b, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0c, 0x00), TAG, "write F2 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0d, 0x00), TAG, "write F4 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0e, 0x24), TAG, "write F8 right connected to ADC2/F6 right connected to ADC1 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0f, 0x00), TAG, "write F3 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x10, 0x00), TAG, "write F1 right disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x11, 0x50), TAG, "write CLEAR right connected to ADC4 register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x12, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x13, 0x06), TAG, "write NIR connected to ADC5 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for flicker detection.
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_setup_smux_flicker_detection(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write config transactions */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x00, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x01, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x02, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x03, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x04, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x05, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x06, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x07, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x08, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x09, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0a, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0b, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0c, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0d, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0e, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x0f, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x10, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x11, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x12, 0x00), TAG, "write reserved or disabled register failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, 0x13, 0x60), TAG, "write flicker connected to ADC5 to left of 0x13 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads ASTATUS register (0x94) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg ASTATUS register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_astatus_register(as7341_device_t *const device, as7341_astatus_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_ASTATUS, &reg->reg), TAG, "read astatus register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads status 2 register (0xA3) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Status 2 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_status2_register(as7341_device_t *const device, as7341_status2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_STATUS2, &reg->reg), TAG, "read status 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads enable register (0x80) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Enable register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_enable_register(as7341_device_t *const device, as7341_enable_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_ENABLE, &reg->reg), TAG, "read enable register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes enable register (0x80) to AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Enable register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_enable_register(as7341_device_t *const device, const as7341_enable_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    as7341_enable_register_t enable = { .reg = reg.reg };

    /* set reserved bits */
    enable.bits.reserved1 = 0;
    enable.bits.reserved2 = 0;
    enable.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_ENABLE, enable.reg), TAG, "write enable register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Disables spectral readings, flicker detection, power, etc.
 * 
 * @param[in] device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_disable_enable_register(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    as7341_enable_register_t enable = { .reg = 0 };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "disable enable register failed" );

    return ESP_OK;
}

/**
 * @brief Reads auxiliary id register (0x90) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Auxiliary id register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_auxiliary_id_register(as7341_device_t *const device, as7341_auxiliary_id_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_AUXID, &reg->reg), TAG, "read auxiliary id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads revision id register (0x91) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Revision id register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_revision_id_register(as7341_device_t *const device, as7341_revision_id_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_REVID, &reg->reg), TAG, "read revision id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads part id register (0x92) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Part id register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_part_id_register(as7341_device_t *const device, as7341_part_id_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_ID, &reg->reg), TAG, "read part id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads configuration register (0x70) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Configuration register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_config_register(as7341_device_t *const device, as7341_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_CONFIG, &reg->reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes configuration register (0x70) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Configuration register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_config_register(as7341_device_t *const device, const as7341_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    as7341_config_register_t config = { .reg = reg.reg };

    /* set reserved bits */
    config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads configuration 0 register (0xA9) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Configuration 0 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_config0_register(as7341_device_t *const device, as7341_config0_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_CONFIG0, &reg->reg), TAG, "read configuration 0 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes configuration 0 register (0xA9) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Configuration 0 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_config0_register(as7341_device_t *const device, const as7341_config0_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    as7341_config0_register_t config0 = { .reg = reg.reg };

    /* set reserved bits */
    config0.bits.reserved1 = 0;
    config0.bits.reserved2 = 0;
    config0.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_CONFIG0, config0.reg), TAG, "write configuration 0 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads configuration 1 (0xAA) register from AS7341.  This register configures the 6 integrated ADC (CH0 to CH5).
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Configuration 1 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_config1_register(as7341_device_t *const device, as7341_config1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_CONFIG1, &reg->reg), TAG, "read configuration 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes configuration 1 (0xAA) register to AS7341.  This register configures the 6 integrated ADC (CH0 to CH5).
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Configuration 1 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_config1_register(as7341_device_t *const device, const as7341_config1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    as7341_config1_register_t config1 = { .reg = reg.reg };

    /* set reserved bits */
    config1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_CONFIG1, config1.reg), TAG, "write configuration 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads configuration 6 register (0xAF) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Configuration 6 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_config6_register(as7341_device_t *const device, as7341_config6_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_CONFIG6, &reg->reg), TAG, "read configuration 6 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes configuration 6 register (0xAF) to AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Configuration 6 register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_config6_register(as7341_device_t *const device, const as7341_config6_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    as7341_config6_register_t config6 = { .reg = reg.reg };

    /* set reserved bits */
    config6.bits.reserved1 = 0;
    config6.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_CONFIG6, config6.reg), TAG, "write configuration 6 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads ATIME (0x81) register from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Number of integration steps from 1 to 256, a value of 29 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_atime_register(as7341_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_ATIME, reg), TAG, "read atime register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes ATIME (0x81) register to AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Number of integration steps from 1 to 256, a value of 29 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_atime_register(as7341_device_t *const device, const uint8_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_ATIME, reg), TAG, "write atime register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads ASTEP (0xCA, 0xCB) register from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Integration time step size.  Integration time step increment of 2.78us, a value of 599 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_astep_register(as7341_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_word_from(device, AS7341_ASTEP_L, reg), TAG, "read astep register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes ASTEP (0xCA, 0xCB) register to AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Integration time step size.  Integration time step increment of 2.78us, a value of 599 is recommended as a starting point, 50ms integration time.  ATIME and ASTEP cannot both be zero.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_astep_register(as7341_device_t *const device, const uint16_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_ASTEP_L, reg), TAG, "write astep register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Reads flicker detection status (0xDB) register from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg Flicker detection status register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_flicker_detection_status_register(as7341_device_t *const device, as7341_flicker_detection_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_FD_STATUS, &reg->reg), TAG, "read flicker detection status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes flicker detection status (0xDB) register to AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg Flicker detection status register structure.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_flicker_detection_status_register(as7341_device_t *const device, const as7341_flicker_detection_status_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    as7341_flicker_detection_status_register_t flicker_detection_status = { .reg = reg.reg };

    /* set reserved bits */
    flicker_detection_status.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_FD_STATUS, flicker_detection_status.reg), TAG, "write flicker detection status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Clears flicker detection status (0xDB) register on AS7341.
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_clear_flicker_detection_status_register(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* clear resettable flags */
    as7341_flicker_detection_status_register_t flicker_detection_status = {
        .bits.fd_saturation_detected = true,
        .bits.fd_measurement_valid   = true,
        .bits.fd_100hz_flicker_valid = true,
        .bits.fd_120hz_flicker_valid = true
    };

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_flicker_detection_status_register(device, flicker_detection_status), TAG, "write flicker detection status register for clear flicker detection status register failed" );

    return ESP_OK;
}

/**
 * @brief Enables access to the AS7341 high register bank (0x80 to 0xFF).
 * 
 * @param[in] device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_enable_hi_register_bank(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register from handle */
    as7341_config0_register_t config0_reg;

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config0_register(device, &config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    /* enable high registers */
    config0_reg.bits.reg_bank_access = false; // 0 or false to access register 0x80 and above

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config0_register(device, config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    return ESP_OK;
}

/**
 * @brief Enables access to the AS7341 low register bank (0x60 to 0x74).
 * 
 * @param[in] device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_enable_lo_register_bank(as7341_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register from handle */
    as7341_config0_register_t config0_reg;

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config0_register(device, &config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    /* enable low registers */
    config0_reg.bits.reg_bank_access = true; // 1 or true to access register 0x60 to 0x74

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config0_register(device, config0_reg), TAG, "write configuration 0 register for enable low registers failed" );

    return ESP_OK;
}

/**
 * @brief Writes SMUX command to AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] command SMUX command.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_smux_command(as7341_device_t *const device, const as7341_smux_commands_t command) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set smux command */
    as7341_config6_register_t config6_reg = { .bits.smux_command = command };

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config6_register(device, config6_reg), TAG, "write configuration 6 register for set smux command failed" );

    return ESP_OK;
}


/**
 * @brief Reads LED register (0x74) from AS7341.
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[out] reg LED register structure.
 * @return esp_err_t ESP_OK on success. 
 */
static inline esp_err_t as7341_i2c_get_led_register(as7341_device_t *const device, as7341_led_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to enable low register bank */
    as7341_i2c_enable_lo_register_bank(device);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(device, AS7341_LED, &reg->reg), TAG, "read LED register failed" );

    /* attempt to enable high register bank */
    as7341_i2c_enable_hi_register_bank(device);
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief Writes LED register (0x74) to AS7341
 * 
 * @param[in] device AS7341 device descriptor.
 * @param[in] reg LED register structure.
 * @return esp_err_t ESP_OK on success. 
 */
static inline esp_err_t as7341_i2c_set_led_register(as7341_device_t *const device, const as7341_led_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

     /* attempt to enable low register bank */
    as7341_i2c_enable_lo_register_bank(device);
    
    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(device, AS7341_LED, reg.reg), TAG, "write LED register failed" );

    /* attempt to enable high register bank */
    as7341_i2c_enable_hi_register_bank(device);
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));
    
    return ESP_OK;
}


/**
 * @brief Configures SMUX registers for low channels (F1-F4, Clear and NIR).
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_smux_lo_channels(as7341_device_t *const device) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register, for set SMUX low channels failed" );

    /* disable spectral measurement */
    enable.bits.spectral_measurement_enabled = false;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register, for set SMUX low channels failed" );

    //ESP_RETURN_ON_ERROR( as7341_disable_spectral_measurement(handle), TAG, "disable spectral measurement for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( as7341_i2c_set_smux_command(device, AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( as7341_i2c_setup_smux_lo_channels(device), TAG, "setup SMUX low channels for set SMUX low channels failed" );

    /* enable smux */
    enable.bits.smux_enabled = true;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register, for set SMUX low channels failed" );
    //ESP_RETURN_ON_ERROR( as7341_enable_smux(handle), TAG, "enable SMUX for set SMUX low channels failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for high channels (F5-F8, Clear and NIR).
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_set_smux_hi_channels(as7341_device_t *const device) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register, for set SMUX high channels failed" );

    /* disable spectral measurement */
    enable.bits.spectral_measurement_enabled = false;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register, for set SMUX high channels failed" );

    //ESP_RETURN_ON_ERROR( as7341_disable_spectral_measurement(handle), TAG, "disable spectral measurement for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( as7341_i2c_set_smux_command(device, AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( as7341_i2c_setup_smux_hi_channels(device), TAG, "setup SMUX high channels for set SMUX high channels failed" );

    /* enable smux */
    enable.bits.smux_enabled = true;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register, for set SMUX high channels failed" );
    //ESP_RETURN_ON_ERROR( as7341_enable_smux(handle), TAG, "enable SMUX for set SMUX high channels failed" );

    return ESP_OK;
}


/**
 * @brief Converts an ADC spectral gain sensitivity enumerator to a spectral gain sensitivity multiplier.
 * 
 * @param gain ADC spectral gain sensitivity enumerator.
 * @return float ADC spectral gain sensitivity multiplier.
 */
static inline float as7341_get_spectral_gain_sensitivity(as7341_spectral_gains_t gain) {
    /* determine gain sensitivity */
    switch (gain) {
        case AS7341_SPECTRAL_GAIN_0_5X:
            return 0.5f;
        case AS7341_SPECTRAL_GAIN_1X:
            return 1.0f;
        case AS7341_SPECTRAL_GAIN_2X:
            return 2.0f;
        case AS7341_SPECTRAL_GAIN_4X:
            return 4.0f;
        case AS7341_SPECTRAL_GAIN_8X:
            return 8.0f;
        case AS7341_SPECTRAL_GAIN_16X:
            return 16.0f;
        case AS7341_SPECTRAL_GAIN_32X:
            return 32.0f;
        case AS7341_SPECTRAL_GAIN_64X:
            return 64.0f;
        case AS7341_SPECTRAL_GAIN_128X:
            return 128.0f;
        case AS7341_SPECTRAL_GAIN_256X:
            return 256.0f;
        case AS7341_SPECTRAL_GAIN_512X:
            return 512.0f;
        default:
            return 1.0f;
    }
}

/**
 * @brief Calculated integration time (𝑡𝑖𝑛𝑡 = (𝐴𝑇𝐼𝑀𝐸 + 1) × (𝐴𝑆𝑇𝐸𝑃 + 1) × 2.78μ𝑠), from ATIME and ASTEP registers, in milli-seconds.
 * 
 * @param device AS7341 device descriptor.
 * @param time Integration time in milli-seconds
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_i2c_get_integration_time(as7341_device_t *const device, float *time) {
    uint8_t atime;
    uint16_t astep;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read astep and atime registers */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_astep_register(device, &astep), TAG, "read astep register for get integration time failed" );
    ESP_RETURN_ON_ERROR( as7341_i2c_get_atime_register(device, &atime), TAG, "read atime register for get integration time failed" );

    /* compute integration time */
    *time = (float)(atime + 1) * (astep + 1) * 2.78f / 1000.0f;

    return ESP_OK;
}

static inline esp_err_t as7341_i2c_setup_registers(as7341_device_t *const device) {
    as7341_part_id_register_t part_id;
    as7341_revision_id_register_t revision_id;
    as7341_enable_register_t enable;
    as7341_config1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register, for setup registers failed" );

    enable.bits.power_enabled = device->config.power_enabled;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register, for setup registers failed" );

    /* attempt to read part id */
    ESP_RETURN_ON_ERROR(as7341_i2c_get_part_id_register(device, &part_id), TAG, "read part id register for setup registers failed");

    /* attempt to read revision id */
    ESP_RETURN_ON_ERROR(as7341_i2c_get_revision_id_register(device, &revision_id), TAG, "read revision id register for setup registers failed");

    /* copy configuration */
    device->part_id = part_id.bits.identifier;
    device->revision_id = revision_id.bits.identifier;

    /* attempt to write atime configuration */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_atime_register(device, device->config.atime), TAG, "write atime for setup registers failed" );

    /* attempt to write astep configuration */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_astep_register(device, device->config.astep), TAG, "write astep for setup registers failed" );

    config1.bits.spectral_gain = device->config.spectral_gain;

    /* attempt to write configuration 1 register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config1_register(device, config1), TAG, "write configuration 1 register for setup registers failed" );

    return ESP_OK;
}

esp_err_t as7341_init(i2c_master_bus_handle_t master_handle, const as7341_config_t *as7341_config, as7341_handle_t *as7341_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && as7341_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, as7341_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, as7341 device handle initialization failed", as7341_config->i2c_address);

    /* validate memory availability for handle */
    as7341_device_t* device = (as7341_device_t*)calloc(1, sizeof(as7341_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as7341 device, init failed");

    /* copy configuration */
    device->config = *as7341_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    /* attempt to setup registers */
    ESP_GOTO_ON_ERROR(as7341_i2c_setup_registers(device), err_handle, TAG, "setup registers for init failed");

    /* set device handle */
    *as7341_handle = (as7341_handle_t)device;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t as7341_get_spectral_measurements(as7341_handle_t handle, as7341_channels_spectral_data_t *const spectral_data) {
    as7341_enable_register_t enable;
    esp_err_t   ret              = ESP_OK;
    float       integration_time = 0;
    uint64_t    start_time       = esp_timer_get_time();
    bool        data_is_ready    = false;
    uint8_t     rx[12]           = { 0 };
    as7341_device_t* device      = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_GOTO_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), err, TAG, "read enable register, for get adc measurements failed" );

    /* attempt to read integration time */
    ESP_GOTO_ON_ERROR( as7341_i2c_get_integration_time(device, &integration_time), err, TAG, "read integration time, for get adc measurements failed." );

    // ************ LOW CHANNELS ***********

    /* attempt to setup low channels */
    ESP_GOTO_ON_ERROR( as7341_i2c_set_smux_lo_channels(device), err, TAG, "setup of SMUX low channels for get adc measurements failed." );

    /* attempt to enable spectral measurement for low channels */
    enable.bits.spectral_measurement_enabled = true;
    //ESP_GOTO_ON_ERROR( as7341_enable_spectral_measurement(handle), err, TAG, "enable spectral measurement, low channels, for get adc measurements failed." );
    ESP_GOTO_ON_ERROR( as7341_i2c_set_enable_register(device, enable), err, TAG, "read enable register, for get adc measurements failed" );


    /* attempt to poll until data, low channels, is available or timeout */
    do {
        as7341_status2_register_t status2;

        /* attempt to check if data is ready */
        //ESP_GOTO_ON_ERROR( as7341_get_data_status(handle, &data_is_ready), err, TAG, "data ready read, low channels, for get adc measurements failed." );
        /* attempt to read device status register */
        ESP_GOTO_ON_ERROR( as7341_i2c_get_status2_register(device, &status2), err, TAG, "read status 2 register (data ready state), for get adc measurements failed" );

        data_is_ready = status2.bits.spectral_valid;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AS7341_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (integration_time + 50) * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read spectral adc data from low channels */
    ESP_GOTO_ON_ERROR( as7341_i2c_read_from(device, AS7341_CH0_ADC_DATA_L, rx, sizeof(rx)), err, TAG, "read low channel measurements for get adc measurements failed" );

    /* set adc data for low channels */
    spectral_data->f1 = (uint16_t)rx[0]  | (uint16_t)(rx[1] << 8);
    spectral_data->f2 = (uint16_t)rx[2]  | (uint16_t)(rx[3] << 8);
    spectral_data->f3 = (uint16_t)rx[4]  | (uint16_t)(rx[5] << 8);
    spectral_data->f4 = (uint16_t)rx[6]  | (uint16_t)(rx[7] << 8);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));


    // ************ HIGH CHANNELS ***********

    /* attempt to setup low channels */
    ESP_GOTO_ON_ERROR( as7341_i2c_set_smux_hi_channels(device), err, TAG, "setup of SMUX high channels for get adc measurements failed." );

    /* attempt to enable spectral measurement for low channels */
    enable.bits.spectral_measurement_enabled = true;
    //ESP_GOTO_ON_ERROR( as7341_enable_spectral_measurement(handle), err, TAG, "enable spectral measurement, high channels, for get adc measurements failed." );
    ESP_GOTO_ON_ERROR( as7341_i2c_set_enable_register(device, enable), err, TAG, "read enable register, for get adc measurements failed" );


    /* reset start time for timeout monitoring and reset data ready flag */
    start_time = esp_timer_get_time();
    data_is_ready = false;

    /* attempt to poll until data, high channels, is available or timeout */
    do {
        as7341_status2_register_t status2;

        /* attempt to check if data is ready */
        //ESP_GOTO_ON_ERROR( as7341_get_data_status(handle, &data_is_ready), err, TAG, "data ready read, low channels, for get adc measurements failed." );
        /* attempt to read device status register */
        ESP_GOTO_ON_ERROR( as7341_i2c_get_status2_register(device, &status2), err, TAG, "read status 2 register (data ready state), for get adc measurements failed" );

        data_is_ready = status2.bits.spectral_valid;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AS7341_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (integration_time + 50) * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read spectral adc data from high channels */
    ESP_GOTO_ON_ERROR( as7341_i2c_read_from(device, AS7341_CH0_ADC_DATA_L, rx, sizeof(rx)), err, TAG, "read high channel measurements for get adc measurements failed" );

    /* set adc data for high channels */
    spectral_data->f5    = (uint16_t)rx[0]  | (uint16_t)(rx[1] << 8);
    spectral_data->f6    = (uint16_t)rx[2]  | (uint16_t)(rx[3] << 8);
    spectral_data->f7    = (uint16_t)rx[4]  | (uint16_t)(rx[5] << 8);
    spectral_data->f8    = (uint16_t)rx[6]  | (uint16_t)(rx[7] << 8);
    spectral_data->clear = (uint16_t)rx[8]  | (uint16_t)(rx[9] << 8);
    spectral_data->nir   = (uint16_t)rx[10] | (uint16_t)(rx[11] << 8);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}

esp_err_t as7341_get_basic_counts(as7341_handle_t handle, const as7341_channels_spectral_data_t spectral_data, as7341_channels_basic_counts_data_t *const basic_counts_data) {
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    as7341_config1_register_t config1;

    /* attempt to read registers (config1) */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config1_register(device, &config1), TAG, "read configuration 1 register for get integration time failed" );

    /* determine gain sensitivity */
    float gain = as7341_get_spectral_gain_sensitivity(config1.bits.spectral_gain);

    /* compute integration time */
    float integration_time = 0;
    ESP_RETURN_ON_ERROR( as7341_i2c_get_integration_time(device, &integration_time), TAG, "read integration time failed" );

    /* convert adc value to basic counts value */
    basic_counts_data->f1       = (float)spectral_data.f1 / gain * integration_time;
    basic_counts_data->f2       = (float)spectral_data.f2 / gain * integration_time;
    basic_counts_data->f3       = (float)spectral_data.f3 / gain * integration_time;
    basic_counts_data->f4       = (float)spectral_data.f4 / gain * integration_time;
    basic_counts_data->f5       = (float)spectral_data.f5 / gain * integration_time;
    basic_counts_data->f6       = (float)spectral_data.f6 / gain * integration_time;
    basic_counts_data->f7       = (float)spectral_data.f7 / gain * integration_time;
    basic_counts_data->f8       = (float)spectral_data.f8 / gain * integration_time;
    basic_counts_data->clear    = (float)spectral_data.clear / gain * integration_time;
    basic_counts_data->nir      = (float)spectral_data.nir   / gain * integration_time;

    return ESP_OK;
}

esp_err_t as7341_get_flicker_detection_status(as7341_handle_t handle, as7341_flicker_detection_states_t *const state) {
    as7341_flicker_detection_status_register_t fd_status;
    as7341_enable_register_t enable;
    esp_err_t ret           = ESP_OK;
    uint64_t  start_time    = 0;
    bool      data_is_ready = false;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable power failed" );

    /* attempt to disable enable register */
    ESP_RETURN_ON_ERROR( as7341_i2c_disable_enable_register(device), TAG, "disable enable register, for get flicker detection status failed." );

    /* attempt to enable power, spectral, flicker */
    enable.bits.power_enabled = true;
    enable.bits.spectral_measurement_enabled = true;
    enable.bits.flicker_detection_enabled = true;
    enable.bits.smux_enabled = true;

    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for get flicker detection status failed" );

    /* attempt to write SMU configuration from RAM to set SMUX chain registers */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_smux_command(device, AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for get flicker detection status failed" );

    /* attempt to setup SMUX flicker detection */
    ESP_RETURN_ON_ERROR( as7341_i2c_setup_smux_flicker_detection(device), TAG, "setup SMUX for flicker detection, for get flicker detection status failed" );

    /* attempt to enable SMUX */
    //ESP_RETURN_ON_ERROR( as7341_enable_smux(handle), TAG, "enable SMUX, for get flicker detection status failed" );

    /* attempt to enable spectral measurement */
    //ESP_RETURN_ON_ERROR( as7341_enable_spectral_measurement(handle), TAG, "enable spectral measurement, for get flicker detection status failed." );

    /* attempt to enable flicker detection */
    //ESP_RETURN_ON_ERROR( as7341_enable_flicker_detection(handle), TAG, "enable flicker detection, for get flicker detection status failed" );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data, high channels, is available or timeout */
    do {

        /* attempt to check if flicker detection measurement is ready */
        ESP_GOTO_ON_ERROR( as7341_i2c_get_flicker_detection_status_register(device, &fd_status), err, TAG, "read flicker detection status register, for get flicker detection status failed" );

        /* if the measurement is valid or saturation is detected, set data ready flag to true */
        if(fd_status.bits.fd_measurement_valid == true || fd_status.bits.fd_saturation_detected == true) {
            data_is_ready = true;
        }

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(5));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (500) * 1000)) // start with 500 ms
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to disable flicker detection */
    //ESP_RETURN_ON_ERROR( as7341_disable_flicker_detection(handle), TAG, "disable flicker detection, for get flicker detection status failed" );
    enable.bits.flicker_detection_enabled = false;
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for get flicker detection status failed" );

    ESP_LOGW(TAG, "FD Status Register:  0x%02x (0b%s)", fd_status.reg, uint8_to_binary(fd_status.reg));

    /* set output parameter */
    if(fd_status.bits.fd_saturation_detected == true) {
        *state = AS7341_FLICKER_DETECTION_SATURATED;
    } else {
        if(fd_status.bits.fd_measurement_valid == true &&
            fd_status.bits.fd_100hz_flicker_valid == false &&
            fd_status.bits.fd_120hz_flicker_valid == false) {
            *state = AS7341_FLICKER_DETECTION_UNKNOWN;
        } else {
            if(fd_status.bits.fd_100hz_flicker_valid == true) {
                *state = AS7341_FLICKER_DETECTION_100HZ;
            } else if(fd_status.bits.fd_120hz_flicker_valid == true) {
                *state = AS7341_FLICKER_DETECTION_120HZ;
            } else {
                *state = AS7341_FLICKER_DETECTION_INVALID;
            }
        }
    }

    return ESP_OK;

    err:
        return ret;
}

esp_err_t as7341_get_data_status(as7341_handle_t handle, bool *const ready) {
    as7341_status2_register_t status2;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_status2_register(device, &status2), TAG, "read status 2 register (data ready state) failed" );

    /* set ready state */
    *ready = status2.bits.spectral_valid;

    return ESP_OK;
}

esp_err_t as7341_get_atime(as7341_handle_t handle, uint8_t *const atime) {
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_atime_register(device, atime), TAG, "read atime register for get atime failed" );

    return ESP_OK;
}

esp_err_t as7341_set_atime(as7341_handle_t handle, const uint8_t atime) {
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_atime_register(device, atime), TAG, "write atime register for set atime failed" );

    return ESP_OK;
}

esp_err_t as7341_get_astep(as7341_handle_t handle, uint16_t *const astep) {
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_astep_register(device, astep), TAG, "read atime register for get astep failed" );

    return ESP_OK;
}

esp_err_t as7341_set_astep(as7341_handle_t handle, const uint16_t astep) {
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_astep_register(device, astep), TAG, "write astep register for set astep failed" );

    return ESP_OK;
}

esp_err_t as7341_get_spectral_gain(as7341_handle_t handle, as7341_spectral_gains_t *const gain) {
    as7341_config1_register_t config1;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config1_register(device, &config1), TAG, "read configuration 1 register for get spectral gain failed" );

    /* set output parameter */
    *gain = config1.bits.spectral_gain;

    return ESP_OK;
}

esp_err_t as7341_set_spectral_gain(as7341_handle_t handle, const as7341_spectral_gains_t gain) {
    as7341_config1_register_t config1;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config1_register(device, &config1), TAG, "read configuration 1 register for get spectral gain failed" );

    /* set spectral gain */
    config1.bits.spectral_gain = gain;

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config1_register(device, config1), TAG, "write configuration 1 register for set spectral gain failed" );

    return ESP_OK;
}

esp_err_t as7341_get_ambient_light_sensing_mode(as7341_handle_t handle, as7341_als_modes_t *const mode) {
    as7341_config_register_t config;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_lo_register_bank(device), TAG, "enable low register bank for get ambient light sensing mode failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config_register(device, &config), TAG, "read configuration register for get ambient light sensing mode failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_hi_register_bank(device), TAG, "enable high register bank for get ambient light sensing mode failed" );

    /* set output parameter */
    *mode = config.bits.irq_mode;

    return ESP_OK;
}

esp_err_t as7341_set_ambient_light_sensing_mode(as7341_handle_t handle, const as7341_als_modes_t mode) {
    as7341_config_register_t config;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_lo_register_bank(device), TAG, "enable low register bank for set ambient light sensing mode failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config_register(device, &config), TAG, "read configuration register for set ambient light sensing mode failed" );

    /* set mode */
    config.bits.irq_mode = mode;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config_register(device, config), TAG, "write configuration register for set ambient light sensing mode failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_hi_register_bank(device), TAG, "enable high register bank for set ambient light sensing mode failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_flicker_detection(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable flicker detection failed" );

    /* enable flicker detection */
    enable.bits.flicker_detection_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for enable flicker detection failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_flicker_detection(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable flicker detection failed" );

    /* disable flicker detection */
    enable.bits.flicker_detection_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for disable flicker detection failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_smux(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable SMUX failed" );

    /* enable smux */
    enable.bits.smux_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for enable SMUX failed" );

    /* validate SMUX operation completed */
    uint16_t timeout = 1000;
    for (uint16_t time = 0; time < timeout; time++) {
        // The SMUXEN bit is cleared once the SMUX operation is finished
        ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable SMUX failed" );

        if (!enable.bits.smux_enabled) {
            return ESP_OK;
        }

        /* delay before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));
    }

    return ESP_ERR_INVALID_STATE;
}

esp_err_t as7341_enable_spectral_measurement(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable spectral measurement failed" );

    /* enable spectral measurement */
    enable.bits.spectral_measurement_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_spectral_measurement(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable spectral measurement failed" );

    /* disable spectral measurement */
    enable.bits.spectral_measurement_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_power(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable power failed" );

    /* enable power */
    enable.bits.power_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for enable power failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_power(as7341_handle_t handle) {
    as7341_enable_register_t enable;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable power failed" );

    /* disable power */
    enable.bits.power_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_enable_register(device, enable), TAG, "write enable register for disable power failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_led(as7341_handle_t handle) {
    as7341_config_register_t config;
    as7341_led_register_t    led;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_lo_register_bank(device), TAG, "enable low register bank for enable LED failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config_register(device, &config), TAG, "read configuration register for enable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_i2c_get_led_register(device, &led), TAG, "read led register for enable LED failed" );

    /* enable led */
    config.bits.led_ldr_control_enabled = true;
    led.bits.led_ldr_enabled            = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config_register(device, config), TAG, "write configuration register for enable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_i2c_set_led_register(device, led), TAG, "write led register for enable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_hi_register_bank(device), TAG, "enable high register bank for enable LED failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_led(as7341_handle_t handle) {
    as7341_config_register_t config;
    as7341_led_register_t    led;
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_lo_register_bank(device), TAG, "enable low register bank for disable LED failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_get_config_register(device, &config), TAG, "read configuration register for disable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_i2c_get_led_register(device, &led), TAG, "read led register for disable LED failed" );

    /* enable led */
    config.bits.led_ldr_control_enabled = false;
    led.bits.led_ldr_enabled            = false;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_i2c_set_config_register(device, config), TAG, "write configuration register for disable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_i2c_set_led_register(device, led), TAG, "write led register for disable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_i2c_enable_hi_register_bank(device), TAG, "enable high register bank for disable LED failed" );

    return ESP_OK;
}

esp_err_t as7341_remove(as7341_handle_t handle) {
    as7341_device_t* device = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(device->i2c_handle);
}

esp_err_t as7341_delete(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( as7341_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* as7341_get_fw_version(void) {
    return (const char*)AS7341_FW_VERSION_STR;
}

int32_t as7341_get_fw_version_number(void) {
    return (int32_t)AS7341_FW_VERSION_INT32;
}