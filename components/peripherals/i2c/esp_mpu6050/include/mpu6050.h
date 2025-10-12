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
 * @file mpu6050.h
 * @defgroup drivers mpu6050
 * @{
 *
 * ESP-IDF driver for mpu6050 motion-tracking sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include "mpu6050_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * MPU6050 definitions
*/
#define I2C_MPU6050_DEV_CLK_SPD                 UINT32_C(100000)        //!< mpu6050 I2C default clock frequency (100KHz)

#define I2C_MPU6050_DEV_ADDR_H                  UINT8_C(0x69)   //!< mpu6050 I2C address when AD0 = 1 or to vcc
#define I2C_MPU6050_DEV_ADDR_L                  UINT8_C(0x68)   //!< mpu6050 I2C address when AD0 = 0 or to gnd


/*
 * MPU6050 macro definitions
*/
#define I2C_MPU6050_CONFIG_DEFAULT {                        \
    .i2c_address                = I2C_MPU6050_DEV_ADDR_L,        \
    .i2c_clock_speed            = I2C_MPU6050_DEV_CLK_SPD,             \
    .low_pass_filter            = MPU6050_DIGITAL_LP_FILTER_ACCEL_260KHZ_GYRO_256KHZ,    \
    .gyro_clock_source          = MPU6050_GYRO_CS_PLL_X_AXIS_REF,   \
    .gyro_full_scale_range      = MPU6050_GYRO_FS_RANGE_500DPS,     \
    .accel_full_scale_range     = MPU6050_ACCEL_FS_RANGE_4G }


/*
 * MPU6050 enumerator and structure declarations
*/

/**
 * @brief MPU6050 external synchronization settings enumerator.
 */
typedef enum mpu6050_ext_sync_settings_e {
    MPU6050_EXT_SYNC_SETTING_INPUT_DISABLED = (0b000),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_TEMP_OUT_L     = (0b001),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_GYRO_XOUT_L    = (0b010),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_GYRO_YOUT_L    = (0b011),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_GYRO_ZOUT_L    = (0b100),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_ACCEL_XOUT_L   = (0b101),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_ACCEL_YOUT_L   = (0b110),      /*!< mpu6050  */
    MPU6050_EXT_SYNC_SETTING_ACCEL_ZOUT_L   = (0b111),      /*!< mpu6050  */
} mpu6050_ext_sync_settings_t;

/**
 * @brief MPU6050 digital low-pass filters enumerator.
 */
typedef enum mpu6050_digital_low_pass_filters_e {
    MPU6050_DIGITAL_LP_FILTER_ACCEL_260KHZ_GYRO_256KHZ  = (0b000),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_ACCEL_184KHZ_GYRO_188KHZ  = (0b001),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_ACCEL_94KHZ_GYRO_98KHZ    = (0b010),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_ACCEL_44KHZ_GYRO_42KHZ    = (0b011),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_ACCEL_21KHZ_GYRO_20KHZ    = (0b100),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_ACCEL_10KHZ_GYRO_10KHZ    = (0b101),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_ACCEL_5KHZ_GYRO_5KHZ      = (0b110),      /*!< mpu6050  */
    MPU6050_DIGITAL_LP_FILTER_RESERVED                  = (0b111),      /*!< mpu6050 reserved but disables low-pass filter */
} mpu6050_digital_low_pass_filters_t;

/**
 * @brief MPU6050 gyroscope full-scale ranges enumerator.
 */
typedef enum mpu6050_gyro_full_scale_ranges_e {
    MPU6050_GYRO_FS_RANGE_250DPS    = (0b00),  /*!< mpu6050 gyroscope full-scale range ± 250 °/s */
    MPU6050_GYRO_FS_RANGE_500DPS    = (0b01),  /*!< mpu6050 gyroscope full-scale range ± 500 °/s */
    MPU6050_GYRO_FS_RANGE_1000DPS   = (0b10),  /*!< mpu6050 gyroscope full-scale range ± 1000 °/s */
    MPU6050_GYRO_FS_RANGE_2000DPS   = (0b11)   /*!< mpu6050 gyroscope full-scale range ± 2000 °/s */
} mpu6050_gyro_full_scale_ranges_t;

/**
 * @brief MPU6050 accelerometer full-scale ranges enumerator.
 */
typedef enum mpu6050_accel_full_scale_ranges_e {
    MPU6050_ACCEL_FS_RANGE_2G       = (0b00),  /*!< mpu6050 accelerometer full-scale range ± 2g */
    MPU6050_ACCEL_FS_RANGE_4G       = (0b01),  /*!< mpu6050 accelerometer full-scale range ± 4g */
    MPU6050_ACCEL_FS_RANGE_8G       = (0b10),  /*!< mpu6050 accelerometer full-scale range ± 8g */
    MPU6050_ACCEL_FS_RANGE_16G      = (0b11)   /*!< mpu6050 accelerometer full-scale range ± 16g */
} mpu6050_accel_full_scale_ranges_t;

/**
 * @brief MPU6050 gyroscope clock sources enumerator.
 */
typedef enum mpu6050_gyro_clock_sources_e {
    MPU6050_GYRO_CS_INT_8MHZ                = (0b000),  /*!< mpu6050 internal 8MHz oscillator */
    MPU6050_GYRO_CS_PLL_X_AXIS_REF          = (0b001),  /*!< mpu6050 PLL with x-axis gyroscope reference */
    MPU6050_GYRO_CS_PLL_Y_AXIS_REF          = (0b010),  /*!< mpu6050 PLL with y-axis gyroscope reference */
    MPU6050_GYRO_CS_PLL_Z_AXIS_REF          = (0b011),  /*!< mpu6050 PLL with z-axis gyroscope reference */
    MPU6050_GYRO_CS_PLL_EXT_32_768KHZ_REF   = (0b100),  /*!< mpu6050 PLL with external 32.768kHz reference */
    MPU6050_GYRO_CS_PLL_EXT_19_2MHZ_REF     = (0b101),  /*!< mpu6050 PLL with external 19.2MHz reference */
    MPU6050_GYRO_CS_RESERVED                = (0b110),  /*!< mpu6050 reserved */
    MPU6050_GYRO_CS_CLOCK_STOP_IN_RESET     = (0b111),  /*!< mpu6050 stops the clock an keeps the timing generator in reset */
} mpu6050_gyro_clock_sources_t;

/**
 * @brief MPU6050 low-power wake controls enumerator.
 */
typedef enum mpu6050_low_power_wake_controls_e {
    MPU6050_LP_WAKE_CONTROL_1_25HZ  = (0b00),  /*!< mpu6050 1.25Hz wake-up frequency */
    MPU6050_LP_WAKE_CONTROL_5HZ     = (0b01),  /*!< mpu6050 5Hz wake-up frequency */
    MPU6050_LP_WAKE_CONTROL_20HZ    = (0b10),  /*!< mpu6050 20Hz wake-up frequency */
    MPU6050_LP_WAKE_CONTROL_40HZ    = (0b11),  /*!< mpu6050 40Hz wake-up frequency */
} mpu6050_low_power_wake_controls_t;

typedef enum mpu6050_irq_pin_active_level_e {
    MPU6050_IRQ_PIN_ACTIVE_HIGH = 0,          /*!< The mpu6050 sets its INT pin HIGH on interrupt */
    MPU6050_IRQ_PIN_ACTIVE_LOW  = 1           /*!< The mpu6050 sets its INT pin LOW on interrupt */
} mpu6050_irq_pin_active_level_t;

typedef enum mpu6050_irq_pin_mode_e {
    MPU6050_IRQ_PIN_PUSH_PULL   = 0,          /*!< The mpu6050 configures its INT pin as push-pull */
    MPU6050_IRQ_PIN_OPEN_DRAIN  = 1           /*!< The mpu6050 configures its INT pin as open drain*/
} mpu6050_irq_pin_mode_t;

typedef enum mpu6050_irq_latch_e {
    MPU6050_IRQ_LATCH_50US            = 0,    /*!< The mpu6050 produces a 50 microsecond pulse on interrupt */
    MPU6050_IRQ_LATCH_UNTIL_CLEARED   = 1     /*!< The mpu6050 latches its INT pin to its active level, until interrupt is cleared */
} mpu6050_irq_latch_t;

typedef enum mpu6050_irq_clear_e {
    MPU6050_IRQ_CLEAR_ON_ANY_READ     = 0,    /*!< INT_STATUS register bits are cleared on any register read */
    MPU6050_IRQ_CLEAR_ON_STATUS_READ  = 1     /*!< INT_STATUS register bits are cleared only by reading INT_STATUS value*/
} mpu6050_irq_clear_t;

/**
 * @brief MPU6050 self-test register (self-test x, y, z) structure.
 */
typedef union __attribute__((packed)) mpu6050_self_test_register_u {
    struct {
        uint8_t     g_test:5;  /*!< mpu6050   (bit:0-4) */
        uint8_t     a_test:3;  /*!< mpu6050   (bit:5-7) */
    } bits;
    uint8_t reg;
} mpu6050_self_test_register_t;

/**
 * @brief MPU6050 self-test a register structure.
 */
typedef union __attribute__((packed)) mpu6050_self_test_a_register_u {
    struct {
        uint8_t     za_test:2;   /*!< mpu6050   (bit:0-1) */
        uint8_t     ya_test:2;   /*!< mpu6050   (bit:2-3) */
        uint8_t     xa_test:2;   /*!< mpu6050   (bit:4-5) */
        uint8_t     reserved:2;  /*!< reserved and set to 0 (bit:6-7) */
    } bits;
    uint8_t reg;
} mpu6050_self_test_a_register_t;

/**
 * @brief MPU6050 configuration register structure.
 */
typedef union __attribute__((packed)) mpu6050_config_register_u {
    struct {
        mpu6050_digital_low_pass_filters_t  low_pass_filter:3;      /*!< mpu6050 digital low-pass filter configuration (bit:0-2) */
        mpu6050_ext_sync_settings_t         ext_sync_setting:3;     /*!< mpu6050 external synchronization setting      (bit:3-5) */
        uint8_t                             reserved:2;             /*!< mpu6050 reserved and set to 0                 (bit:6-7) */
    } bits;
    uint8_t reg;
} mpu6050_config_register_t;

/**
 * @brief MPU6050 gyroscope configuration register structure.
 */
typedef union __attribute__((packed)) mpu6050_gyro_config_register_u {
    struct {
        uint8_t                             reserved:3;                 /*!< mpu6050 reserved and set to 0                            (bit:0-2) */
        mpu6050_gyro_full_scale_ranges_t    full_scale_range:2;         /*!< mpu6050 gyroscope full scale range                        (bit:3-4) */
        bool                                z_axis_selftest_enabled:1;  /*!< mpu6050 z-axis gyroscope performs self-test when enabled (bit:5) */
        bool                                y_axis_selftest_enabled:1;  /*!< mpu6050 y-axis gyroscope performs self-test when enabled (bit:6) */
        bool                                x_axis_selftest_enabled:1;  /*!< mpu6050 x-axis gyroscope performs self-test when enabled (bit:7) */
    } bits;
    uint8_t reg;
} mpu6050_gyro_config_register_t;

/**
 * @brief MPU6050 accelerometer configuration register structure.
 */
typedef union __attribute__((packed)) mpu6050_accel_config_register_u {
    struct {
        uint8_t                             reserved:3;                 /*!< mpu6050 reserved and set to 0                                (bit:0-2) */
        mpu6050_accel_full_scale_ranges_t   full_scale_range:2;         /*!< mpu6050 accelerometer full scale range                       (bit:3-4) */
        bool                                z_axis_selftest_enabled:1;  /*!< mpu6050 z-axis accelerometer performs self-test when enabled (bit:5) */
        bool                                y_axis_selftest_enabled:1;  /*!< mpu6050 y-axis accelerometer performs self-test when enabled (bit:6) */
        bool                                x_axis_selftest_enabled:1;  /*!< mpu6050 x-axis accelerometer performs self-test when enabled (bit:7) */
    } bits;
    uint8_t reg;
} mpu6050_accel_config_register_t;

/**
 * @brief MPU6050 FIFO enable register structure.
 */
typedef union __attribute__((packed)) mpu6050_fifo_enable_register_u {
    struct {
        bool                                slave_0_fifo_enabled:1;     /*!< mpu6050 enables slave 0 to be written into the FIFO buffer (bit:0) */
        bool                                slave_1_fifo_enabled:1;     /*!< mpu6050 enables slave 1 to be written into the FIFO buffer (bit:1) */
        bool                                slave_2_fifo_enabled:1;     /*!< mpu6050 enables slave 2 to be written into the FIFO buffer (bit:2) */
        bool                                accel_fifo_enabled:1;       /*!< mpu6050 enables accelerometer (x, y, z) to be written into the FIFO buffer (bit:3) */
        bool                                gyro_z_fifo_enabled:1;      /*!< mpu6050 enables gyroscope z-axis to be written into the FIFO buffer  (bit:4) */
        bool                                gyro_y_fifo_enabled:1;      /*!< mpu6050 enables gyroscope y-axis to be written into the FIFO buffer  (bit:5) */
        bool                                gyro_x_fifo_enabled:1;      /*!< mpu6050 enables gyroscope x-axis to be written into the FIFO buffer (bit:6) */
        bool                                temp_fifo_enabled:1;        /*!< mpu6050 enables temperature to be written into the FIFO buffer (bit:7) */
    } bits;
    uint8_t reg;
} mpu6050_fifo_enable_register_t;

/**
 * @brief MPU6050 interrupt enable register structure.
 */
typedef union __attribute__((packed)) mpu6050_interrupt_enable_register_u {
    struct {
        bool                                data_ready_enabled:1;       /*!< mpu6050 enables data ready interrupt       (bit:0) */
        uint8_t                             reserved1:2;                /*!< mpu6050 reserved and set to 0              (bit:1-2) */
        bool                                i2c_master_enabled:1;       /*!< mpu6050 enables i2c master interrupt       (bit:3) */
        bool                                fifo_overflow_enabled:1;    /*!< mpu6050 enabled fifo overflow interrupt    (bit:4) */
        uint8_t                             reserved2:3;                /*!< mpu6050 reserved and set to 0              (bit:5-7) */
    } bits;
    uint8_t reg;
} mpu6050_interrupt_enable_register_t;

/**
 * @brief MPU6050 interrupt pin configuration register structure.
 */
typedef union __attribute__((packed)) mpu6050_interrupt_pin_config_register_u {
    struct {
        uint8_t                             reserved1:1;  /*!< reserved and set to 0                     (bit:0)   */
        bool                                i2c_bypass_enabled:1;   /*!<  (bit:1) */
        bool                                fsynch_irq_enabled:1;   /*!<  (bit:2) */
        bool                                fsynch_irq_level:1;     /*!<  (bit:3) */
        mpu6050_irq_clear_t                 irq_read_clear:1;       /*!<  (bit:4) */
        mpu6050_irq_latch_t                 irq_latch:1;            /*!<  (bit:5) */
        mpu6050_irq_pin_mode_t              irq_level_mode:1;       /*!<  (bit:6) */
        mpu6050_irq_pin_active_level_t      irq_active_level:1;     /*!<  (bit:7) */
    } bits;
    uint8_t reg;
} mpu6050_interrupt_pin_config_register_t;

/**
 * @brief MPU6050 interrupt status register structure.
 * 
 * @note Each bit will clear after the register is read.
 * 
 */
typedef union __attribute__((packed)) mpu6050_interrupt_status_register_u {
    struct {
        bool                                irq_data_ready:1;       /*!< mpu6050 data is ready true when interrupt is generated (bit:0) */
        uint8_t                             reserved1:2;            /*!< mpu6050 reserved and set to 0                          (bit:1-2) */
        bool                                irq_i2c_master:1;       /*!< mpu6050 i2c master interrupt generated when true       (bit:3) */
        bool                                irq_fifo_overflow:1;    /*!< mpu6050 fifo overflow interrupt generated when true    (bit:4) */
        uint8_t                             reserved2:3;            /*!< mpu6050 reserved and set to 0                          (bit:5-7) */
    } bits;
    uint8_t reg;
} mpu6050_interrupt_status_register_t;

/**
 * @brief MPU6050 signal path reset register structure.
 */
typedef union __attribute__((packed)) mpu6050_signal_path_reset_register_u {
    struct {
        bool                                temperature_reset:1;    /*!< mpu6050 reset temperature analog and digital signal paths when true   (bit:0) */
        bool                                accelerometer_reset:1;  /*!< mpu6050 reset accelerometer analog and digital signal paths when true (bit:1) */
        bool                                gyroscope_reset:1;      /*!< mpu6050 reset gyroscope analog and digital signal paths when true     (bit:2) */
        uint8_t                             reserved:5;             /*!< mpu6050 reserved and set to 0                                         (bit:3-7) */
    } bits;
    uint8_t reg;
} mpu6050_signal_path_reset_register_t;

/**
 * @brief MPU6050 user control register structure.
 */
typedef union __attribute__((packed)) mpu6050_user_control_register_u {
    struct {
        bool                                signal_condition_reset:1;           /*!< mpu6050 resets signal paths for all sensors when true      (bit:0) */
        bool                                i2c_master_reset:1;                 /*!< mpu6050 resets i2c master when true                        (bit:1) */
        bool                                fifo_reset:1;                       /*!< mpu6050 resets fifo buffer when true                       (bit:2) */
        uint8_t                             reserved1:1;                        /*!< mpu6050 reserved and set to 0                              (bit:3) */
        bool                                i2c_master_interface_disabled:1;    /*!< mpu6050 disables i2c interface and enables spi interface   (bit:4) */
        bool                                i2c_master_enabled:1;               /*!< mpu6050 enables i2c master mode                            (bit:5) */
        bool                                fifo_enabled:1;                     /*!< mpu6050 enables fifo operation                             (bit:6) */
        uint8_t                             reserved2:1;                        /*!< mpu6050 reserved and set to 0                              (bit:7) */
    } bits;
    uint8_t reg;
} mpu6050_user_control_register_t;

/**
 * @brief MPU6050 power management 1 register structure.
 */
typedef union __attribute__((packed)) mpu6050_power_management1_register_u {
    struct {
        mpu6050_gyro_clock_sources_t        clock_source:3;     /*!< mpu6050 device clock source                                            (bit:0-2) */
        bool                                temp_disabled:1;    /*!< mpu6050 disables temperature sensor when true                          (bit:3) */
        uint8_t                             reserved:1;         /*!< mpu6050 reserved and set to 0                                          (bit:4) */
        bool                                cycle_enabled:1;    /*!< mpu6050 will cycle between sleep mode to waking up when enabled to take a single smaple of data from active sensors (bit:5) */
        bool                                sleep_enabled:1;    /*!< mpu6050 puts the device into sleep mode when enabled                   (bit:6) */
        bool                                reset_enabled:1;    /*!< mpu6050 resets all internal registers to default values when enabled   (bit:7) */
    } bits;
    uint8_t reg;
} mpu6050_power_management1_register_t;

/**
 * @brief MPU6050 power management 2 register structure.
 */
typedef union __attribute__((packed)) mpu6050_power_management2_register_u {
    struct {
        bool                                gyro_z_axis_standby:1;  /*!< mpu6050 gyroscope z-axis in stand-by mode when true        (bit:0) */
        bool                                gyro_y_axis_standby:1;  /*!< mpu6050 gyroscope y-axis in stand-by mode when true        (bit:1) */
        bool                                gyro_x_axis_standby:1;  /*!< mpu6050 gyroscope x-axis in stand-by mode when true        (bit:2) */
        bool                                accel_z_axis_standby:1; /*!< mpu6050 accelerometer z-axis in stand-by mode when true    (bit:3) */
        bool                                accel_y_axis_standby:1; /*!< mpu6050 accelerometer y-axis in stand-by mode when true    (bit:4) */
        bool                                accel_x_axis_standby:1; /*!< mpu6050 accelerometer x-axis in stand-by mode when true    (bit:5) */
        mpu6050_low_power_wake_controls_t   wakeup_control:2;       /*!< mpu6050 frequency of wake-ups in low-power mode            (bit:6-7)*/
    } bits;
    uint8_t reg;
} mpu6050_power_management2_register_t;

/**
 * @brief MPU6050 who am i or device identifier register structure.
 */
typedef union __attribute__((packed)) mpu6050_who_am_i_register_u {
    struct {
        uint8_t     reserved1:1;  /*!< reserved and set to 0                     (bit:0)   */
        uint8_t     who_am_i:6;   /*!< mpu6050 device identifier (default: 0x68) (bit:1-6) */
        uint8_t     reserved2:1;  /*!< reserved and set to 0                     (bit:7)   */
    } bits;
    uint8_t reg;
} mpu6050_who_am_i_register_t;


/**
 * @brief MPU6050 raw data axes structure.
 */
typedef struct mpu6050_data_axes_s {
    int16_t x_axis;    /*!< mpu6050 x-axis raw gyroscope or accelerometer measurement */
    int16_t y_axis;    /*!< mpu6050 y-axis raw gyroscope or accelerometer measurement */
    int16_t z_axis;    /*!< mpu6050 z-axis raw gyroscope or accelerometer measurement */
} mpu6050_data_axes_t;

typedef struct mpu6050_gyro_data_axes_s {
    float x_axis;    /*!< mpu6050 x-axis gyroscope measurement in degrees per second (±250, ±500, ±1000, ±2000°/sec) */
    float y_axis;    /*!< mpu6050 y-axis gyroscope measurement in degrees per second (±250, ±500, ±1000, ±2000°/sec) */
    float z_axis;    /*!< mpu6050 z-axis gyroscope measurement in degrees per second (±250, ±500, ±1000, ±2000°/sec) */
} mpu6050_gyro_data_axes_t;

typedef struct mpu6050_accel_data_axes_s {
    float x_axis;    /*!< mpu6050 x-axis accelerometer measurement relative to standard gravity (±2g, ±4g, ±8g, ±16g) */
    float y_axis;    /*!< mpu6050 y-axis accelerometer measurement relative to standard gravity (±2g, ±4g, ±8g, ±16g) */
    float z_axis;    /*!< mpu6050 z-axis accelerometer measurement relative to standard gravity (±2g, ±4g, ±8g, ±16g) */
} mpu6050_accel_data_axes_t;

typedef struct mpu6050_attitude_s {
	float x;
	float y;
	float z;
	float w;
	float roll;
	float pitch;
	float yaw;
} mpu6050_attitude_t;

/**
 * @brief MPU6050 configuration structure definition.
 */
typedef struct mpu6050_config_s {
    uint16_t                            i2c_address;            /*!< mpu6050 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< mpu6050 i2c device scl clock speed in hz */
    mpu6050_digital_low_pass_filters_t  low_pass_filter;        /*!< mpu6050 digital low-pass filter configuration */
    mpu6050_gyro_clock_sources_t        gyro_clock_source;      /*!< mpu6050 gyroscope clock source configuration */
    mpu6050_gyro_full_scale_ranges_t    gyro_full_scale_range;  /*!< mpu6050 gyroscope full scale range configuration */
    mpu6050_accel_full_scale_ranges_t   accel_full_scale_range; /*!< mpu6050 accelerometer full scale range configuration */
    gpio_num_t                          irq_io_num;             /*!< mpu6050 GPIO interrupt pin  */
    mpu6050_irq_pin_active_level_t      irq_io_active_level;    /*!< Active level of mpu6050 INT pin         */
    mpu6050_irq_pin_mode_t              irq_io_mode;            /*!< Push-pull or open drain mode for INT pin*/
    mpu6050_irq_latch_t                 irq_latch;              /*!< The interrupt pulse behavior of INT pin */
    mpu6050_irq_clear_t                 irq_clear_behavior;     /*!< Interrupt status clear behavior         */
} mpu6050_config_t;

/**
 * @brief MPU6050 context structure.
 */
struct mpu6050_context_t {
    mpu6050_config_t                    dev_config;         /*!< mpu6050 device configuration */
    i2c_master_dev_handle_t             i2c_handle;         /*!< mpu6050 I2C device handle */
    float                               accel_sensitivity;  /*!< mpu6050 accelerometer sensitivity value */
    float                               gyro_sensitivity;   /*!< mpu6050 gyroscope sensitivity value */
};

/**
 * @brief MPU6050 context structure definition.
 */
typedef struct mpu6050_context_t mpu6050_context_t;

/**
 * @brief MPU6050 handle structure definition.
 */
typedef struct mpu6050_context_t *mpu6050_handle_t;

/**
 * @brief MPU6050 interrupt service routine definition.
 */
typedef gpio_isr_t mpu6050_isr_t;



/**
 * @brief Reads sample rate divider register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 sample rate divider value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_sample_rate_divider_register(mpu6050_handle_t handle, uint8_t *const reg);

/**
 * @brief Writes sample rate divider register to MPU6050.
 * 
 * @note The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:  
 *              Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) 
 *       where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), 
 *       and 1kHz when the DLPF is enabled (see Register 26). 
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 sample rate divider value.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_sample_rate_divider_register(mpu6050_handle_t handle, const uint8_t reg);

/**
 * @brief Reads configuration register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_config_register(mpu6050_handle_t handle, mpu6050_config_register_t *const reg);

/**
 * @brief Writes configuration register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_config_register(mpu6050_handle_t handle, const mpu6050_config_register_t reg);

/**
 * @brief Reads gyroscope configuration register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 gyroscope configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_gyro_config_register(mpu6050_handle_t handle, mpu6050_gyro_config_register_t *const reg);

/**
 * @brief Writes gyroscope configuration register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 gyroscope configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_gyro_config_register(mpu6050_handle_t handle, const mpu6050_gyro_config_register_t reg);

/**
 * @brief Reads accelerometer configuration register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 accelerometer configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_accel_config_register(mpu6050_handle_t handle, mpu6050_accel_config_register_t *const reg);

/**
 * @brief Writes accelerometer configuration register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 accelerometer configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_accel_config_register(mpu6050_handle_t handle, const mpu6050_accel_config_register_t reg);
 
/**
 * @brief Reads interrupt enable register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 interrupt enable register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_interrupt_enable_register(mpu6050_handle_t handle, mpu6050_interrupt_enable_register_t *const reg);

/**
 * @brief Writes interrupt enable register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 interrupt enable register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_interrupt_enable_register(mpu6050_handle_t handle, const mpu6050_interrupt_enable_register_t reg);

/**
 * @brief Reads interrupt pin configuration register from MPU6050.
 * 
 * @param handle MPU6050 device handle.
 * @param[out] reg MPU6050 interrupt pin configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_interrupt_pin_config_register(mpu6050_handle_t handle, mpu6050_interrupt_pin_config_register_t *const reg);

/**
 * @brief Writes interrupt pin configuration register to MPU6050.
 * 
 * @param handle MPU6050 device handle.
 * @param reg MPU6050 interrupt pin configuration register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_interrupt_pin_config_register(mpu6050_handle_t handle, const mpu6050_interrupt_pin_config_register_t reg);

/**
 * @brief Reads interrupt status register from MPU6050.
 * 
 * @note Interrupt status bits clear when read.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 interrupt status register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_interrupt_status_register(mpu6050_handle_t handle, mpu6050_interrupt_status_register_t *const reg);

/**
 * @brief Reads signal path reset register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 signal path reset register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_signal_path_reset_register(mpu6050_handle_t handle, mpu6050_signal_path_reset_register_t *const reg);

/**
 * @brief Writes signal path reset register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 signal path reset register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_signal_path_reset_register(mpu6050_handle_t handle, const mpu6050_signal_path_reset_register_t reg);

/**
 * @brief Reads user control register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 user control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_user_control_register(mpu6050_handle_t handle, mpu6050_user_control_register_t *const reg);

/**
 * @brief Writes user control register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 user control register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_user_control_register(mpu6050_handle_t handle, const mpu6050_user_control_register_t reg);

/**
 * @brief Reads power management 1 register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 power management 1 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_power_management1_register(mpu6050_handle_t handle, mpu6050_power_management1_register_t *const reg);

/**
 * @brief Writes power management 1 register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 power management 1 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_power_management1_register(mpu6050_handle_t handle, const mpu6050_power_management1_register_t reg);

/**
 * @brief Reads power management 2 register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 power management 2 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_power_management2_register(mpu6050_handle_t handle, mpu6050_power_management2_register_t *const reg);

/**
 * @brief Writes power management 2 register to MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[in] reg MPU6050 power management 2 register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_set_power_management2_register(mpu6050_handle_t handle, const mpu6050_power_management2_register_t reg);

/**
 * @brief Reads who am i (i.e. device identifier) register from MPU6050.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] reg MPU6050 who am i register.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_who_am_i_register(mpu6050_handle_t handle, mpu6050_who_am_i_register_t *const reg);

/**
 * @brief Configures interrupt pin behavior and setup target GPIO.
 * 
 * @param handle MPU6050 device handle.
 * @param config MPU6050 device configuration.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_configure_interrupts(mpu6050_handle_t handle, const mpu6050_config_t *const config);

/**
 * @brief Initializes an MPU6050 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] mpu6050_config MPU6050 device configuration.
 * @param[out] mpu6050_handle MPU6050 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_init(i2c_master_bus_handle_t master_handle, const mpu6050_config_t *mpu6050_config, mpu6050_handle_t *mpu6050_handle);

/**
 * @brief Reads status for FIFO buffer overflow, I2C master, and data ready interrupts.
 * 
 * @note Interrupt status bits clear when read.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] fifo_overflow FIFO buffer overflow interrupt generated when true.
 * @param[out] i2c_master I2C master interrupt generated when true.
 * @param[out] data_ready Measurement data ready interrupt generated when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_status(mpu6050_handle_t handle, bool *fifo_overflow, bool *i2c_master, bool *data_ready);

/**
 * @brief Reads interrupt status for data ready interrupt.
 * 
 * @note Interrupt status bits clear when read.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] ready Measurement data ready interrupt generated when true.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_data_status(mpu6050_handle_t handle, bool *ready);

/**
 * @brief Reads gyroscope, accelerometer, and temperature measurements.
 * 
 * @param[in] handle MPU6050 device handle.
 * @param[out] gyro_data Gyroscope data axes measurements in degrees per second.
 * @param[out] accel_data Accelerometer data axes measurements relative to standard gravity (g).
 * @param[out] temperature Temperature measurement in degrees Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_get_motion(mpu6050_handle_t handle, mpu6050_gyro_data_axes_t *gyro_data, mpu6050_accel_data_axes_t *accel_data, float *temperature);

esp_err_t mpu6050_get_rotation(mpu6050_handle_t handle, mpu6050_gyro_data_axes_t *gyro_data);

esp_err_t mpu6050_get_acceleration(mpu6050_handle_t handle, mpu6050_accel_data_axes_t *accel_data);

esp_err_t mpu6050_get_temperature(mpu6050_handle_t handle, float *temperature);

esp_err_t mpu6050_reset_signal_condition(mpu6050_handle_t handle);

esp_err_t mpu6050_reset_fifo(mpu6050_handle_t handle);

esp_err_t mpu6050_reset_sensors(mpu6050_handle_t handle);

/**
 * @brief Registers an Interrupt Service Routine to handle MPU6050 interrupts.
 * 
 * @param handle MPU6050 device handle.
 * @param isr Function to handle interrupts produced by MPU6050.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_register_isr(mpu6050_handle_t handle, const mpu6050_isr_t isr);

/**
 * @brief Issues soft-reset to MPU6050 and initializes MPU6050 device handle registers.
 *        However, device configuration registers must be configured before reading data registers.
 * 
 * @param handle MPU6050 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_reset(mpu6050_handle_t handle);

/**
 * @brief Removes an MPU6050 device from master bus.
 *
 * @param[in] handle MPU6050 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_remove(mpu6050_handle_t handle);

/**
 * @brief Removes an MPU6050 device from master bus and frees handle.
 * 
 * @param handle MPU6050 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t mpu6050_delete(mpu6050_handle_t handle);

/**
 * @brief Converts MPU6050 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* MPU6050 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* mpu6050_get_fw_version(void);

/**
 * @brief Converts MPU6050 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t MPU6050 firmware version number.
 */
int32_t mpu6050_get_fw_version_number(void);



#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __MPU6050_H__
