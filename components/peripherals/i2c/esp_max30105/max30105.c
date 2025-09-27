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
 * @file max30105.c
 *
 * ESP-IDF driver for MAX30105 sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/max30105.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/*
 * MAX30105 definitions
 */

#define MAX30105_REG_INT_STS1_R              UINT8_C(0x00) /*!< max30105 */ 
#define MAX30105_REG_INT_STS2_R              UINT8_C(0x01) /*!< max30105 */ 
#define MAX30105_REG_INT_ENB1_RW             UINT8_C(0x02) /*!< max30105 */ 
#define MAX30105_REG_INT_ENB2_RW             UINT8_C(0x03) /*!< max30105 */ 

#define MAX30105_REG_FIFO_WR_PTR_RW          UINT8_C(0x04) /*!< max30105 */ 
#define MAX30105_REG_FIFO_OVF_CNT_RW         UINT8_C(0x05) /*!< max30105 */ 
#define MAX30105_REG_FIFO_RD_PTR_RW          UINT8_C(0x06) /*!< max30105 */ 
#define MAX30105_REG_FIFO_DATA_RW            UINT8_C(0x07) /*!< max30105 */ 

#define MAX30105_REG_FIFO_CONFIG_RW          UINT8_C(0x08) /*!< max30105 */
#define MAX30105_REG_MODE_CONFIG_RW          UINT8_C(0x09) /*!< max30105 */
#define MAX30105_REG_SPO2_CONFIG_RW          UINT8_C(0x0A) /*!< max30105 */
#define MAX30105_REG_LED1_PA_RW              UINT8_C(0x0C) /*!< max30105 */
#define MAX30105_REG_LED2_PA_RW              UINT8_C(0x0D) /*!< max30105 */
#define MAX30105_REG_LED3_PA_RW              UINT8_C(0x0E) /*!< max30105 */
#define MAX30105_REG_PILOT_PA_RW             UINT8_C(0x10) /*!< max30105 */
#define MAX30105_REG_MLED1_MC_RW             UINT8_C(0x11) /*!< max30105 */
#define MAX30105_REG_MLED2_MC_RW             UINT8_C(0x12) /*!< max30105 */

#define MAX30105_REG_DIETEMP_INT_R           UINT8_C(0x1F) /*!< max30105 */
#define MAX30105_REG_DIETEMP_FRAC_R          UINT8_C(0x20) /*!< max30105 */
#define MAX30105_REG_DIETEMP_CONFIG_R        UINT8_C(0x21) /*!< max30105 */

#define MAX30105_REG_PROX_INT_THLD_RW        UINT8_C(0x30) /*!< max30105 */
#define MAX30105_REG_REV_ID_R                UINT8_C(0xFE) /*!< max30105 */
#define MAX30105_REG_PART_ID_R               UINT8_C(0xFE) /*!< max30105 */



#define MAX30105_DATA_POLL_TIMEOUT_MS       UINT16_C(100)
#define MAX30105_DATA_READY_DELAY_MS        UINT16_C(2)
#define MAX30105_POWERUP_DELAY_MS           UINT16_C(120)
#define MAX30105_RESET_DELAY_MS             UINT16_C(25)
#define MAX30105_SETUP_DELAY_MS             UINT16_C(15)
#define MAX30105_APPSTART_DELAY_MS          UINT16_C(10)    /*!< max30105 delay after initialization before application start-up */
#define MAX30105_CMD_DELAY_MS               UINT16_C(5)     /*!< max30105 delay before attempting I2C transactions after a command is issued */
#define MAX30105_TX_RX_DELAY_MS             UINT16_C(10)    /*!< max30105 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

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
typedef union __attribute__((packed)) max30105_mode_config_register_u {
    struct {
        max30105_control_modes_t control_mode:3;     /*!< max30105 These bits set the operating state of the MAX30105. Changing modes does not change any other setting, nor does it erase any previously stored data inside the data registers. (bit:0-2) */
        uint8_t                  reserved:3;  /*!< reserved (bit:5-3) */
        bool                     reset_enabled:1;    /*!< max30105 soft-reset is asserted when enabled, When the RESET bit is set to one, all configuration, threshold, and data registers are reset to their power-on-state through a power-on reset. (bit:6) */
        bool                     shutdown_enabled:1; /*!< max30105 shutdown is asserted when enabled, The part can be put into a power-save mode by setting this bit to one. While in power-save mode, all registers retain their values, and write/read operations function as normal. All interrupts are cleared to zero in this mode. (bit:7) */
    } bits;
    uint8_t reg;
} max30105_mode_config_register_t;

/**
 * @brief MAX30105 multi-LED mode control 1 register (0x11, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_multi_led_mode_control1_register_u {
    struct {
        max30105_multi_led_control_modes_t slot_1:3;    /*!< max30105 multi-LED mode time slot 1 (bit:0-2) */
        uint8_t                            reserved1:1; /*!< reserved (bit:3) */
        max30105_multi_led_control_modes_t slot_2:3;    /*!< max30105 multi-LED mode time slot 2 (bit:4-6) */
        uint8_t                            reserved2:1; /*!< reserved (bit:7) */
    } bits;
    uint8_t reg;
} max30105_multi_led_mode_control1_register_t;

/**
 * @brief MAX30105 multi-LED mode control 2 register (0x12, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_multi_led_mode_control2_register_u {
    struct {
        max30105_multi_led_control_modes_t slot_3:3;    /*!< max30105 multi-LED mode time slot 3 (bit:0-2) */
        uint8_t                            reserved1:1; /*!< reserved (bit:3) */
        max30105_multi_led_control_modes_t slot_4:3;    /*!< max30105 multi-LED mode time slot 4 (bit:4-6) */
        uint8_t                            reserved2:1; /*!< reserved (bit:7) */
    } bits;
    uint8_t reg;
} max30105_multi_led_mode_control2_register_t;

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
 * @brief MAX30105 FIFO configuration register (0x08, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_fifo_config_register_u {
    struct {
        uint8_t fifo_almost_full_threshold:4;  /*!< max30105 number of samples in fifo before triggering fifo almost full interrupt (bit:0-3) */
        bool fifo_rollover_enabled:1;            /*!< max30105 fifo rollover enabled   (bit:4) */
        max30105_sample_averages_t sample_averaging:3; /*!< max30105 sampling averaging (bit:5-7) */
    } bits;
    uint8_t reg;
} max30105_fifo_config_register_t;

/**
 * @brief MAX30105 particle-sensing configuration register (0x0A, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_particle_sensing_config_register_u {
    struct {
        max30105_led_pulse_width_controls_t led_pulse_width:2;  /*!< max30105 LED pulse width control              (bit:0-1) */
        max30105_sample_rate_controls_t sample_rate:3;          /*!< max30105 particle-sensing sample rate control (bit:2-4) */
        max30105_adc_range_controls_t adc_resolution:2;         /*!< max30105 particle-sensing ADC range control   (bit:5-6) */
        uint8_t reserved:1;                                     /*!< reserved  (bit:7) */
    } bits;
    uint8_t reg;
} max30105_particle_sensing_config_register_t;

/**
 * @brief MAX30105 temperature fraction register (0x20, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_temperature_fraction_register_u {
    struct {
        uint8_t fraction:4;  /*!< max30105 temperature fraction  (bit:0-3) */
        uint8_t reserved:4;  /*!< reserved   (bit:4-7) */
    } bits;
    uint8_t reg;
} max30105_temperature_fraction_register_t;

/**
 * @brief MAX30105 temperature configuration register (0x21, read-write | POR State 0x00) structure.
 */
typedef union __attribute__((packed)) max30105_temperature_config_register_u {
    struct {
        uint8_t enable:1;  /*!< max30105 initiates a single temperature reading when enabled (bit:0) */
        uint8_t reserved:7;  /*!< reserved   (bit:1-7) */
    } bits;
    uint8_t reg;
} max30105_temperature_config_register_t;



/**
 * @brief MAX30105 device descriptor structure definition.
 */
typedef struct max30105_device_s {
    max30105_config_t                      config;      /*!< max30105 device configuration */
    i2c_master_dev_handle_t                i2c_handle;  /*!< max30105 I2C device handle */
} max30105_device_t;

/*
* static constant declarations
*/
static const char *TAG = "max30105";



/**
 * @brief MAX30105 I2C HAL read byte array from register address transaction.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg_addr MAX30105 register address to read from.
 * @param buffer MAX30105 read transaction return byte array.
 * @param size MAX30105 number of bytes to read.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_read_from(max30105_device_t *const device, const uint8_t reg_addr, uint8_t *const buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read from failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read from failed" );

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read byte from register address transaction.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg_addr MAX30105 register address to read from.
 * @param byte MAX30105 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_read_byte_from(max30105_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read byte from failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read byte from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write byte to register address transaction.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg_addr MAX30105 register address to write to.
 * @param byte MAX30105 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_write_byte_to(max30105_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write byte to failed" );
                        
    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read interrupt status 1 register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Interrupt status 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_interrupt_status1_register(max30105_device_t *const device, max30105_interrupt_status1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_INT_STS1_R, &reg->reg), TAG, "read interrupt status 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read interrupt status 2 register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Interrupt status 2 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_interrupt_status2_register(max30105_device_t *const device, max30105_interrupt_status2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_INT_STS2_R, &reg->reg), TAG, "read interrupt status 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read interrupt enable 1 register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Interrupt enable 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_interrupt_enable1_register(max30105_device_t *const device, max30105_interrupt_enable1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_INT_ENB1_RW, &reg->reg), TAG, "read interrupt enable 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write interrupt enable 1 register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Interrupt enable 1 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_interrupt_enable1_register(max30105_device_t *const device, const max30105_interrupt_enable1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_interrupt_enable1_register_t irq_enable1 = { .reg = reg.reg };

    /* set register reserved settings */
    irq_enable1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_INT_ENB1_RW, irq_enable1.reg), TAG, "write interrupt enable 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read interrupt enable 2 register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Interrupt enable 2 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_interrupt_enable2_register(max30105_device_t *const device, max30105_interrupt_enable2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_INT_ENB2_RW, &reg->reg), TAG, "read interrupt enable 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write interrupt enable 2 register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Interrupt enable 2 register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_interrupt_enable2_register(max30105_device_t *const device, const max30105_interrupt_enable2_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_interrupt_enable2_register_t irq_enable2 = { .reg = reg.reg };

    /* set register reserved settings */
    irq_enable2.bits.reserved1 = 0;
    irq_enable2.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_INT_ENB2_RW, irq_enable2.reg), TAG, "write interrupt enable 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read FIFO write pointer register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg FIFO write pointer register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_fifo_write_pointer_register(max30105_device_t *const device, max30105_fifo_write_pointer_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_FIFO_WR_PTR_RW, &reg->reg), TAG, "read FIFO write pointer register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write FIFO write pointer register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg FIFO write pointer register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_fifo_write_pointer_register(max30105_device_t *const device, const max30105_fifo_write_pointer_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_fifo_write_pointer_register_t fifo_write = { .reg = reg.reg };

    /* set register reserved settings */
    fifo_write.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_FIFO_WR_PTR_RW, fifo_write.reg), TAG, "write FIFO write pointer register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read FIFO overflow counter register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg FIFO overflow counter register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_fifo_overflow_counter_register(max30105_device_t *const device, max30105_fifo_overflow_counter_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_FIFO_OVF_CNT_RW, &reg->reg), TAG, "read FIFO overflow counter register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write FIFO overflow counter register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg FIFO overflow counter register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_fifo_overflow_counter_register(max30105_device_t *const device, const max30105_fifo_overflow_counter_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_fifo_overflow_counter_register_t fifo_ovf = { .reg = reg.reg };

    /* set register reserved settings */
    fifo_ovf.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_FIFO_OVF_CNT_RW, fifo_ovf.reg), TAG, "write FIFO overflow counter register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read FIFO read pointer register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg FIFO read pointer register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_fifo_read_pointer_register(max30105_device_t *const device, max30105_fifo_read_pointer_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_FIFO_RD_PTR_RW, &reg->reg), TAG, "read FIFO read pointer register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write FIFO read pointer register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg FIFO read pointer register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_fifo_read_pointer_register(max30105_device_t *const device, const max30105_fifo_read_pointer_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_fifo_read_pointer_register_t fifo_read = { .reg = reg.reg };

    /* set register reserved settings */
    fifo_read.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_FIFO_RD_PTR_RW, fifo_read.reg), TAG, "write FIFO read pointer register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read FIFO data register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg_buffer FIFO data register buffer.
 * @param size Size of FIFO data register buffer.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_fifo_data_register(max30105_device_t *const device, uint8_t *const reg_buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_from(device, MAX30105_REG_FIFO_DATA_RW, reg_buffer, size), TAG, "read FIFO data register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write FIFO data register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg FIFO data register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_fifo_data_register(max30105_device_t *const device, const uint8_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_FIFO_DATA_RW, reg), TAG, "write FIFO data register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read FIFO configuration register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg FIFO configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_fifo_config_register(max30105_device_t *const device, max30105_fifo_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_FIFO_CONFIG_RW, &reg->reg), TAG, "read FIFO configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write FIFO configuration register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg FIFO configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_fifo_config_register(max30105_device_t *const device, const max30105_fifo_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_fifo_config_register_t fifo_config = { .reg = reg.reg };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_FIFO_CONFIG_RW, fifo_config.reg), TAG, "write FIFO configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read mode configuration register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Mode configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_mode_config_register(max30105_device_t *const device, max30105_mode_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_MODE_CONFIG_RW, &reg->reg), TAG, "read mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write mode configuration register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Mode configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_mode_config_register(max30105_device_t *const device, const max30105_mode_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_mode_config_register_t mode_config = { .reg = reg.reg };

    /* set register reserved settings */
    mode_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_MODE_CONFIG_RW, mode_config.reg), TAG, "write mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read particle-sensing configuration register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Particle-sensing configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_particle_sensing_config_register(max30105_device_t *const device, max30105_particle_sensing_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_SPO2_CONFIG_RW, &reg->reg), TAG, "read particle-sensing configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write particle-sensing configuration register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Particle-sensing configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_particle_sensing_config_register(max30105_device_t *const device, const max30105_particle_sensing_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_particle_sensing_config_register_t particle_config = { .reg = reg.reg };

    /* set register reserved settings */
    particle_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_SPO2_CONFIG_RW, particle_config.reg), TAG, "write particle-sensing configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}



/**
 * @brief MAX30105 I2C HAL read red (LED1) LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Red (LED1) LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_red_led_pulse_amplitude_register(max30105_device_t *const device, max30105_led_pulse_amplitudes_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_LED1_PA_RW, (uint8_t*)reg), TAG, "read red (LED1) LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write red (LED1) LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Red (LED1) LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_red_led_pulse_amplitude_register(max30105_device_t *const device, const max30105_led_pulse_amplitudes_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_LED1_PA_RW, reg), TAG, "write red (LED1) LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read infrared (LED2) LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Infrared (LED2) LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_ir_led_pulse_amplitude_register(max30105_device_t *const device, max30105_led_pulse_amplitudes_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_LED2_PA_RW, (uint8_t*)reg), TAG, "read infrared (LED2) LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write infrared (LED2) LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Infrared (LED2) LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_ir_led_pulse_amplitude_register(max30105_device_t *const device, const max30105_led_pulse_amplitudes_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_LED2_PA_RW, reg), TAG, "write infrared (LED2) LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read green (LED3) LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Green (LED3) LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_green_led_pulse_amplitude_register(max30105_device_t *const device, max30105_led_pulse_amplitudes_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_LED3_PA_RW, (uint8_t*)reg), TAG, "read green (LED3) LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write green (LED3) LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Green (LED3) LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_green_led_pulse_amplitude_register(max30105_device_t *const device, const max30105_led_pulse_amplitudes_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_LED3_PA_RW, reg), TAG, "write green (LED3) LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read proximity mode LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Proximity LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_proximity_led_pulse_amplitude_register(max30105_device_t *const device, max30105_led_pulse_amplitudes_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_PILOT_PA_RW, (uint8_t*)reg), TAG, "read proximity LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write proximity mode LED pulse amplitude register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Proximity LED pulse amplitude register (0x00h = 0.0mA, 0x01h = 0.2mA, 0x0Fh = 3.1mA, 0xFFh = 50.0mA).
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_proximity_led_pulse_amplitude_register(max30105_device_t *const device, const max30105_led_pulse_amplitudes_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_PILOT_PA_RW, reg), TAG, "write proximity LED pulse amplitude register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read multi LED mode control 1 (0x11) register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Multi LED mode control 1 (0x11) register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_multi_led_mode_control1_register(max30105_device_t *const device, max30105_multi_led_mode_control1_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_MLED1_MC_RW, &reg->reg), TAG, "read multi LED mode control 1 (0x11) register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write multi LED mode control 1 (0x11) register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Multi LED mode control 1 (0x11) register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_multi_led_mode_control1_register(max30105_device_t *const device, const max30105_multi_led_mode_control1_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_multi_led_mode_control1_register_t multi_led_config = { .reg = reg.reg };

    /* set register reserved settings */
    multi_led_config.bits.reserved1 = 0;
    multi_led_config.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_MLED1_MC_RW, multi_led_config.reg), TAG, "write multi LED mode control 1 (0x11) register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read multi LED mode control 2 (0x12) register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Multi LED mode control 2 (0x12) register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_multi_led_mode_control2_register(max30105_device_t *const device, max30105_multi_led_mode_control2_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_MLED2_MC_RW, &reg->reg), TAG, "read multi LED mode control 2 (0x12) register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write multi LED mode control 2 (0x12) register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Multi LED mode control 2 (0x12) register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_multi_led_mode_control2_register(max30105_device_t *const device, const max30105_multi_led_mode_control2_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_multi_led_mode_control2_register_t multi_led_config = { .reg = reg.reg };

    /* set register reserved settings */
    multi_led_config.bits.reserved1 = 0;
    multi_led_config.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_MLED2_MC_RW, multi_led_config.reg), TAG, "write multi LED mode control 2 (0x12) register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read temperature integer register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Temperature integer register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_temperature_integer_register(max30105_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_DIETEMP_INT_R, reg), TAG, "read temperature integer register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read temperature fraction register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Temperature fraction register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_temperature_fraction_register(max30105_device_t *const device, max30105_temperature_fraction_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_DIETEMP_FRAC_R, &reg->reg), TAG, "read temperature fraction register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL read temperature configuration register.
 *
 * @param device MAX30105 device descriptor.
 * @param reg Temperature configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_get_temperature_config_register(max30105_device_t *const device, max30105_temperature_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_read_byte_from(device, MAX30105_REG_DIETEMP_CONFIG_R, &reg->reg), TAG, "read temperature configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief MAX30105 I2C HAL write temperature configuration register.
 * 
 * @param device MAX30105 device descriptor.
 * @param reg Temperature configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t max30105_i2c_set_temperature_config_register(max30105_device_t *const device, const max30105_temperature_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* copy register */
    max30105_temperature_config_register_t temperature_config = { .reg = reg.reg };

    /* set register reserved settings */
    temperature_config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_write_byte_to(device, MAX30105_REG_DIETEMP_CONFIG_R, temperature_config.reg), TAG, "write temperature configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    return ESP_OK;
}

/**
 * @brief 
 * 
 * @param device 
 * @param red_count 
 * @param ir_count 
 * @param green_count 
 * @param count_size 
 * @return esp_err_t 
 */
static inline esp_err_t max30105_i2c_get_fifo_data(max30105_device_t *const device, uint32_t *const red_count, uint32_t *const ir_count, uint32_t *const green_count, uint8_t *const count_size) {
    max30105_mode_config_register_t mode_config = { 0 };
    max30105_particle_sensing_config_register_t particle_config = { 0 };
    max30105_fifo_read_pointer_register_t fifo_read_ptr = { 0 };
    max30105_fifo_write_pointer_register_t fifo_write_ptr = { 0 };
    uint8_t *fifo_buffer = NULL;
    uint8_t fifo_pointer_size = { 0 };
    uint8_t fifo_buffer_size = { 0 };
    uint8_t fifo_frame_size = { 0 };
    uint8_t data_resolution_bit = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt fifo read pointer register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_fifo_read_pointer_register(device, &fifo_read_ptr), TAG, "read FIFO read pointer register failed" );

    /* attempt fifo write pointer register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_fifo_write_pointer_register(device, &fifo_write_ptr), TAG, "read FIFO write pointer register failed" );

    /* determine pointer size of fifo data */
    if(fifo_write_ptr.bits.fifo_write_pointer > fifo_read_ptr.bits.fifo_read_pointer) {
        fifo_pointer_size = fifo_write_ptr.bits.fifo_write_pointer - fifo_read_ptr.bits.fifo_read_pointer;
    } else {
        fifo_pointer_size = 32 + fifo_write_ptr.bits.fifo_write_pointer - fifo_read_ptr.bits.fifo_read_pointer;
    }

    /* attempt mode configuration register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_mode_config_register(device, &mode_config), TAG, "read mode configuration register failed" );

    /* determine size of fifo frame */
    switch(mode_config.bits.control_mode) {
        case MAX30105_CM_RED_LED:
            fifo_frame_size = 3;
            break;
        case MAX30105_CM_RED_IR_LED:
            fifo_frame_size = 6;
            break;
        case MAX30105_CM_GREEN_RED_IR_LED:
            fifo_frame_size = 9;
            break;
        default:
            ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_RESPONSE, TAG, "unknown control mode in mode configuration register");
    }

    /* set fifo buffer size and attempt to allocate fifo buffer */
    fifo_buffer_size = fifo_frame_size * fifo_pointer_size;
    fifo_buffer      = malloc(fifo_buffer_size);
    ESP_RETURN_ON_FALSE(fifo_buffer != NULL, ESP_ERR_NO_MEM, TAG, "malloc for FIFO buffer failed");

    /* attempt fifo data register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_fifo_data_register(device, fifo_buffer, fifo_buffer_size), TAG, "read fifo data register failed" );

    /* attempt particle-sensing configuration register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_particle_sensing_config_register(device, &particle_config), TAG, "read particle-sensing configuration register failed" );

    /* determine adc resolution */
    switch(particle_config.bits.led_pulse_width) {
        case MAX30105_LPWC_69US_15BITS:
            data_resolution_bit = 3;
            break;
        case MAX30105_LPWC_118US_16BITS:
            data_resolution_bit = 2;
            break;
        case MAX30105_LPWC_215US_17BITS:
            data_resolution_bit = 1;
            break;
        case MAX30105_LPWC_411US_18BITS:
            data_resolution_bit = 0;
            break;
        default:
            ESP_RETURN_ON_FALSE(false, ESP_ERR_INVALID_RESPONSE, TAG, "unknown ADC resolution in particle-sensing configuration register");
    }

    /* iterate through fifo buffer */
    for(uint8_t i = 0; i < fifo_pointer_size; i++) {
        /* extract data by number of LEDs configured */
        switch(mode_config.bits.control_mode) {
            case MAX30105_CM_RED_LED:
                red_count[i] = ((uint32_t)fifo_buffer[i * fifo_frame_size + 0] << 16) |                                            
                               ((uint32_t)fifo_buffer[i * fifo_frame_size + 1] << 8) |                                               
                               ((uint32_t)fifo_buffer[i * fifo_frame_size + 2] << 0);                                                
                red_count[i] = red_count[i] >> data_resolution_bit;
                break;
            case MAX30105_CM_RED_IR_LED:
                red_count[i] = ((uint32_t)fifo_buffer[i * fifo_frame_size + 0] << 16) |                                            
                               ((uint32_t)fifo_buffer[i * fifo_frame_size + 1] << 8) |                                               
                               ((uint32_t)fifo_buffer[i * fifo_frame_size + 2] << 0);                                                
                red_count[i] = red_count[i] >> data_resolution_bit;
                ir_count[i]  = ((uint32_t)fifo_buffer[i * fifo_frame_size + 3] << 16) |                                            
                               ((uint32_t)fifo_buffer[i * fifo_frame_size + 4] << 8) |                                               
                               ((uint32_t)fifo_buffer[i * fifo_frame_size + 5] << 0);                                                
                ir_count[i]  = ir_count[i] >> data_resolution_bit;
                break;
            case MAX30105_CM_GREEN_RED_IR_LED:
                red_count[i]   = ((uint32_t)fifo_buffer[i * fifo_frame_size + 0] << 16) |                                            
                                 ((uint32_t)fifo_buffer[i * fifo_frame_size + 1] << 8) |                                               
                                 ((uint32_t)fifo_buffer[i * fifo_frame_size + 2] << 0);                                                
                red_count[i]   = red_count[i] >> data_resolution_bit;
                ir_count[i]    = ((uint32_t)fifo_buffer[i * fifo_frame_size + 3] << 16) |                                            
                                 ((uint32_t)fifo_buffer[i * fifo_frame_size + 4] << 8) |                                               
                                 ((uint32_t)fifo_buffer[i * fifo_frame_size + 5] << 0);                                                
                ir_count[i]    = ir_count[i] >> data_resolution_bit;
                green_count[i] = ((uint32_t)fifo_buffer[i * fifo_frame_size + 6] << 16) |                                            
                                 ((uint32_t)fifo_buffer[i * fifo_frame_size + 7] << 8) |                                               
                                 ((uint32_t)fifo_buffer[i * fifo_frame_size + 8] << 0);                                                
                green_count[i] = green_count[i] >> data_resolution_bit;
                break;
        }
    }

    return ESP_OK;
}

static inline esp_err_t max30105_i2c_set_reset(max30105_device_t *const device) {
    max30105_mode_config_register_t mode_config = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt mode configuration register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_mode_config_register(device, &mode_config), TAG, "read mode configuration register failed" );

    mode_config.bits.reset_enabled = true;

    /* attempt mode configuration register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_mode_config_register(device, mode_config), TAG, "write mode configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_RESET_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t max30105_i2c_setup(max30105_device_t *const device) {
    max30105_interrupt_enable1_register_t irq_enable1 = { 0 };
    max30105_interrupt_enable2_register_t irq_enable2 = { 0 };
    max30105_fifo_read_pointer_register_t fifo_read_ptr = { 0 };
    max30105_fifo_write_pointer_register_t fifo_write_ptr = { 0 };
    max30105_fifo_config_register_t fifo_config = { 0 };
    max30105_mode_config_register_t mode_config = { 0 };
    max30105_particle_sensing_config_register_t particle_config = { 0 };
    max30105_multi_led_mode_control1_register_t multi_led_config1 = { 0 };
    max30105_multi_led_mode_control2_register_t multi_led_config2 = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt interrupt enable 1 register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_interrupt_enable1_register(device, &irq_enable1), TAG, "read interrupt enable 1 register failed" );

    /* attempt interrupt enable 2 register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_interrupt_enable2_register(device, &irq_enable2), TAG, "read interrupt enable 2 register failed" );

    /* attempt fifo configuration register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_fifo_config_register(device, &fifo_config), TAG, "read FIFO configuration register failed" );

    /* attempt mode configuration register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_mode_config_register(device, &mode_config), TAG, "read mode configuration register failed" );

    /* attempt particle-sensing configuration register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_particle_sensing_config_register(device, &particle_config), TAG, "read particle-sensing configuration register failed" );

    /* attempt multi led mode control 1 register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_multi_led_mode_control1_register(device, &multi_led_config1), TAG, "read multi led mode control 1 register failed" );

    /* attempt multi led mode control 2 register read transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_get_multi_led_mode_control2_register(device, &multi_led_config2), TAG, "read multi led mode control 2 register failed" );


    /* configure registers */
    irq_enable1.bits.alc_overflow_irq_enabled           = true;
    irq_enable1.bits.data_ready_irq_enabled             = true;
    irq_enable1.bits.fifo_almost_full_irq_enabled       = true;
    irq_enable1.bits.proximity_irq_enabled              = true;

    irq_enable2.bits.irq_die_temperature_ready_enabled  = true;

    fifo_read_ptr.bits.fifo_read_pointer                = 0;
    fifo_write_ptr.bits.fifo_write_pointer              = 0;

    fifo_config.bits.fifo_almost_full_threshold         = device->config.fifo_almost_full_threshold;
    fifo_config.bits.fifo_rollover_enabled              = device->config.fifo_rollover_enabled;
    fifo_config.bits.sample_averaging                   = device->config.fifo_sample_average;
    
    mode_config.bits.control_mode                       = device->config.control_mode;

    particle_config.bits.sample_rate                    = device->config.particle_sample_rate;
    particle_config.bits.adc_resolution                 = device->config.particle_adc_resolution;
    particle_config.bits.led_pulse_width                = device->config.led_pulse_width;
    
    multi_led_config1.bits.slot_1                       = device->config.multi_led_mode_slot1;
    multi_led_config1.bits.slot_2                       = device->config.multi_led_mode_slot2;

    multi_led_config2.bits.slot_3                       = device->config.multi_led_mode_slot3;
    multi_led_config2.bits.slot_4                       = device->config.multi_led_mode_slot4;

    /* write configured registers */

    /* attempt interrupt enable 1 register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_interrupt_enable1_register(device, irq_enable1), TAG, "write interrupt enable 1 register failed" );

    /* attempt interrupt enable 2 register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_interrupt_enable2_register(device, irq_enable2), TAG, "write interrupt enable 2 register failed" );

    /* attempt fifo read pointer register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_fifo_read_pointer_register(device, fifo_read_ptr), TAG, "write FIFO read pointer register failed" );

    /* attempt fifo write pointer register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_fifo_write_pointer_register(device, fifo_write_ptr), TAG, "write FIFO write pointer register failed" );

    /* attempt fifo configuration register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_fifo_config_register(device, fifo_config), TAG, "write FIFO configuration register failed" );

    /* attempt mode configuration register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_mode_config_register(device, mode_config), TAG, "write mode configuration register failed" );

    /* attempt particle-sensing configuration register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_particle_sensing_config_register(device, particle_config), TAG, "write particle-sensing configuration register failed" );

    /* attempt red LED pulse amplitude register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_red_led_pulse_amplitude_register(device, device->config.red_led_pulse_amplitude), TAG, "write red LED pulse amplitude register failed" );

    /* attempt infrared LED pulse amplitude register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_ir_led_pulse_amplitude_register(device, device->config.ir_led_pulse_amplitude), TAG, "write infrared LED pulse amplitude register failed" );

    /* attempt green LED pulse amplitude register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_green_led_pulse_amplitude_register(device, device->config.green_led_pulse_amplitude), TAG, "write green LED pulse amplitude register failed" );

    /* attempt proximity LED pulse amplitude register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_proximity_led_pulse_amplitude_register(device, device->config.proximity_led_pulse_amplitude), TAG, "write proximity LED pulse amplitude register failed" );

    /* attempt multi led mode control 1 register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_multi_led_mode_control1_register(device, multi_led_config1), TAG, "write multi led mode control 1 register failed" );

    /* attempt multi led mode control 2 register write transaction */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_multi_led_mode_control2_register(device, multi_led_config2), TAG, "write multi led mode control 2 register failed" );

    return ESP_OK;
}

esp_err_t max30105_init(i2c_master_bus_handle_t master_handle, const max30105_config_t *max30105_config, max30105_handle_t *max30105_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && max30105_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, max30105_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, max30105 device handle initialization failed", max30105_config->i2c_address);

    /* validate memory availability for handle */
    max30105_device_t* device = (max30105_device_t*)calloc(1, sizeof(max30105_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c max30105 device, init failed");

    /* copy configuration */
    device->config = *max30105_config;

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
    vTaskDelay(pdMS_TO_TICKS(MAX30105_CMD_DELAY_MS));

    /* attempt soft-reset */
    ESP_RETURN_ON_ERROR( max30105_i2c_set_reset(device), TAG, "write reset for init failed" );

    /* attempt setup */
    ESP_RETURN_ON_ERROR( max30105_i2c_setup(device), TAG, "setup for init failed" );

    /* set device handle */
    *max30105_handle = (max30105_handle_t)device;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(MAX30105_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}