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
 * @file tcs3472.c
 *
 * ESP-IDF driver for OSRAM TCS3472 RGBC sensor.  This is an updated implementation 
 * with inline HAL functions for reading and writing to TCS3472 registers.  This 
 * change exposes API properties specific to the component without exposing hardware 
 * abstraction layer functions related to device interfacing.
 * 
 * https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.cpp
 * 
 * https://github.com/hideakitai/TCS34725
 * 
 * https://look.ams-osram.com/m/57eb503c1d12d800/original/TCS34xx-AN000517.pdf
 * 
 * https://look.ams-osram.com/m/512cf89f2f0a2d98/original/ColorSensors-AN000166.pdf
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/tcs3472.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


/*
 * TCS3472 I2C register definitions
*/
#define TCS3472_CMD_BIT             UINT8_C(0x80)   /**< tcs3472 command bit **/
#define TCS3472_REG_ENABLE_RW       UINT8_C(0x00)   /*!< tcs3472 enable register */
#define TCS3472_REG_ATIME_RW        UINT8_C(0x01)   /*!< tcs3472 RGBC integration time register */
#define TCS3472_REG_WTIME_RW        UINT8_C(0x03)   /*!< tcs3472 wait time register */
#define TCS3472_REG_AILTL_RW        UINT8_C(0x04)   /*!< tcs3472 clear interrupt low threshold low byte register */
#define TCS3472_REG_AILTH_RW        UINT8_C(0x05)   /*!< tcs3472 clear interrupt low threshold high byte register */
#define TCS3472_REG_AIHTL_RW        UINT8_C(0x06)   /*!< tcs3472 clear interrupt high threshold low byte register */
#define TCS3472_REG_AIHTH_RW        UINT8_C(0x07)   /*!< tcs3472 clear interrupt high threshold high byte register */
#define TCS3472_REG_PERS_RW         UINT8_C(0x0C)   /*!< tcs3472 interrupt persistence filter register */
#define TCS3472_REG_CONFIG_RW       UINT8_C(0x0D)   /*!< tcs3472 configuration register */
#define TCS3472_REG_CONTROL_RW      UINT8_C(0x0F)   /*!< tcs3472 gain control register */
#define TCS3472_REG_ID_R            UINT8_C(0x12)   /*!< tcs3472 device ID register */
#define TCS3472_REG_STATUS_R        UINT8_C(0x13)   /*!< tcs3472 device status register */
#define TCS3472_REG_CDATAL_R        UINT8_C(0x14)   /*!< tcs3472 clear data low byte register */
#define TCS3472_REG_CDATAH_R        UINT8_C(0x15)   /*!< tcs3472 clear data high byte register */
#define TCS3472_REG_RDATAL_R        UINT8_C(0x16)   /*!< tcs3472 red data low byte register */
#define TCS3472_REG_RDATAH_R        UINT8_C(0x17)   /*!< tcs3472 red data high byte register */
#define TCS3472_REG_GDATAL_R        UINT8_C(0x18)   /*!< tcs3472 green data low byte register */
#define TCS3472_REG_GDATAH_R        UINT8_C(0x19)   /*!< tcs3472 green data high byte register */
#define TCS3472_REG_BDATAL_R        UINT8_C(0x1A)   /*!< tcs3472 blue data low byte register */
#define TCS3472_REG_BDATAH_R        UINT8_C(0x1B)   /*!< tcs3472 blue data high byte register */


#define TCS3472_POWERUP_DELAY_MS    UINT16_C(5)     /*!< tcs3472 delay on power-up before attempting I2C transactions */
#define TCS3472_APPSTART_DELAY_MS   UINT16_C(10)    /*!< tcs3472 delay after initialization before application start-up */
#define TCS3472_DATA_READY_DELAY_MS UINT16_C(1)
#define TCS3472_CMD_DELAY_MS        UINT16_C(5)     /*!< tcs3472 delay before attempting I2C transactions after a command is issued */
#define TCS3472_RETRY_DELAY_MS      UINT16_C(2)     /*!< tcs3472 delay between an I2C receive transaction retry */
#define TCS3472_TX_RX_DELAY_MS      UINT16_C(10)    /*!< tcs3472 delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */


#define I2C_XFR_TIMEOUT_MS          (500)          //!< I2C transaction timeout in milliseconds


// --- DN40 Constants for TCS3472 ---
#define TCS3472_DF                  310.0f   // Device Factor (from datasheet/DN40)
#define TCS3472_R_COEF              0.136f   // Red Coefficient
#define TCS3472_G_COEF              1.000f   // Green Coefficient
#define TCS3472_B_COEF              -0.444f  // Blue Coefficient

/* Colour Temperature Calculation Coefficients - McCamy's cubic approximation */
#define TCS3472_CT_RED_X_COEF       -0.14282f
#define TCS3472_CT_RED_Y_COEF       -0.32466f
#define TCS3472_CT_RED_Z_COEF       -0.68202f
#define TCS3472_CT_GREEN_X_COEF     1.54924f
#define TCS3472_CT_GREEN_Y_COEF     1.57837f
#define TCS3472_CT_GREEN_Z_COEF     0.77073f
#define TCS3472_CT_BLUE_X_COEF      -0.95641f
#define TCS3472_CT_BLUE_Y_COEF      -0.73191f
#define TCS3472_CT_BLUE_Z_COEF      0.56332f
#define TCS3472_CT_XC_COEF          0.3320f
#define TCS3472_CT_YC_COEF          0.1858f
#define TCS3472_CT_CCT_A_COEF       449.0f
#define TCS3472_CT_CCT_B_COEF       3525.0f
#define TCS3472_CT_CCT_C_COEF       6823.0f
#define TCS3472_CT_CCT_D_COEF       5520.33f

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))

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
 * @brief TCS3472 device descriptor structure definition.
 */
typedef struct tcs3472_device_s {
    tcs3472_config_t            config;             /*!< tcs3472 device configuration */
    i2c_master_dev_handle_t     i2c_handle;         /*!< tcs3472 i2c device handle */
} tcs3472_device_t;



/*
* static constant declarations
*/
static const char *TAG = "tcs3472";

static inline float tcs3472_get_gain_multiplier(const tcs3472_gain_controls_t gain_control) {
    switch(gain_control) {
        case TCS3472_GAIN_CONTROL_1X:
            return 1.0f;
        case TCS3472_GAIN_CONTROL_4X:
            return 4.0f;
        case TCS3472_GAIN_CONTROL_16X:
            return 16.0f;
        case TCS3472_GAIN_CONTROL_60X:
            return 60.0f;
        default:
            return 1.0f;
    }
}

/**
 * @brief TCS3472 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device TCS3472 device descriptor.
 * @param reg_addr TCS3472 register address to read from.
 * @param buffer TCS3472 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_read_from(tcs3472_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    //const bit8_uint8_buffer_t tx = { reg_addr };
    const bit8_uint8_buffer_t tx = { (uint8_t)(TCS3472_CMD_BIT | reg_addr) };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "tcs3472_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read byte from register address transaction.
 * 
 * @param device TCS3472 device descriptor.
 * @param reg_addr TCS3472 register address to read from.
 * @param byte TCS3472 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_read_byte_from(tcs3472_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    //const bit8_uint8_buffer_t tx = { reg_addr };
    const bit8_uint8_buffer_t tx = { (uint8_t)(TCS3472_CMD_BIT | reg_addr) };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "tcs3472_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read word from register address transaction.
 * 
 * @param device TCS3472 device descriptor.
 * @param reg_addr TCS3472 register address to read from.
 * @param word TCS3472 read transaction return fword.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_read_word_from(tcs3472_device_t *const device, const uint8_t reg_addr, uint16_t *const word) {
    //const bit8_uint8_buffer_t tx = { reg_addr };
    const bit8_uint8_buffer_t tx = { (uint8_t)(TCS3472_CMD_BIT | reg_addr) };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "tcs3472_i2c_read_word_from failed" );

    /* set output parameter */
    //*word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);
    *word = ((uint16_t)rx[1] << 8) | ((uint16_t)rx[0] & 0xFF);

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write byte to register address transaction.
 * 
 * @param device TCS3472 device descriptor.
 * @param reg_addr TCS3472 register address to write to.
 * @param byte TCS3472 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_write_byte_to(tcs3472_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    //const bit16_uint8_buffer_t tx = { reg_addr, byte };
    const bit16_uint8_buffer_t tx = { (uint8_t)(TCS3472_CMD_BIT | reg_addr), byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "tcs3472_i2c_write_byte_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write word to register address transaction.
 * 
 * @param device TCS3472 device descriptor.
 * @param reg_addr TCS3472 register address to write to.
 * @param word TCS3472 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_write_word_to(tcs3472_device_t *const device, const uint8_t reg_addr, const uint16_t word) {
    //const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)word, (uint8_t)word >> 8 };
    const bit24_uint8_buffer_t tx = { (uint8_t)(TCS3472_CMD_BIT | reg_addr), (uint8_t)word, (uint8_t)word >> 8 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "tcs3472_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read enable register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Enable register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_enable_register(tcs3472_device_t *const device, tcs3472_enable_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_ENABLE_RW, &reg->reg), TAG, "read enable register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write enable register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Enable register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_enable_register(tcs3472_device_t *const device, const tcs3472_enable_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(device, TCS3472_REG_ENABLE_RW, reg.reg), TAG, "write enable register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read RGBC integration time register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg RGBC time register controls the internal integration time of the RGBC clear and IR channel ADCs in 2.4-ms increments.
 *                 Max RGBC Count = (256 − ATIME) × 1024 up to a maximum of 65535.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_atime_register(tcs3472_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_ATIME_RW, reg), TAG, "read atime register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write RGBC integration time register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg RGBC time register controls the internal integration time of the RGBC clear and IR channel ADCs in 2.4-ms increments.
 *            Max RGBC Count = (256 − ATIME) × 1024 up to a maximum of 65535.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_atime_register(tcs3472_device_t *const device, const uint8_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(device, TCS3472_REG_ATIME_RW, reg), TAG, "write atime register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read wait time register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Wait time register is set 2.4 ms increments unless the WLONG bit is asserted, in which case the wait times are 12× longer.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_wtime_register(tcs3472_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_WTIME_RW, reg), TAG, "read wtime register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write wait time register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Wait time register is set 2.4 ms increments unless the WLONG bit is asserted, in which case the wait times are 12× longer.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_wtime_register(tcs3472_device_t *const device, const uint8_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(device, TCS3472_REG_WTIME_RW, reg), TAG, "write wtime register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read interrupt persistence filter register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Persistence register controls the filtering interrupt capabilities of the device.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_persistence_register(tcs3472_device_t *const device, tcs3472_persistence_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_PERS_RW, &reg->reg), TAG, "read persistence register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write interrupt persistence filter register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Persistence register controls the filtering interrupt capabilities of the device.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_persistence_register(tcs3472_device_t *const device, const tcs3472_persistence_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(device, TCS3472_REG_PERS_RW, reg.reg), TAG, "write persistence register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read configuration register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_config_register(tcs3472_device_t *const device, tcs3472_config_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_CONFIG_RW, &reg->reg), TAG, "read configuration register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write configuration register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_config_register(tcs3472_device_t *const device, const tcs3472_config_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(device, TCS3472_REG_CONFIG_RW, reg.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read control register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_control_register(tcs3472_device_t *const device, tcs3472_control_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_CONTROL_RW, &reg->reg), TAG, "read control register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write control register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Control register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_control_register(tcs3472_device_t *const device, const tcs3472_control_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(device, TCS3472_REG_CONTROL_RW, reg.reg), TAG, "write control register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read clear channel high threshold register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Clear channel high threshold register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_threshold_hi_register(tcs3472_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_word_from(device, TCS3472_REG_AIHTL_RW, reg), TAG, "read high threshold register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write clear channel high threshold register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Clear channel high threshold register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_threshold_hi_register(tcs3472_device_t *const device, const uint16_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_word_to(device, TCS3472_REG_AIHTL_RW, reg), TAG, "write high threshold register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read clear channel low threshold register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Clear channel low threshold register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_threshold_lo_register(tcs3472_device_t *const device, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_word_from(device, TCS3472_REG_AILTL_RW, reg), TAG, "read low threshold register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL write clear channel low threshold register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[in] reg Clear channel low threshold register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_set_threshold_lo_register(tcs3472_device_t *const device, const uint16_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_word_to(device, TCS3472_REG_AILTL_RW, reg), TAG, "write low threshold register failed" );

    return ESP_OK;
}

/**
 * @brief TCS3472 I2C HAL read status register.
 * 
 * @param[in] device TCS3472 device descriptor.
 * @param[out] reg Status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_get_status_register(tcs3472_device_t *const device, tcs3472_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(device, TCS3472_REG_STATUS_R, &reg->reg), TAG, "read status register failed" );

    return ESP_OK;
}

/**
 * @brief Converts time in milliseconds to steps at 2.4-ms increments.
 * 
 * @param time Time to convert in milliseconds (2.4-ms to 614-ms).
 * @param long_wait_enabled Long wait time multiplier (x12) is applied when enabled.
 * @return uint8_t Number of steps converted from time in milliseconds.
 */
static inline uint8_t tcs3472_convert_time_to_steps(const float time, const bool long_wait_enabled) {
    // ATIME = 256 − Integration Time / 2.4 ms

    const uint16_t max_steps = 256;
    const float time_step    = 2.4f;
    float wlong_multiplier   = 1.0f;

    /* validate wait long time multiplier */
    if(long_wait_enabled) {
        wlong_multiplier = 12.0f;
    }

    /* convert time (ms) to steps */
    return max_steps - (time / (time_step * wlong_multiplier));
}

/**
 * @brief Converts steps at 2.4-ms increments to time in milliseconds.
 * 
 * @param steps Number of steps to convert at 2.4-ms increments.
 * @param long_wait_enabled Long wait time multiplier (x12) is applied when enabled.
 * @return float Time in milliseconds converted from steps at 2.4-ms increments.
 */
static inline float tcs3472_convert_steps_to_time(const uint8_t steps, const bool long_wait_enabled) {
    // Time = 2.4 ms × (256 − ATIME)

    const float max_steps  = 256.0f;
    const float time_step  = 2.4f;
    float wlong_multiplier = 1.0f;

    /* validate wait long time multiplier */
    if(long_wait_enabled) {
        wlong_multiplier = 12.0f;
    }

    /* convert steps to time (ms) */
    return wlong_multiplier * (time_step * (max_steps - (float)steps));
}

/**
 * @brief Gets total wait time (initialization, integration time and wait time) in milliseconds from TCS3472.
 * 
 * @param device TCS3472 device descriptor.
 * @return float Total wait time (initialization, integration time and wait time) in milliseconds.
 */
static inline float tcs3472_get_total_wait_time(tcs3472_device_t *const device) {
    const float init_time = 2.4f;

    /* validate arguments */
    if (!device) return init_time;

    /* calculate integration time in milliseconds */
    return init_time + device->config.integration_time + device->config.wait_time;
}



 static inline float tcs3472_get_total_wait_time____(tcs3472_device_t *const device) {
    const float init_time = 2.4f;
    uint8_t atime_step = { 0 };
    uint8_t wtime_step = { 0 };
    tcs3472_config_register_t config = { 0 };

    /* validate arguments */
    if (!device) return init_time;

    /* attempt to read device atime register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_atime_register(device, &atime_step), TAG, "read atime register failed" );

    /* attempt to read device wtime register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_wtime_register(device, &wtime_step), TAG, "read wtime register failed" );

    /* attempt to read device configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_config_register(device, &config), TAG, "read configuration register failed" );

    /* calculate integration and wait times in milliseconds */
    const float atime = tcs3472_convert_steps_to_time(atime_step, false);
    const float wtime = tcs3472_convert_steps_to_time(wtime_step, config.bits.long_wait_enabled);

    /* calculate integration time in milliseconds */
    return init_time + atime + wtime;
}

/**
 * @brief TCS3472 I2C HAL to setup registers.
 * 
 * @param device TCS3472 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_i2c_setup_registers(tcs3472_device_t *const device) {
    tcs3472_enable_register_t       enable  = { 0 };
    tcs3472_config_register_t       config  = { 0 };
    tcs3472_persistence_register_t  persist = { 0 };
    tcs3472_control_register_t      control = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_config_register(device, &config), TAG, "read configuration register failed" );

    /* attempt to read persistence register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_persistence_register(device, &persist), TAG, "read persistence register failed" );

    /* attempt to read control register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_control_register(device, &control), TAG, "read control register failed" );

    /* configure enable register from device configuration structure */
    enable.bits.adc_enabled       = device->config.adc_enabled;
    enable.bits.power_enabled     = device->config.power_enabled;
    enable.bits.irq_enabled       = device->config.irq_enabled;
    enable.bits.wait_enabled      = device->config.wait_time_enabled;

    /* configure configuration register from device configuration structure */
    config.bits.long_wait_enabled = device->config.long_wait_enabled;

    /* configure persistence register from device configuration structure */
    persist.bits.irq_control_rate = device->config.irq_control_rate;

    /* configure control register from device configuration structure */
    control.bits.gain             = device->config.gain_control;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_config_register(device, config), TAG, "write configuration register failed" );

    /* attempt to write atime (integration time) register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_atime_register(device, tcs3472_convert_time_to_steps(device->config.integration_time, false)), TAG, "write atime register failed" );

    /* attempt to write wtime (wait time) register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_wtime_register(device, tcs3472_convert_time_to_steps(device->config.wait_time, device->config.long_wait_enabled)), TAG, "write wtime register failed" );

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register failed" );

    /* attempt to write persistence register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_persistence_register(device, persist), TAG, "write persistence register failed" );

    /* attempt to write control register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_control_register(device, control), TAG, "write control register failed" );

    /* attempt to write high and low threshold registers */
    if(device->config.set_irq_thresholds) {
        ESP_RETURN_ON_ERROR( tcs3472_i2c_set_threshold_hi_register(device, device->config.irq_high_threshold), TAG, "write high irq threshold failed" );
        ESP_RETURN_ON_ERROR( tcs3472_i2c_set_threshold_lo_register(device, device->config.irq_low_threshold), TAG, "write low irq threshold failed" );
    } else {
        ESP_RETURN_ON_ERROR( tcs3472_i2c_get_threshold_hi_register(device, &device->config.irq_high_threshold), TAG, "read high irq threshold failed" );
        ESP_RETURN_ON_ERROR( tcs3472_i2c_get_threshold_lo_register(device, &device->config.irq_low_threshold), TAG, "read low irq threshold failed" );
    }

    return ESP_OK;
}

/**
 * @brief Reads an RGBC count by channel type from TCS3472.
 * 
 * @param device TCS3472 device descriptor.
 * @param channel RGBC channel type.
 * @param count Count by RGBC channel type.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_get_channel_count(tcs3472_device_t *const device, const tcs3472_channels_t channel, uint16_t *const count) {
    esp_err_t             ret = ESP_OK;
    const uint64_t start_time = esp_timer_get_time();
    bool        data_is_ready = false;
    uint8_t           lo_reg;
    uint16_t        rx_count;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set overall wait time */
    const float wait_time = tcs3472_get_total_wait_time(device);

    /* set high and low data registers by RGBC channel type */
    switch(channel) {
        case TCS3472_CHANNEL_RED:
            lo_reg = TCS3472_REG_RDATAL_R;
            break;
        case TCS3472_CHANNEL_GREEN:
            lo_reg = TCS3472_REG_GDATAL_R;
            break;
        case TCS3472_CHANNEL_BLUE:
            lo_reg = TCS3472_REG_BDATAL_R;  
            break;
        case TCS3472_CHANNEL_CLEAR:
            lo_reg = TCS3472_REG_CDATAL_R;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* attempt to poll high and low channel data from device until data is available or timeout */
    do {
        tcs3472_status_register_t status = { 0 };

        /* attempt to read status register */
        ESP_GOTO_ON_ERROR( tcs3472_i2c_get_status_register(device, &status), err, TAG, "read status register failed" );

        /* set data is ready flag */
        data_is_ready = status.bits.data_valid;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(TCS3472_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, wait_time * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // attempt to read low and high channels from device.
    ESP_GOTO_ON_ERROR( tcs3472_i2c_read_word_from(device, lo_reg, &rx_count), err, TAG, "read RGBC low and high data channels failed." );

    /* set output parameter */
    *count = rx_count;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t tcs3472_init(i2c_master_bus_handle_t master_handle, const tcs3472_config_t *tcs3472_config, tcs3472_handle_t *tcs3472_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && tcs3472_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TCS3472_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, tcs3472_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, tcs3472 device handle initialization failed", tcs3472_config->i2c_address);

    /* validate memory availability for handle */
    tcs3472_device_t* device = (tcs3472_device_t*)calloc(1, sizeof(tcs3472_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 tcs3472 device for init");

    /* copy configuration */
    device->config = *tcs3472_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = device->config.i2c_address,
        .scl_speed_hz       = device->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (device->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &device->i2c_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TCS3472_CMD_DELAY_MS));

    /* attempt to reset the device and initialize registers */
    ESP_GOTO_ON_ERROR(tcs3472_i2c_setup_registers(device), err_handle, TAG, "setup registers for init failed");

    /* set output parameter */
    *tcs3472_handle = (tcs3472_handle_t)device;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TCS3472_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (device && device->i2c_handle) {
            i2c_master_bus_rm_device(device->i2c_handle);
        }
        free(device);
    err:
        return ret;
}

esp_err_t tcs3472_get_channels_count(tcs3472_handle_t handle, tcs3472_channels_data_t *const channels) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && channels );

    /* attempt to read red channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_red_channel_count(handle, &channels->red), TAG, "read red channel count for get channels count failed" );

    /* attempt to read green channel count */ 
    ESP_RETURN_ON_ERROR( tcs3472_get_green_channel_count(handle, &channels->green), TAG, "read green channel count for get channels count failed" );

    /* attempt to read blue channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_blue_channel_count(handle, &channels->blue), TAG, "read blue channel count for get channels count failed" );

    /* attempt to read clear channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_clear_channel_count(handle, &channels->clear), TAG, "read clear channel count for get channels count failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_red_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && count );

    /* attempt to read red channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_channel_count(device, TCS3472_CHANNEL_RED, count), TAG, "read red channel count failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_green_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && count );

    /* attempt to read green channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_channel_count(device, TCS3472_CHANNEL_GREEN, count), TAG, "read green channel count failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_blue_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && count );

    /* attempt to read blur channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_channel_count(device, TCS3472_CHANNEL_BLUE, count), TAG, "read blue channel count failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_clear_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && count );

    /* attempt to read clear channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_channel_count(device, TCS3472_CHANNEL_CLEAR, count), TAG, "read clear channel count failed" );

    return ESP_OK;
}

esp_err_t tcs3472_normalize_colours(const tcs3472_channels_data_t channels, tcs3472_colours_data_t *const colours) {
    /* validate arguments */
    ESP_ARG_CHECK( colours );

    /* avoid dividing by 0 */
    if(channels.clear == 0) {
        colours->red = colours->green = colours->blue = 0;
        return ESP_OK;
    }

    const float sum = channels.clear;

    colours->red   = (uint8_t)((float)channels.red / sum * 255.0f);
    colours->green = (uint8_t)((float)channels.green / sum * 255.0f);
    colours->blue  = (uint8_t)((float)channels.blue / sum * 255.0f);

    return ESP_OK;
}

esp_err_t tcs3472_get_colours(tcs3472_handle_t handle, tcs3472_colours_data_t *const colours) {
    tcs3472_channels_data_t channels = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle && colours );

    /* attempt to read red channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_red_channel_count(handle, &channels.red), TAG, "read red channel count for get colours failed" );

    /* attempt to read green channel count */ 
    ESP_RETURN_ON_ERROR( tcs3472_get_green_channel_count(handle, &channels.green), TAG, "read green channel count for get colours failed" );

    /* attempt to read blue channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_blue_channel_count(handle, &channels.blue), TAG, "read blue channel count for get colours failed" );

    /* attempt to read clear channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_clear_channel_count(handle, &channels.clear), TAG, "read clear channel count for get colours failed" );

    /* attempt to normalize channels count data to RGB colours */
    ESP_RETURN_ON_ERROR( tcs3472_normalize_colours(channels, colours), TAG, "normalize colours for get colours failed" );

    return ESP_OK;
}

uint16_t tcs3472_get_colour_temperature(const tcs3472_channels_data_t channels) {
    /* 1. Map RGB values to their XYZ counterparts.    */
    /* Based on 6500K fluorescent, 3000K fluorescent   */
    /* and 60W incandescent values for a wide range.   */
    /* Note: Y = Illuminance or lux                    */
    const float X = (TCS3472_CT_RED_X_COEF * channels.red) + (TCS3472_CT_GREEN_X_COEF * channels.green) + (TCS3472_CT_BLUE_X_COEF * channels.blue);
    const float Y = (TCS3472_CT_RED_Y_COEF * channels.red) + (TCS3472_CT_GREEN_Y_COEF * channels.green) + (TCS3472_CT_BLUE_Y_COEF * channels.blue);
    const float Z = (TCS3472_CT_RED_Z_COEF * channels.red) + (TCS3472_CT_GREEN_Z_COEF * channels.green) + (TCS3472_CT_BLUE_Z_COEF * channels.blue);

    /* 2. Calculate the chromaticity co-ordinates      */
    const float xyz_sum = X + Y + Z;
    if (xyz_sum == 0.0f) return 0;
    const float xc = X / xyz_sum;
    const float yc = Y / xyz_sum;

    // Safety check to avoid McCamy divide-by-zero or invalid bounds
    if (yc <= TCS3472_CT_YC_COEF) return 0;

    /* 3. Use McCamy's formula to determine the CCT    */
    const float n = (xc - TCS3472_CT_XC_COEF) / (TCS3472_CT_YC_COEF - yc);
    const float cct = (TCS3472_CT_CCT_A_COEF * powf(n, 3)) + (TCS3472_CT_CCT_B_COEF * powf(n, 2)) + (TCS3472_CT_CCT_C_COEF * n) + TCS3472_CT_CCT_D_COEF;

    return (uint16_t)cct;
}

esp_err_t tcs3472_get_illuminance(tcs3472_handle_t handle, const tcs3472_channels_data_t channels, float *const illuminance) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && illuminance );

    // 1. Calculate Counts Per Lux (CPL)
    // Formula: CPL = (ATIME_ms * AGAINx) / (GA * DF)
    const float gain_multiplier = tcs3472_get_gain_multiplier(device->config.gain_control);
    const float cpl = (device->config.integration_time * gain_multiplier) / (device->config.glass_attenuation * TCS3472_DF);

    // Safety check to prevent division by zero
    if (cpl == 0.0f) {
        *illuminance = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }

    // 2. Calculate Weighted IR-Corrected RGB
    // Formula: Lux = (R_Coef * R + G_Coef * G + B_Coef * B) / CPL
    // Note: The coefficients handle the IR rejection mathematically
    const float weighted_rgb = (TCS3472_R_COEF * (float)channels.red) + 
                               (TCS3472_G_COEF * (float)channels.green) + 
                               (TCS3472_B_COEF * (float)channels.blue);

    // 3. Final Lux Calculation
    *illuminance = weighted_rgb / cpl;

    // Lux cannot be negative (can happen due to noise or bad coefficients)
    if (*illuminance < 0.0f) *illuminance = 0.0f;

    return ESP_OK;
}

uint16_t tcs3472_get_ir_light(const tcs3472_channels_data_t channels) {
    // Calculate the sum of RGB channels
    const uint32_t rgb_sum = channels.red + channels.green + channels.blue;

    // If RGB sum > Clear, it suggests spectral overlap or noise 
    // where the IR filter wasn't needed or effective.
    if (channels.clear < rgb_sum) {
        return 0;
    }

    // Basic heuristic: The excess energy in Clear is IR + UV.
    // We divide by 2 to account for the broader bandwidth sensitivity differences.
    return (uint16_t)((channels.clear - rgb_sum) / 2);
}

esp_err_t tcs3472_get_irq_thresholds(tcs3472_handle_t handle, uint16_t *const high_threshold, uint16_t *const low_threshold) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && high_threshold && low_threshold );

    *high_threshold = device->config.irq_high_threshold;
    *low_threshold  = device->config.irq_low_threshold;

    return ESP_OK;
}

esp_err_t tcs3472_set_irq_thresholds(tcs3472_handle_t handle, const uint16_t high_threshold, const uint16_t low_threshold) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to write high and low irq thresholds */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_threshold_hi_register(device, high_threshold), TAG, "write high irq threshold failed" );
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_threshold_lo_register(device, low_threshold), TAG, "write low irq threshold failed" );

    /* set device configuration parameter */
    device->config.irq_high_threshold = high_threshold;
    device->config.irq_low_threshold  = low_threshold;
    device->config.set_irq_thresholds = true;

    return ESP_OK;
}

esp_err_t tcs3472_get_gain_control(tcs3472_handle_t handle, tcs3472_gain_controls_t *const gain) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *gain = device->config.gain_control;

    return ESP_OK;
}

esp_err_t tcs3472_set_gain_control(tcs3472_handle_t handle, const tcs3472_gain_controls_t gain) {
    tcs3472_control_register_t control = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set gain */
    control.bits.gain = gain;

    /* attempt to write control register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_control_register(device, control), TAG, "write control register failed" );

    /* set device configuration parameter */
    device->config.gain_control = gain;

    return ESP_OK;
}

esp_err_t tcs3472_get_integration_time(tcs3472_handle_t handle, float *const time) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *time = device->config.integration_time;

    return ESP_OK;
}

esp_err_t tcs3472_set_integration_time(tcs3472_handle_t handle, const float time) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* convert time (ms) to steps */
    const uint8_t atime = tcs3472_convert_time_to_steps(time, false);

    /* attempt to write atime register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_atime_register(device, atime), TAG, "write atime register failed" );

    /* set device configuration parameter */
    device->config.integration_time = time;

    return ESP_OK;
}

esp_err_t tcs3472_get_wait_time(tcs3472_handle_t handle, float *const time) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set output parameter */
    *time = device->config.wait_time;

    return ESP_OK;
}

esp_err_t tcs3472_set_wait_time(tcs3472_handle_t handle, const float time) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* convert time (ms) to steps */
    const uint8_t wtime = tcs3472_convert_time_to_steps(time, device->config.long_wait_enabled);

    /* attempt to write wtime register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_wtime_register(device, wtime), TAG, "write wtime register failed" );

    /* set device configuration parameter */
    device->config.wait_time = time;

    return ESP_OK;
}

esp_err_t tcs3472_get_data_status(tcs3472_handle_t handle, bool *const ready) {
    tcs3472_status_register_t status = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_status_register(device, &status), TAG, "read status register failed" );

    /* set output parameters */
    *ready = status.bits.data_valid;

    return ESP_OK;
}

esp_err_t tcs3472_get_irq_status(tcs3472_handle_t handle, bool *const asserted) {
    tcs3472_status_register_t status = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_status_register(device, &status), TAG, "read status register failed" );

    /* set output parameters */
    *asserted = status.bits.irq_clear_ch;

    return ESP_OK;
}

esp_err_t tcs3472_get_status(tcs3472_handle_t handle, bool *const data_ready, bool *const irq_asserted) {
    tcs3472_status_register_t status = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_status_register(device, &status), TAG, "read status register failed" );

    /* set output parameters */
    *data_ready = status.bits.data_valid;
    *irq_asserted = status.bits.irq_clear_ch;

    return ESP_OK;
}

esp_err_t tcs3472_enable_wait_time(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable wait time failed" );

    /* initialize config register */
    enable.bits.wait_enabled = true;

    /* attempt to write config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register for enable wait time failed" );

    /* set device configuration parameter */
    device->config.wait_time_enabled = true;

    return ESP_OK;
}

esp_err_t tcs3472_disable_wait_time(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable wait time failed" );

    /* initialize config register */
    enable.bits.wait_enabled = false;

    /* attempt to write config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register for disable wait time failed" );

    /* set device configuration parameter */
    device->config.wait_time_enabled = false;

    return ESP_OK;
}

esp_err_t tcs3472_enable_long_wait_time(tcs3472_handle_t handle) {
    tcs3472_config_register_t config = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_config_register(device, &config), TAG, "read config register for enable long wait time failed" );

    /* initialize config register */
    config.bits.long_wait_enabled = true;

    /* attempt to write config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_config_register(device, config), TAG, "write config register for enable long wait time failed" );

    /* set device configuration parameter */
    device->config.long_wait_enabled = true;

    return ESP_OK;
}

esp_err_t tcs3472_disable_long_wait_time(tcs3472_handle_t handle) {
    tcs3472_config_register_t config = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_config_register(device, &config), TAG, "read config register for disable long wait time failed" );

    /* initialize config register */
    config.bits.long_wait_enabled = false;

    /* attempt to write config register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_config_register(device, config), TAG, "write config register for disable long wait time failed" );

    /* set device configuration parameter */
    device->config.long_wait_enabled = false;

    return ESP_OK;
}

esp_err_t tcs3472_enable_adc(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable RGBC ADC failed" );

    /* initialize enable register */
    enable.bits.adc_enabled = true;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register for enable RGBC ADC failed" );

    /* set device configuration parameter */
    device->config.adc_enabled = true;

    return ESP_OK;
}

esp_err_t tcs3472_disable_adc(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable RGBC ADC failed" );

    /* initialize enable register */
    enable.bits.adc_enabled = false;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register for disable RGBC ADC failed" );

    /* set device configuration parameter */
    device->config.adc_enabled = false;

    return ESP_OK;
}

esp_err_t tcs3472_enable_power(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register for enable power failed" );

    /* initialize enable register */
    enable.bits.power_enabled = true;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register for enable power failed" );

    /* set device configuration parameter */
    device->config.power_enabled = true;

    return ESP_OK;
}

esp_err_t tcs3472_disable_power(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable = { 0 };
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_get_enable_register(device, &enable), TAG, "read enable register for disable power failed" );

    /* initialize enable register */
    enable.bits.power_enabled = false;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_set_enable_register(device, enable), TAG, "write enable register for disable power failed" );

    /* set device configuration parameter */
    device->config.power_enabled = false;

    return ESP_OK;
}

esp_err_t tcs3472_remove(tcs3472_handle_t handle) {
    tcs3472_device_t* device = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* validate handle instance */
    if(device->i2c_handle) {
        /* remove device from i2c master bus */
        esp_err_t ret = i2c_master_bus_rm_device(device->i2c_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_rm_device failed");
            return ret;
        }
        device->i2c_handle = NULL;
    }

    return ESP_OK;
}

esp_err_t tcs3472_delete(tcs3472_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    esp_err_t ret = tcs3472_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}

const char* tcs3472_get_fw_version(void) {
    return (const char*)TCS3472_FW_VERSION_STR;
}

int32_t tcs3472_get_fw_version_number(void) {
    return (int32_t)TCS3472_FW_VERSION_INT32;
}
