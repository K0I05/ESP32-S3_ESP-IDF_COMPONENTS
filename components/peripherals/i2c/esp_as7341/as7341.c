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
#define IAS7341_SP_TH_H_MSB          UINT8_C(0x87)  //!< as7341
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
#define AS7341_TX_RX_DELAY_MS        UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

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
 * @brief Converts an ADC spectral gain sensitivity enumerator to a spectral gain sensitivity multiplier.
 * 
 * @param gain ADC spectral gain sensitivity enumerator.
 * @return float ADC spectral gain sensitivity multiplier.
 */
static inline float as7341_get_spectral_gain_sensitivity(as7341_spectral_gains_t gain) {
    /* determine gain sensitivity */
    switch (gain) {
        case AS7341_SPECTRAL_GAIN_0_5X:
            return 0.5;
        case AS7341_SPECTRAL_GAIN_1X:
            return 1;
        case AS7341_SPECTRAL_GAIN_2X:
            return 2;
        case AS7341_SPECTRAL_GAIN_4X:
            return 4;
        case AS7341_SPECTRAL_GAIN_8X:
            return 8;
        case AS7341_SPECTRAL_GAIN_16X:
            return 16;
        case AS7341_SPECTRAL_GAIN_32X:
            return 32;
        case AS7341_SPECTRAL_GAIN_64X:
            return 64;
        case AS7341_SPECTRAL_GAIN_128X:
            return 128;
        case AS7341_SPECTRAL_GAIN_256X:
            return 256;
        case AS7341_SPECTRAL_GAIN_512X:
            return 512;
        default:
            return 1;
    }
}

/**
 * @brief Calculated integration time (𝑡𝑖𝑛𝑡 = (𝐴𝑇𝐼𝑀𝐸 + 1) × (𝐴𝑆𝑇𝐸𝑃 + 1) × 2.78μ𝑠), from ATIME and ASTEP registers, in milli-seconds.
 * 
 * @param as7341_handle AS7341 device handle.
 * @param time Integration time in milli-seconds
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_get_integration_time(as7341_handle_t handle, float *time) {
    uint8_t atime;
    uint16_t astep;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read astep and atime registers */
    ESP_RETURN_ON_ERROR( as7341_get_astep_register(handle, &astep), TAG, "read astep register for get integration time failed" );
    ESP_RETURN_ON_ERROR( as7341_get_atime_register(handle, &atime), TAG, "read atime register for get integration time failed" );

    /* compute integration time */
    *time = (float)(atime + 1) * (astep + 1) * 2.78f / 1000.0f;

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for low channels F1-F4, Clear and NIR.
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_setup_smux_lo_channels(as7341_device_t *const device) {
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
static inline esp_err_t as7341_setup_smux_hi_channels(as7341_device_t *const device) {
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
 * @brief Configures SMUX registers for low channels (F1-F4, Clear and NIR).
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_set_smux_lo_channels(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( as7341_disable_spectral_measurement(handle), TAG, "disable spectral measurement for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( as7341_set_smux_command(handle, AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( as7341_setup_smux_lo_channels(handle), TAG, "setup SMUX low channels for set SMUX low channels failed" );

    ESP_RETURN_ON_ERROR( as7341_enable_smux(handle), TAG, "enable SMUX for set SMUX low channels failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for high channels (F5-F8, Clear and NIR).
 * 
 * @param as7341_handle AS7341 device handle.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_set_smux_hi_channels(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( as7341_disable_spectral_measurement(handle), TAG, "disable spectral measurement for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( as7341_set_smux_command(handle, AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( as7341_setup_smux_hi_channels(handle), TAG, "setup SMUX high channels for set SMUX high channels failed" );

    ESP_RETURN_ON_ERROR( as7341_enable_smux(handle), TAG, "enable SMUX for set SMUX high channels failed" );

    return ESP_OK;
}

/**
 * @brief Configures SMUX registers for flicker detection.
 * 
 * @param device AS7341 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as7341_setup_smux_flicker_detection(as7341_device_t *const device) {
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

esp_err_t as7341_get_led_register(as7341_handle_t handle, as7341_led_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt to enable low register bank */
    as7341_enable_lo_register_bank(handle);

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_LED, &reg->reg), TAG, "read LED register failed" );

    /* attempt to enable high register bank */
    as7341_enable_hi_register_bank(handle);
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_led_register(as7341_handle_t handle, const as7341_led_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

     /* attempt to enable low register bank */
    as7341_enable_lo_register_bank(handle);
    
    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_LED, reg.reg), TAG, "write LED register failed" );

    /* attempt to enable high register bank */
    as7341_enable_hi_register_bank(handle);
    
    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));
    
    return ESP_OK;
}

esp_err_t as7341_get_astatus_register(as7341_handle_t handle, as7341_astatus_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_ASTATUS, &reg->reg), TAG, "read astatus register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_status2_register(as7341_handle_t handle, as7341_status2_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_STATUS2, &reg->reg), TAG, "read status 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_disable_enable_register(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    as7341_enable_register_t enable = { .reg = 0 };

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "disable enable register failed" );

    return ESP_OK;
}

esp_err_t as7341_get_enable_register(as7341_handle_t handle, as7341_enable_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_ENABLE, &reg->reg), TAG, "read enable register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_enable_register(as7341_handle_t handle, const as7341_enable_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    as7341_enable_register_t enable = { .reg = reg.reg };

    /* set reserved bits */
    enable.bits.reserved1 = 0;
    enable.bits.reserved2 = 0;
    enable.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_ENABLE, enable.reg), TAG, "write enable register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_auxiliary_id_register(as7341_handle_t handle, as7341_auxiliary_id_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_AUXID, &reg->reg), TAG, "read auxiliary id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_revision_id_register(as7341_handle_t handle, as7341_revision_id_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_REVID, &reg->reg), TAG, "read revision id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_part_id_register(as7341_handle_t handle, as7341_part_id_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );


    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_ID, &reg->reg), TAG, "read part id register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_config_register(as7341_handle_t handle, as7341_config_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_CONFIG, &reg->reg), TAG, "read configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_config_register(as7341_handle_t handle, const as7341_config_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    as7341_config_register_t config = { .reg = reg.reg };

    /* set reserved bits */
    config.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_CONFIG, config.reg), TAG, "write configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_config0_register(as7341_handle_t handle, as7341_config0_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_CONFIG0, &reg->reg), TAG, "read configuration 0 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_config0_register(as7341_handle_t handle, const as7341_config0_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    as7341_config0_register_t config0 = { .reg = reg.reg };

    /* set reserved bits */
    config0.bits.reserved1 = 0;
    config0.bits.reserved2 = 0;
    config0.bits.reserved3 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_CONFIG0, config0.reg), TAG, "write configuration 0 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_config1_register(as7341_handle_t handle, as7341_config1_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_CONFIG1, &reg->reg), TAG, "read configuration 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_config1_register(as7341_handle_t handle, const as7341_config1_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    as7341_config1_register_t config1 = { .reg = reg.reg };

    /* set reserved bits */
    config1.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_CONFIG1, config1.reg), TAG, "write configuration 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_config6_register(as7341_handle_t handle, as7341_config6_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_CONFIG6, &reg->reg), TAG, "read configuration 6 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_config6_register(as7341_handle_t handle, const as7341_config6_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    as7341_config6_register_t config6 = { .reg = reg.reg };

    /* set reserved bits */
    config6.bits.reserved1 = 0;
    config6.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_CONFIG6, config6.reg), TAG, "write configuration 6 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_atime_register(as7341_handle_t handle, uint8_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_ATIME, reg), TAG, "read atime register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_atime_register(as7341_handle_t handle, const uint8_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_ATIME, reg), TAG, "write atime register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_astep_register(as7341_handle_t handle, uint16_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_word_from(dev, AS7341_ASTEP_L, reg), TAG, "read astep register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_astep_register(as7341_handle_t handle, const uint16_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_ASTEP_L, reg), TAG, "write astep register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_get_flicker_detection_status_register(as7341_handle_t handle, as7341_flicker_detection_status_register_t *const reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_read_byte_from(dev, AS7341_FD_STATUS, &reg->reg), TAG, "read flicker detection status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_set_flicker_detection_status_register(as7341_handle_t handle, const as7341_flicker_detection_status_register_t reg) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* copy register */
    as7341_flicker_detection_status_register_t flicker_detection_status = { .reg = reg.reg };

    /* set reserved bits */
    flicker_detection_status.bits.reserved = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( as7341_i2c_write_byte_to(dev, AS7341_FD_STATUS, flicker_detection_status.reg), TAG, "write flicker detection status register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    return ESP_OK;
}

esp_err_t as7341_clear_flicker_detection_status_register(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* clear resettable flags */
    as7341_flicker_detection_status_register_t flicker_detection_status = {
        .bits.fd_saturation_detected = true,
        .bits.fd_measurement_valid   = true,
        .bits.fd_100hz_flicker_valid = true,
        .bits.fd_120hz_flicker_valid = true
    };

    /* attempt to set device handle register */
    ESP_RETURN_ON_ERROR( as7341_set_flicker_detection_status_register(handle, flicker_detection_status), TAG, "write flicker detection status register for clear flicker detection status register failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_hi_register_bank(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register from handle */
    as7341_config0_register_t config0_reg;

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_config0_register(handle, &config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    /* enable high registers */
    config0_reg.bits.reg_bank_access = false; // 0 or false to access register 0x80 and above

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_config0_register(handle, config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_lo_register_bank(as7341_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* copy register from handle */
    as7341_config0_register_t config0_reg;

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_config0_register(handle, &config0_reg), TAG, "write configuration 0 register for enable high registers failed" );

    /* enable low registers */
    config0_reg.bits.reg_bank_access = true; // 1 or true to access register 0x60 to 0x74

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_config0_register(handle, config0_reg), TAG, "write configuration 0 register for enable low registers failed" );

    return ESP_OK;
}

esp_err_t as7341_set_smux_command(as7341_handle_t handle, const as7341_smux_commands_t command) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* set smux command */
    as7341_config6_register_t config6_reg = { .bits.smux_command = command };

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_set_config6_register(handle, config6_reg), TAG, "write configuration 6 register for set smux command failed" );

    return ESP_OK;
}

esp_err_t as7341_init(i2c_master_bus_handle_t master_handle, const as7341_config_t *as7341_config, as7341_handle_t *as7341_handle) {
    as7341_part_id_register_t part_id;
    as7341_revision_id_register_t revision_id;

    /* validate arguments */
    ESP_ARG_CHECK( master_handle && as7341_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, as7341_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, as7341 device handle initialization failed", as7341_config->i2c_address);

    /* validate memory availability for handle */
    as7341_device_t* dev = (as7341_device_t*)calloc(1, sizeof(as7341_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as7341 device, init failed");

    /* copy configuration */
    dev->config = *as7341_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c new bus for init failed");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));

    /* attempt to enable power */
    ESP_GOTO_ON_ERROR(as7341_enable_power((as7341_handle_t)dev), err_handle, TAG, "enable power for init failed");

    /* attempt to read part id */
    ESP_GOTO_ON_ERROR(as7341_get_part_id_register((as7341_handle_t)dev, &part_id), err_handle, TAG, "read part id register for init failed");

    /* attempt to read revision id */
    ESP_GOTO_ON_ERROR(as7341_get_revision_id_register((as7341_handle_t)dev, &revision_id), err_handle, TAG, "read revision id register for init failed");

    /* copy configuration */
    dev->part_id = part_id.bits.identifier;
    dev->revision_id = revision_id.bits.identifier;

    /* attempt to write atime configuration */
    ESP_GOTO_ON_ERROR(as7341_set_atime((as7341_handle_t)dev, dev->config.atime), err_handle, TAG, "write atime for init failed");

    /* attempt to write astep configuration */
    ESP_GOTO_ON_ERROR(as7341_set_astep((as7341_handle_t)dev, dev->config.astep), err_handle, TAG, "write astep for init failed");

    /*attempt to write spectral gain configuration */
    ESP_GOTO_ON_ERROR(as7341_set_spectral_gain((as7341_handle_t)dev, dev->config.spectral_gain), err_handle, TAG, "write spectral gain for init failed");

    /* set device handle */
    *as7341_handle = (as7341_handle_t)dev;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t as7341_get_spectral_measurements(as7341_handle_t handle, as7341_channels_spectral_data_t *const spectral_data) {
    esp_err_t   ret              = ESP_OK;
    float       integration_time = 0;
    uint64_t    start_time       = esp_timer_get_time();
    bool        data_is_ready    = false;
    uint8_t     rx[12]           = { 0 };
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt to read integration time */
    ESP_GOTO_ON_ERROR( as7341_get_integration_time(handle, &integration_time), err, TAG, "read integration time, for get adc measurements failed." );

    // ************ LOW CHANNELS ***********

    /* attempt to setup low channels */
    ESP_GOTO_ON_ERROR( as7341_set_smux_lo_channels(handle), err, TAG, "setup of SMUX low channels for get adc measurements failed." );

    /* attempt to enable spectral measurement for low channels */
    ESP_GOTO_ON_ERROR( as7341_enable_spectral_measurement(handle), err, TAG, "enable spectral measurement, low channels, for get adc measurements failed." );

    /* attempt to poll until data, low channels, is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( as7341_get_data_status(handle, &data_is_ready), err, TAG, "data ready read, low channels, for get adc measurements failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AS7341_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (integration_time + 50) * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read spectral adc data from low channels */
    ESP_GOTO_ON_ERROR( as7341_i2c_read_from(dev, AS7341_CH0_ADC_DATA_L, rx, sizeof(rx)), err, TAG, "read low channel measurements for get adc measurements failed" );

    /* set adc data for low channels */
    spectral_data->f1 = (uint16_t)rx[0]  | (uint16_t)(rx[1] << 8);
    spectral_data->f2 = (uint16_t)rx[2]  | (uint16_t)(rx[3] << 8);
    spectral_data->f3 = (uint16_t)rx[4]  | (uint16_t)(rx[5] << 8);
    spectral_data->f4 = (uint16_t)rx[6]  | (uint16_t)(rx[7] << 8);

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS7341_CMD_DELAY_MS));


    // ************ HIGH CHANNELS ***********

    /* attempt to setup low channels */
    ESP_GOTO_ON_ERROR( as7341_set_smux_hi_channels(handle), err, TAG, "setup of SMUX high channels for get adc measurements failed." );

    /* attempt to enable spectral measurement for low channels */
    ESP_GOTO_ON_ERROR( as7341_enable_spectral_measurement(handle), err, TAG, "enable spectral measurement, high channels, for get adc measurements failed." );

    /* reset start time for timeout monitoring and reset data ready flag */
    start_time = esp_timer_get_time();
    data_is_ready = false;

    /* attempt to poll until data, high channels, is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( as7341_get_data_status(handle, &data_is_ready), err, TAG, "data ready read, low channels, for get adc measurements failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(AS7341_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, (integration_time + 50) * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read spectral adc data from high channels */
    ESP_GOTO_ON_ERROR( as7341_i2c_read_from(dev, AS7341_CH0_ADC_DATA_L, rx, sizeof(rx)), err, TAG, "read high channel measurements for get adc measurements failed" );

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
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    as7341_config1_register_t config1;

    /* attempt to read registers (config1) */
    ESP_RETURN_ON_ERROR( as7341_get_config1_register(handle, &config1), TAG, "read configuration 1 register for get integration time failed" );

    /* determine gain sensitivity */
    float gain = as7341_get_spectral_gain_sensitivity(config1.bits.spectral_gain);

    /* compute integration time */
    float integration_time = 0;
    ESP_RETURN_ON_ERROR( as7341_get_integration_time(handle, &integration_time), TAG, "read integration time failed" );

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
    esp_err_t ret           = ESP_OK;
    uint64_t  start_time    = 0;
    bool      data_is_ready = false;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to disable enable register */
    ESP_RETURN_ON_ERROR( as7341_disable_enable_register(handle), TAG, "disable enable register, for get flicker detection status failed." );

    /* attempt to enable power */
    ESP_RETURN_ON_ERROR( as7341_enable_power(handle), TAG, "enable power, for get flicker detection status failed" );

    /* attempt to write SMU configuration from RAM to set SMUX chain registers */
    ESP_RETURN_ON_ERROR( as7341_set_smux_command(handle, AS7341_SMUX_CMD_WRITE), TAG, "write SMUX command for get flicker detection status failed" );

    /* attempt to setup SMUX flicker detection */
    ESP_RETURN_ON_ERROR( as7341_setup_smux_flicker_detection(handle), TAG, "setup SMUX for flicker detection, for get flicker detection status failed" );

    /* attempt to enable SMUX */
    ESP_RETURN_ON_ERROR( as7341_enable_smux(handle), TAG, "enable SMUX, for get flicker detection status failed" );

    /* attempt to enable spectral measurement */
    ESP_RETURN_ON_ERROR( as7341_enable_spectral_measurement(handle), TAG, "enable spectral measurement, for get flicker detection status failed." );

    /* attempt to enable flicker detection */
    ESP_RETURN_ON_ERROR( as7341_enable_flicker_detection(handle), TAG, "enable flicker detection, for get flicker detection status failed" );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time();

    /* attempt to poll until data, high channels, is available or timeout */
    do {

        /* attempt to check if flicker detection measurement is ready */
        ESP_GOTO_ON_ERROR( as7341_get_flicker_detection_status_register(handle, &fd_status), err, TAG, "read flicker detection status register, for get flicker detection status failed" );

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
    ESP_RETURN_ON_ERROR( as7341_disable_flicker_detection(handle), TAG, "disable flicker detection, for get flicker detection status failed" );

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

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( as7341_get_status2_register(handle, &status2), TAG, "read status 2 register (data ready state) failed" );

    /* set ready state */
    *ready = status2.bits.spectral_valid;

    return ESP_OK;
}

esp_err_t as7341_get_atime(as7341_handle_t handle, uint8_t *const atime) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_get_atime_register(handle, atime), TAG, "read atime register for get atime failed" );

    return ESP_OK;
}

esp_err_t as7341_set_atime(as7341_handle_t handle, const uint8_t atime) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_set_atime_register(handle, atime), TAG, "write atime register for set atime failed" );

    return ESP_OK;
}

esp_err_t as7341_get_astep(as7341_handle_t handle, uint16_t *const astep) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_get_astep_register(handle, astep), TAG, "read atime register for get astep failed" );

    return ESP_OK;
}

esp_err_t as7341_set_astep(as7341_handle_t handle, const uint16_t astep) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_set_astep_register(handle, astep), TAG, "write astep register for set astep failed" );

    return ESP_OK;
}

esp_err_t as7341_get_spectral_gain(as7341_handle_t handle, as7341_spectral_gains_t *const gain) {
    as7341_config1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_get_config1_register(handle, &config1), TAG, "read configuration 1 register for get spectral gain failed" );

    /* set output parameter */
    *gain = config1.bits.spectral_gain;

    return ESP_OK;
}

esp_err_t as7341_set_spectral_gain(as7341_handle_t handle, const as7341_spectral_gains_t gain) {
    as7341_config1_register_t config1;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( as7341_get_config1_register(handle, &config1), TAG, "read configuration 1 register for get spectral gain failed" );

    /* set spectral gain */
    config1.bits.spectral_gain = gain;

    /* attempt to set register */
    ESP_RETURN_ON_ERROR( as7341_set_config1_register(handle, config1), TAG, "write configuration 1 register for set spectral gain failed" );

    return ESP_OK;
}

esp_err_t as7341_get_ambient_light_sensing_mode(as7341_handle_t handle, as7341_als_modes_t *const mode) {
    as7341_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_lo_register_bank(handle), TAG, "enable low register bank for get ambient light sensing mode failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( as7341_get_config_register(handle, &config), TAG, "read configuration register for get ambient light sensing mode failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_hi_register_bank(handle), TAG, "enable high register bank for get ambient light sensing mode failed" );

    /* set output parameter */
    *mode = config.bits.irq_mode;

    return ESP_OK;
}

esp_err_t as7341_set_ambient_light_sensing_mode(as7341_handle_t handle, const as7341_als_modes_t mode) {
    as7341_config_register_t config;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_lo_register_bank(handle), TAG, "enable low register bank for set ambient light sensing mode failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_get_config_register(handle, &config), TAG, "read configuration register for set ambient light sensing mode failed" );

    /* set mode */
    config.bits.irq_mode = mode;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_set_config_register(handle, config), TAG, "write configuration register for set ambient light sensing mode failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_hi_register_bank(handle), TAG, "enable high register bank for set ambient light sensing mode failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_flicker_detection(as7341_handle_t handle) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for enable flicker detection failed" );

    /* enable flicker detection */
    enable.bits.flicker_detection_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for enable flicker detection failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_flicker_detection(as7341_handle_t handle) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for disable flicker detection failed" );

    /* disable flicker detection */
    enable.bits.flicker_detection_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for disable flicker detection failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_smux(as7341_handle_t handle) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for enable SMUX failed" );

    /* enable smux */
    enable.bits.smux_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for enable SMUX failed" );

    /* validate SMUX operation completed */
    uint16_t timeout = 1000;
    for (uint16_t time = 0; time < timeout; time++) {
        // The SMUXEN bit is cleared once the SMUX operation is finished
        ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for enable SMUX failed" );

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

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for enable spectral measurement failed" );

    /* enable spectral measurement */
    enable.bits.spectral_measurement_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_spectral_measurement(as7341_handle_t handle) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for disable spectral measurement failed" );

    /* disable spectral measurement */
    enable.bits.spectral_measurement_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for enable spectral measurement failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_power(as7341_handle_t handle) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for enable power failed" );

    /* enable power */
    enable.bits.power_enabled = true;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for enable power failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_power(as7341_handle_t handle) {
    as7341_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read */
    ESP_RETURN_ON_ERROR( as7341_get_enable_register(handle, &enable), TAG, "read enable register for disable power failed" );

    /* disable power */
    enable.bits.power_enabled = false;

    /* attempt to write */
    ESP_RETURN_ON_ERROR( as7341_set_enable_register(handle, enable), TAG, "write enable register for disable power failed" );

    return ESP_OK;
}

esp_err_t as7341_enable_led(as7341_handle_t handle) {
    as7341_config_register_t config;
    as7341_led_register_t    led;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_lo_register_bank(handle), TAG, "enable low register bank for enable LED failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( as7341_get_config_register(handle, &config), TAG, "read configuration register for enable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_get_led_register(handle, &led), TAG, "read led register for enable LED failed" );

    /* enable led */
    config.bits.led_ldr_control_enabled = true;
    led.bits.led_ldr_enabled            = true;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_set_config_register(handle, config), TAG, "write configuration register for enable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_set_led_register(handle, led), TAG, "write led register for enable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_hi_register_bank(handle), TAG, "enable high register bank for enable LED failed" );

    return ESP_OK;
}

esp_err_t as7341_disable_led(as7341_handle_t handle) {
    as7341_config_register_t config;
    as7341_led_register_t    led;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to enable low register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_lo_register_bank(handle), TAG, "enable low register bank for disable LED failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( as7341_get_config_register(handle, &config), TAG, "read configuration register for disable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_get_led_register(handle, &led), TAG, "read led register for disable LED failed" );

    /* enable led */
    config.bits.led_ldr_control_enabled = false;
    led.bits.led_ldr_enabled            = false;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( as7341_set_config_register(handle, config), TAG, "write configuration register for disable LED failed" );

    /* attempt to write to led register*/
    ESP_RETURN_ON_ERROR( as7341_set_led_register(handle, led), TAG, "write led register for disable LED failed" );

    /* attempt to enable high register bank */
    ESP_RETURN_ON_ERROR( as7341_enable_hi_register_bank(handle), TAG, "enable high register bank for disable LED failed" );

    return ESP_OK;
}

esp_err_t as7341_remove(as7341_handle_t handle) {
    as7341_device_t* dev = (as7341_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(dev->i2c_handle);
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