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
 * ESP-IDF driver for OSRAM TCS3472 RGBC sensor
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


#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))

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
    const bit8_uint8_buffer_t tx = { reg_addr };

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
    const bit8_uint8_buffer_t tx = { reg_addr };
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
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "tcs3472_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

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
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Gets calculated total wait time (initialization, integration time and wait time) in milliseconds from device descriptor.
 * 
 * @param device TCS3472 device descriptor.
 * @return size_t Total wait time (initialization, integration time and wait time) in milliseconds.
 */
static inline size_t tcs3472_get_total_wait_time_ms(tcs3472_device_t *const device) {
    // ATIME = 256 − Integration Time / 2.4 ms
    // Integration Time = 2.4 ms × (256 − ATIME)

    const float init_time = 2.4f;
    const float time_step = 2.4f;
    float wlong_multiplier = 1.0f;
    uint8_t atime_step = time_step;
    uint8_t wtime_step;
    tcs3472_config_register_t config;

    /* validate arguments */
    if (!device) return time_step;

    /* attempt to read device atime register */
    ESP_RETURN_ON_ERROR( tcs3472_get_atime_register((tcs3472_handle_t)device, &atime_step), TAG, "read atime register failed" );

    /* attempt to read device wtime register */
    ESP_RETURN_ON_ERROR( tcs3472_get_wtime_register((tcs3472_handle_t)device, &wtime_step), TAG, "read wtime register failed" );

    /* attempt to read device configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_get_config_register((tcs3472_handle_t)device, &config), TAG, "read configuration register failed" );

    /* validate wait long time multiplier */
    if(config.bits.long_wait_enabled) {
        wlong_multiplier = 12.0f;
    }

    /* calculate integration and wait times in milliseconds */
    float atime = time_step * (256.0f - atime_step);
    float wtime = wlong_multiplier * (time_step * (256.0f - wtime_step));

    /* calculate integration time in milliseconds */
    return init_time + atime + wtime;
}

/**
 * @brief Setup and configure TCS3472 registers.
 * 
 * @param device TCS3472 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tcs3472_setup(tcs3472_device_t *const device) {
    tcs3472_enable_register_t       enable;
    tcs3472_config_register_t       config;
    tcs3472_persistence_register_t  persist;
    tcs3472_control_register_t      control;
    uint16_t                        irq_high_threshold;
    uint16_t                        irq_low_threshold;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_get_enable_register((tcs3472_handle_t)device, &enable), TAG, "read enable register failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_get_config_register((tcs3472_handle_t)device, &config), TAG, "read configuration register failed" );

    /* attempt to read persistence register */
    ESP_RETURN_ON_ERROR( tcs3472_get_persistence_register((tcs3472_handle_t)device, &persist), TAG, "read persistence register failed" );

    /* attempt to read control register */
    ESP_RETURN_ON_ERROR( tcs3472_get_control_register((tcs3472_handle_t)device, &control), TAG, "read control register failed" );

    /* configure enable register from device configuration structure */
    enable.bits.adc_enabled       = true;
    enable.bits.power_enabled     = device->config.power_enabled;
    enable.bits.irq_enabled       = device->config.irq_enabled;
    enable.bits.wait_enabled      = device->config.wait_time_enabled;
    enable.bits.reserved1         = 0;
    enable.bits.reserved2         = 0;

    /* configure configuration register from device configuration structure */
    config.bits.long_wait_enabled = device->config.long_wait_enabled;
    config.bits.reserved1         = 0;

    /* configure persistence register from device configuration structure */
    persist.bits.irq_control_rate = device->config.irq_control_rate;
    persist.bits.reserved1        = 0;

    /* configure control register from device configuration structure */
    control.bits.gain             = device->config.gain_control;
    control.bits.reserved1        = 0;

    /* attempt to configure atime register from device configuration structure */
    ESP_RETURN_ON_ERROR( tcs3472_set_integration_time((tcs3472_handle_t)device, device->config.integration_time), TAG, "write integration time failed" );

    /* attempt to configure wtime register from device configuration structure */
    ESP_RETURN_ON_ERROR( tcs3472_set_wait_time((tcs3472_handle_t)device, device->config.wait_time), TAG, "write RGBC wait time failed" );

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_set_enable_register((tcs3472_handle_t)device, enable), TAG, "write enable register failed" );

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_set_config_register((tcs3472_handle_t)device, config), TAG, "write configuration register failed" );

    /* attempt to write persistence register */
    ESP_RETURN_ON_ERROR( tcs3472_set_persistence_register((tcs3472_handle_t)device, persist), TAG, "write persistence register failed" );

    /* attempt to write control register */
    ESP_RETURN_ON_ERROR( tcs3472_set_control_register((tcs3472_handle_t)device, control), TAG, "write control register failed" );

    /* attempt to write high and low thresholds */
    if(device->config.set_irq_thresholds) {
        ESP_RETURN_ON_ERROR( tcs3472_set_irq_thresholds((tcs3472_handle_t)device, device->config.irq_high_threshold, device->config.irq_low_threshold), TAG, "write high and low irq thresholds failed" );
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
    esp_err_t ret            = ESP_OK;
    uint64_t  start_time     = esp_timer_get_time();
    bool      data_is_ready  = false;
    uint8_t   hi_reg;
    uint8_t   lo_reg;
    //bit8_uint8_buffer_t rx_hi_byte;
    //bit8_uint8_buffer_t rx_lo_byte;
    uint16_t  rx_count;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set overall wait time */
    float wait_time = tcs3472_get_total_wait_time_ms(device);

    /* set high and low data registers by RGBC channel type */
    switch(channel) {
        case TCS3472_CHANNEL_RED:
            hi_reg = TCS3472_REG_RDATAH_R;
            lo_reg = TCS3472_REG_RDATAL_R;
            break;
        case TCS3472_CHANNEL_GREEN:
            hi_reg = TCS3472_REG_GDATAH_R;
            lo_reg = TCS3472_REG_GDATAL_R;
            break;
        case TCS3472_CHANNEL_BLUE:
            hi_reg = TCS3472_REG_BDATAH_R;
            lo_reg = TCS3472_REG_BDATAL_R;  
            break;
        case TCS3472_CHANNEL_CLEAR:
            hi_reg = TCS3472_REG_CDATAH_R;
            lo_reg = TCS3472_REG_CDATAL_R;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* attempt to poll high and low channel data from device until data is available or timeout */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( tcs3472_get_data_status((tcs3472_handle_t)device, &data_is_ready), err, TAG, "data ready ready for get channel count failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(TCS3472_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, wait_time * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // attempt to read low channel byte from device.
    //ESP_GOTO_ON_ERROR( tcs3472_i2c_read_byte_from(device, lo_reg, &rx_lo_byte), err, TAG, "read RGBC low channel data failed." );
    // attempt to read high channel byte from device.
    //ESP_GOTO_ON_ERROR( tcs3472_i2c_read_byte_from(device, hi_reg, &rx_hi_byte), err, TAG, "read RGBC high channel data failed." );
    /* set output parameter */
    //*count = (uint16_t)rx_lo_byte[0] | ((uint16_t)rx_hi_byte[0] << 8);

    // attempt to read low and high channels from device.
    ESP_GOTO_ON_ERROR( tcs3472_i2c_read_word_from(device, lo_reg, &rx_count), err, TAG, "read RGBC low and high data channels failed." );

    /* set output parameter */
    *count = rx_count;

    return ESP_OK;

    err:
        return ret;
}

esp_err_t tcs3472_get_enable_register(tcs3472_handle_t handle, tcs3472_enable_register_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_ENABLE_RW, &reg->reg), TAG, "read enable register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_set_enable_register(tcs3472_handle_t handle, const tcs3472_enable_register_t reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(dev, TCS3472_REG_ENABLE_RW, reg.reg), TAG, "write enable register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_atime_register(tcs3472_handle_t handle, uint8_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_ATIME_RW, reg), TAG, "read atime register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_set_atime_register(tcs3472_handle_t handle, const uint8_t reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(dev, TCS3472_REG_ATIME_RW, reg), TAG, "write atime register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_wtime_register(tcs3472_handle_t handle, uint8_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_WTIME_RW, reg), TAG, "read wtime register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_set_wtime_register(tcs3472_handle_t handle, const uint8_t reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(dev, TCS3472_REG_WTIME_RW, reg), TAG, "write wtime register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_persistence_register(tcs3472_handle_t handle, tcs3472_persistence_register_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_PERS_RW, &reg->reg), TAG, "read persistence register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_set_persistence_register(tcs3472_handle_t handle, const tcs3472_persistence_register_t reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(dev, TCS3472_REG_PERS_RW, reg.reg), TAG, "write persistence register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_config_register(tcs3472_handle_t handle, tcs3472_config_register_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_CONFIG_RW, &reg->reg), TAG, "read configuration register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_set_config_register(tcs3472_handle_t handle, const tcs3472_config_register_t reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(dev, TCS3472_REG_CONFIG_RW, reg.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_control_register(tcs3472_handle_t handle, tcs3472_control_register_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_CONTROL_RW, &reg->reg), TAG, "read control register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_set_control_register(tcs3472_handle_t handle, const tcs3472_control_register_t reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_write_byte_to(dev, TCS3472_REG_CONTROL_RW, reg.reg), TAG, "write control register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_status_register(tcs3472_handle_t handle, tcs3472_status_register_t *const reg) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev && reg );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tcs3472_i2c_read_byte_from(dev, TCS3472_REG_STATUS_R, &reg->reg), TAG, "read status register failed" );

    return ESP_OK;
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
    tcs3472_device_t* dev = (tcs3472_device_t*)calloc(1, sizeof(tcs3472_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c0 tcs3472 device for init");

    /* copy configuration */
    dev->config = *tcs3472_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c0 new bus failed for init");
    }

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TCS3472_CMD_DELAY_MS));

    /* attempt to reset the device and initialize registers */
    ESP_GOTO_ON_ERROR(tcs3472_setup(dev), err_handle, TAG, "setup registers for init failed");

    /* set output parameter */
    *tcs3472_handle = (tcs3472_handle_t)dev;

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TCS3472_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t tcs3472_get_channels(tcs3472_handle_t handle, tcs3472_channels_data_t *const data) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && data );

    /* attempt to read red channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_red_channel_count(handle, &data->red), TAG, "read red channel count failed" );

    /* attempt to read green channel count */ 
    ESP_RETURN_ON_ERROR( tcs3472_get_green_channel_count(handle, &data->green), TAG, "read green channel count failed" );

    /* attempt to read blue channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_blue_channel_count(handle, &data->blue), TAG, "read blue channel count failed" );

    /* attempt to read clear channel count */
    ESP_RETURN_ON_ERROR( tcs3472_get_clear_channel_count(handle, &data->clear), TAG, "read clear channel count failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_red_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    return tcs3472_get_channel_count(handle, TCS3472_CHANNEL_RED, count);
}

esp_err_t tcs3472_get_green_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    return tcs3472_get_channel_count(handle, TCS3472_CHANNEL_GREEN, count);
}

esp_err_t tcs3472_get_blue_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    return tcs3472_get_channel_count(handle, TCS3472_CHANNEL_BLUE, count);
}

esp_err_t tcs3472_get_clear_channel_count(tcs3472_handle_t handle, uint16_t *const count) {
    return tcs3472_get_channel_count(handle, TCS3472_CHANNEL_CLEAR, count);
}

esp_err_t tcs3472_get_irq_thresholds(tcs3472_handle_t handle, uint16_t *const high_threshold, uint16_t *const low_threshold) {
    return ESP_OK;
}

esp_err_t tcs3472_set_irq_thresholds(tcs3472_handle_t handle, const uint16_t high_threshold, const uint16_t low_threshold) {
    return ESP_OK;
}

esp_err_t tcs3472_get_gain_control(tcs3472_handle_t handle, tcs3472_gain_controls_t *const gain) {
    tcs3472_control_register_t control;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read control register */
    ESP_RETURN_ON_ERROR( tcs3472_get_control_register(handle, &control), TAG, "read control register failed" );

    /* set output parameter */
    *gain = control.bits.gain;

    return ESP_OK;
}

esp_err_t tcs3472_set_gain_control(tcs3472_handle_t handle, const tcs3472_gain_controls_t gain) {
    tcs3472_control_register_t control;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* set gain */
    control.bits.gain = gain;

    /* attempt to write control register */
    ESP_RETURN_ON_ERROR( tcs3472_set_control_register(handle, control), TAG, "write control register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_integration_time(tcs3472_handle_t handle, float *const time) {
    uint8_t atime;

    // Integration Time = 2.4 ms × (256 − ATIME)

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read atime register */
    ESP_RETURN_ON_ERROR( tcs3472_get_atime_register(handle, &atime), TAG, "read atime register failed" );

    *time = 2.4f * (256.0f - (float)atime);

    return ESP_OK;
}

esp_err_t tcs3472_set_integration_time(tcs3472_handle_t handle, const float time) {
    uint8_t atime;

    // ATIME = 256 − Integration Time / 2.4 ms

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    atime = 256 - (time / (2.4f));

    /* attempt to write atime register */
    ESP_RETURN_ON_ERROR( tcs3472_set_atime_register(handle, atime), TAG, "write atime register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_wait_time(tcs3472_handle_t handle, float *const time) {
    tcs3472_config_register_t config;
    uint8_t wtime;
    float wlong_multiplier = 1.0f;

    // Integration Time = 2.4 ms × (256 − ATIME)

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_get_config_register(handle, &config), TAG, "read configuration register failed" );

    /* attempt to read wtime register */
    ESP_RETURN_ON_ERROR( tcs3472_get_wtime_register(handle, &wtime), TAG, "read wtime register failed" );

    /* validate wait long time multiplier */
    if(config.bits.long_wait_enabled) {
        wlong_multiplier = 12.0f;
    }

    /* set output parameter */
    *time = wlong_multiplier * (2.4f * (256.0f - (float)wtime));

    return ESP_OK;
}

esp_err_t tcs3472_set_wait_time(tcs3472_handle_t handle, const float time) {
    tcs3472_config_register_t config;
    uint8_t wtime;
    float wlong_multiplier = 1.0f;

    // ATIME = 256 − Integration Time / 2.4 ms

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( tcs3472_get_config_register(handle, &config), TAG, "read configuration register failed" );

    /* attempt to read wtime register */
    ESP_RETURN_ON_ERROR( tcs3472_get_wtime_register(handle, &wtime), TAG, "read wtime register failed" );

    /* validate wait long time multiplier */
    if(config.bits.long_wait_enabled) {
        wlong_multiplier = 12.0f;
    }

    wtime = 256 - (time / (2.4f * wlong_multiplier));

    /* attempt to write wtime register */
    ESP_RETURN_ON_ERROR( tcs3472_set_wtime_register(handle, wtime), TAG, "write wtime register failed" );

    return ESP_OK;
}

esp_err_t tcs3472_get_data_status(tcs3472_handle_t handle, bool *const ready) {
    tcs3472_status_register_t status;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( tcs3472_get_status_register(handle, &status), TAG, "read status register failed" );

    /* set output parameters */
    *ready = status.bits.data_valid;

    return ESP_OK;
}

esp_err_t tcs3472_get_status(tcs3472_handle_t handle, bool *const data_ready, bool *const irq_asserted) {
    tcs3472_status_register_t status;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( tcs3472_get_status_register(handle, &status), TAG, "read status register failed" );

    /* set output parameters */
    *data_ready = status.bits.data_valid;
    *irq_asserted = status.bits.irq_clear_ch;

    return ESP_OK;
}

esp_err_t tcs3472_enable_power(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_get_enable_register(handle, &enable), TAG, "read enable register for enable power failed" );

    /* initialize enable register */
    enable.bits.power_enabled = true;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_set_enable_register(handle, enable), TAG, "write enable register for enable power failed" );

    return ESP_OK;
}

esp_err_t tcs3472_disable_power(tcs3472_handle_t handle) {
    tcs3472_enable_register_t enable;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read enable register */
    ESP_RETURN_ON_ERROR( tcs3472_get_enable_register(handle, &enable), TAG, "read enable register for disable power failed" );

    /* initialize enable register */
    enable.bits.power_enabled = false;

    /* attempt to write enable register */
    ESP_RETURN_ON_ERROR( tcs3472_set_enable_register(handle, enable), TAG, "write enable register for disable power failed" );

    return ESP_OK;
}

esp_err_t tcs3472_remove(tcs3472_handle_t handle) {
    tcs3472_device_t* dev = (tcs3472_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t tcs3472_delete(tcs3472_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( tcs3472_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* tcs3472_get_fw_version(void) {
    return (const char*)TCS3472_FW_VERSION_STR;
}

int32_t tcs3472_get_fw_version_number(void) {
    return (int32_t)TCS3472_FW_VERSION_INT32;
}
