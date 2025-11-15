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
 * @file tlv493d.c
 *
 * ESP-IDF driver for TLV493D magnetic sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/tlv493d.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * TLV493D definitions
 */

#define TLV493D_REG_BX_MSB_R          UINT8_C(0x00)
#define TLV493D_REG_BY_MSB_R          UINT8_C(0x01)
#define TLV493D_REG_BZ_MSB_R          UINT8_C(0x02)
#define TLV493D_REG_TEMP_MSB_R        UINT8_C(0x03)
#define TLV493D_REG_BX_BY_LSB_R       UINT8_C(0x04)
#define TLV493D_REG_BZ_LSB_R          UINT8_C(0x05)
#define TLV493D_REG_TEMP_LSB_R        UINT8_C(0x06)
#define TLV493D_REG_FACTSET1_R        UINT8_C(0x07)  // factory setting for write register 0x01 (bits: 4-3 device specific)
#define TLV493D_REG_FACTSET2_R        UINT8_C(0x08)  // factory setting for write register 0x02
#define TLV493D_REG_FACTSET3_R        UINT8_C(0x09)  // factory setting for write register 0x03 (bits: 4-0 device specific)
#define TLV493D_REG_MOD1_W            UINT8_C(0x01)  // set bits 4-3 (device specific) from read register 0x07
#define TLV493D_REG_RESERVED2_W       UINT8_C(0x02)  // set bits from read register 0x08
#define TLV493D_REG_MOD2_W            UINT8_C(0x03)  // set bits 4-0 (device specific) from read register 0x09

#define TLV493D_DATA_POLL_TIMEOUT_MS  UINT16_C(1000)
#define TLV493D_DATA_READY_DELAY_MS   UINT16_C(2)
#define TLV493D_POWERUP_DELAY_MS      UINT16_C(120)
#define TLV493D_RESET_DELAY_MS        UINT16_C(25)
#define TLV493D_SETUP_DELAY_MS        UINT16_C(15)
#define TLV493D_APPSTART_DELAY_MS     UINT16_C(25)    /*!< delay after initialization before application start-up */
#define TLV493D_CMD_DELAY_MS          UINT16_C(5)     /*!< delay before attempting I2C transactions after a command is issued */
#define TLV493D_TX_RX_DELAY_MS        UINT16_C(10)    /*!< delay after attempting an I2C transmit transaction and attempting an I2C receive transaction */

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/**
 * @brief TLV493D device descriptor structure definition.
 */
typedef struct tlv493d_device_s {
    tlv493d_config_t config; /*!< tlv493d device configuration */
    i2c_master_dev_handle_t     i2c_handle; /*!< tlv493d I2C device handle */
    tlv493d_factory_setting1_register_t factory_setting1_reg;
    tlv493d_factory_setting2_register_t factory_setting2_reg;
    tlv493d_factory_setting3_register_t factory_setting3_reg;
} tlv493d_device_t;

/*
* static constant declarations
*/
static const char *TAG = "tlv493d";

/**
 * @brief TLV493D I2C HAL read byte from register address transaction.
 * 
 * @param device TLV493D device descriptor.
 * @param reg_addr TLV493D register address to read from.
 * @param byte TLV493D read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tlv493d_i2c_read_byte_from(tlv493d_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c read from failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_TX_RX_DELAY_MS));

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( i2c_master_receive(device->i2c_handle, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_receive, i2c read from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief TLV493D I2C HAL write byte to register address transaction.
 * 
 * @param device TLV493D device descriptor.
 * @param reg_addr TLV493D register address to write to.
 * @param byte TLV493D write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t tlv493d_i2c_write_byte_to(tlv493d_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write to failed" );
                        
    return ESP_OK;
}

static inline uint8_t tlv493d_calculate_parity(uint8_t data) {
    uint8_t out = data;
	out ^= out >> 4;
	out ^= out >> 2;
	out ^= out >> 1;
	return out & 1U;
}

static inline uint8_t tlv493d_get_odd_parity(uint8_t parity) {
    return (parity ^ 1U) & 1U;
}

static inline uint8_t tlv493d_get_even_parity(uint8_t parity) {
    return parity & 1U;
}


static inline esp_err_t tlv493d_get_temperature_msb_register(tlv493d_handle_t handle, tlv493d_temperature_msb_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_read_byte_from(dev, TLV493D_REG_TEMP_MSB_R, &reg->reg), TAG, "read bz, t, ff, and pd register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t tlv493d_get_bz_lsb_register(tlv493d_handle_t handle, tlv493d_bz_lsb_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_read_byte_from(dev, TLV493D_REG_BZ_LSB_R, &reg->reg), TAG, "read bz, t, ff, and pd register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t tlv493d_get_factory_setting1_register(tlv493d_handle_t handle, tlv493d_factory_setting1_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_read_byte_from(dev, TLV493D_REG_FACTSET1_R, &reg->reg), TAG, "read factory setting 1 configuration register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t tlv493d_get_factory_setting2_register(tlv493d_handle_t handle, tlv493d_factory_setting2_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_read_byte_from(dev, TLV493D_REG_FACTSET2_R, &reg->reg), TAG, "read factory setting 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t tlv493d_get_factory_setting3_register(tlv493d_handle_t handle, tlv493d_factory_setting3_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_read_byte_from(dev, TLV493D_REG_FACTSET3_R, &reg->reg), TAG, "read factory setting 3 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}

static inline esp_err_t tlv493d_set_mode1_register(tlv493d_handle_t handle, const tlv493d_mode1_register_t reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_write_byte_to(dev, TLV493D_REG_MOD1_W, reg.reg), TAG, "write mode 1 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}


static inline esp_err_t tlv493d_set_mode2_register(tlv493d_handle_t handle, const tlv493d_mode2_register_t reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( tlv493d_i2c_write_byte_to(dev, TLV493D_REG_MOD2_W, reg.reg), TAG, "write mode 2 register failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;
}

static inline int16_t tlv493d_concat_12bit_data(const uint8_t msb, const uint8_t lsb) {
    int16_t value = 0x0000;	//16-bit signed integer for 12-bit values of sensor
    value=(msb&0x0F)<<12;
	value|=lsb<<4;
    value>>=4;				//shift left so that value is a signed 12 bit integer
	return value;
}

static inline esp_err_t tlv493d_get_fixed_magnetic_axes(tlv493d_handle_t handle, tlv493d_data_signal_t *const raw_data) {
    esp_err_t               ret             = ESP_OK;
    //uint64_t                start_time      = 0;
    //bool                    data_is_ready   = false;
    const bit8_uint8_buffer_t tx_buffer = { TLV493D_REG_BX_MSB_R };
    bit56_uint8_buffer_t      rx_buffer	= { 0 };
    tlv493d_data_signal_t    out_data;
    tlv493d_temperature_msb_register_t  temperature_msb_reg;
    tlv493d_bx_by_lsb_register_t        bx_by_lsb_reg;
    tlv493d_bz_lsb_register_t           bz_lsb_reg;
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;


    /* validate arguments */
    ESP_ARG_CHECK( dev );

    /* set start time (us) for timeout monitoring */
    //start_time = esp_timer_get_time(); 

    /* attempt to poll until data is available or timeout */
   // do {
        /* attempt to poll if data is ready or timeout */
    //    ESP_GOTO_ON_ERROR( i2c_tlv493d_get_data_status(tlv493d_handle, &data_is_ready), err, TAG, "data ready read for measurement failed." );

        /* delay task before next i2c transaction */
     //   vTaskDelay(pdMS_TO_TICKS(I2C_TLV493D_DATA_READY_DELAY_MS));

        /* validate timeout condition */
    //    if (ESP_TIMEOUT_CHECK(start_time, (I2C_TLV493D_DATA_POLL_TIMEOUT_MS * 1000)))
    //        return ESP_ERR_TIMEOUT;
    //} while (data_is_ready == false);

    vTaskDelay(pdMS_TO_TICKS(50));

    /* attempt i2c write and read transaction */
    //ESP_GOTO_ON_ERROR( i2c_master_transmit_receive(tlv493d_handle->i2c_dev_handle, tx_buffer, I2C_UINT8_SIZE, rx_buffer, I2C_UINT56_SIZE, I2C_XFR_TIMEOUT_MS), err, TAG, "unable to write to i2c device handle, get measurement failed");
	
    ESP_GOTO_ON_ERROR( i2c_master_transmit(dev->i2c_handle, tx_buffer, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), err, TAG, "unable to write to i2c device handle, get measurement failed");
	
    vTaskDelay(pdMS_TO_TICKS(5));
    
    ESP_GOTO_ON_ERROR( i2c_master_receive(dev->i2c_handle, rx_buffer, BIT56_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), err, TAG, "unable to read from i2c device handle, get measurement failed");
	

    //data = rx[0] | (rx[1] << 8);
    temperature_msb_reg.reg = rx_buffer[3];
    bx_by_lsb_reg.reg = rx_buffer[4];
    bz_lsb_reg.reg = rx_buffer[5];
    out_data.x_axis = tlv493d_concat_12bit_data(rx_buffer[0], bx_by_lsb_reg.bits.bx_lsb);
    out_data.y_axis = tlv493d_concat_12bit_data(rx_buffer[1], bx_by_lsb_reg.bits.by_lsb);
    out_data.z_axis = tlv493d_concat_12bit_data(rx_buffer[2], bz_lsb_reg.bits.bz_lsb);
    out_data.temperature = tlv493d_concat_12bit_data(temperature_msb_reg.bits.temperature_msb, rx_buffer[6]);
    out_data.temperature_enabled = true;



    //out_data.x_axis = i2c_tlv493d_concat_12bit_data(tx_buffer[0], tx_buffer[4]);
    //out_data.y_axis = i2c_tlv493d_concat_12bit_data(tx_buffer[1], tx_buffer[4]);
    //out_data.z_axis = i2c_tlv493d_concat_12bit_data(tx_buffer[2], tx_buffer[5]);
    //out_data.temperature = i2c_tlv493d_concat_12bit_data(tx_buffer[3], tx_buffer[6]);

    //out_data.x_axis = i2c_tlv493d_concat_12bit_data(bx_by_lsb_reg.bits.bx_lsb, rx_buffer[0]);
    //out_data.y_axis = i2c_tlv493d_concat_12bit_data(bx_by_lsb_reg.bits.by_lsb, rx_buffer[1]);
    //out_data.z_axis = i2c_tlv493d_concat_12bit_data(bz_lsb_reg.bits.bz_lsb, rx_buffer[2]);
    //out_data.temperature = i2c_tlv493d_concat_12bit_data(rx_buffer[6], temperature_msb_reg.bits.temperature_msb);

    ESP_LOGW(TAG, "x-axis 0x%02x | 0x%02x", tx_buffer[0], bx_by_lsb_reg.bits.bx_lsb);
    ESP_LOGW(TAG, "y-axis 0x%02x | 0x%02x", tx_buffer[1], bx_by_lsb_reg.bits.by_lsb);
    ESP_LOGW(TAG, "z-axis 0x%02x | 0x%02x", tx_buffer[2], bz_lsb_reg.bits.bz_lsb);

    //ESP_LOGW(TAG, "x-axis: %d", out_data.x_axis);
    //ESP_LOGW(TAG, "y-axis: %d", out_data.y_axis);
    //ESP_LOGW(TAG, "z-axis: %d", out_data.z_axis);
    //ESP_LOGW(TAG, "temp:   %d", out_data.temperature);

    /* set raw data */
    *raw_data = out_data;

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    return ESP_OK;

    err:
        return ret;
}


static inline esp_err_t tlv493d_configure_power_mode1_register(tlv493d_handle_t handle, tlv493d_mode1_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    switch(dev->config.power_mode) {
        case TLV493D_POWER_DOWN_MODE:
            reg->bits.fast_mode_enabled      = false;
            reg->bits.low_power_mode_enabled = false;
            break;
        case TLV493D_FAST_MODE:
            reg->bits.fast_mode_enabled      = true;
            reg->bits.low_power_mode_enabled = false;
            break;
        case TLV493D_LOW_POWER_MODE:
        case TLV493D_ULTRA_LOW_POWER_MODE:
            reg->bits.fast_mode_enabled      = false;
            reg->bits.low_power_mode_enabled = true;
            break;
        case TLV493D_MASTER_CONTROLLED_MODE:
            reg->bits.fast_mode_enabled      = true;
            reg->bits.low_power_mode_enabled = true;
            break;
    }

    return ESP_OK;
}

static inline esp_err_t tlv493d_configure_power_mode2_register(tlv493d_handle_t handle, tlv493d_mode2_register_t *const reg) {
    tlv493d_device_t* dev = (tlv493d_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    switch(dev->config.power_mode) {
        case TLV493D_POWER_DOWN_MODE:
            reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_100MS;
            break;
        case TLV493D_FAST_MODE:
            reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_100MS;
            break;
        case TLV493D_LOW_POWER_MODE:
            reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_12MS;
            break;
        case TLV493D_ULTRA_LOW_POWER_MODE:
            reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_100MS;
            break;
        case TLV493D_MASTER_CONTROLLED_MODE:
            reg->bits.low_power_period = TLV493D_LOW_POWER_PERIOD_12MS;
            break;
    }

    return ESP_OK;
}

esp_err_t tlv493d_init(i2c_master_bus_handle_t master_handle, const tlv493d_config_t *tlv493d_config, tlv493d_handle_t *tlv493d_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && tlv493d_config );

    /* power-up task delay */
    //vTaskDelay(pdMS_TO_TICKS(I2C_TLV493D_POWERUP_DELAY_MS));
    vTaskDelay(pdMS_TO_TICKS(200));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, tlv493d_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, tlv493d device handle initialization failed", tlv493d_config->i2c_address);

    /* validate memory availability for handle */
    tlv493d_device_t* dev = (tlv493d_device_t*)calloc(1, sizeof(tlv493d_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c tlv493d device, init failed");

    /* copy configuration */
    dev->config = *tlv493d_config;

    /* set device configuration */
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
    vTaskDelay(pdMS_TO_TICKS(TLV493D_CMD_DELAY_MS));

    /* attempt i2c recovery transaction */
    //ESP_GOTO_ON_ERROR( i2c_master_bus_write_cmd(out_handle->i2c_dev_handle, 0xFF), err_handle, TAG, "write to recovery register failed" );

    /* delay before next i2c transaction */
    //vTaskDelay(pdMS_TO_TICKS(25));

    /* attempt i2c recovery transaction */
    //ESP_GOTO_ON_ERROR( i2c_master_bus_write_cmd(out_handle->i2c_handle, 0x00), err_handle, TAG, "reset failed" );

    /* delay before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(250));


    /* attempt to read factory setting registers */
    //ESP_GOTO_ON_ERROR(tlv493d_get_factorysetting_registers(out_handle), err_handle, TAG, "read factory setting registers for init failed");

    //tlv493d_reserved1_register_t    reserved1_reg;
    tlv493d_mode1_register_t        mode1_reg;
    //tlv493d_reserved2_register_t    reserved2_reg;
    tlv493d_mode2_register_t        mode2_reg;

    //reserved1_reg.reg                   = 0;
    //mode1_reg.bits.factory_setting      = out_handle->factorysetting1_reg.bits.factory_setting;
    //mode1_reg.bits.irq_pin_enabled      = out_handle->irq_pin_enabled;
    mode1_reg.bits.i2c_slave_address    = TLV493D_I2C_ADDRESS_00;
    ESP_GOTO_ON_ERROR(tlv493d_configure_power_mode1_register((tlv493d_handle_t)dev, &mode1_reg), err_handle, TAG, "configure power mode 1 register for init failed");
    //reserved2_reg.bits.factory_setting  = out_handle->factorysetting2_reg.bits.factory_setting;
    //mode2_reg.bits.factory_setting      = out_handle->factorysetting3_reg.bits.factory_setting;
    //mode2_reg.bits.parity_test_enabled  = out_handle->parity_test_enabled;
    //mode2_reg.bits.temperature_disabled = out_handle->temperature_disabled;
    ESP_GOTO_ON_ERROR(tlv493d_configure_power_mode2_register((tlv493d_handle_t)dev, &mode2_reg), err_handle, TAG, "configure power mode 2 register for init failed");

    ESP_GOTO_ON_ERROR(tlv493d_set_mode2_register((tlv493d_handle_t)dev, mode2_reg), err_handle, TAG, "write mode 2 register for init failed");

    /* sum registers */
    uint8_t result = 0x00;
    //result ^= reserved1_reg.reg;
    //result ^= mode1_reg.reg;
    //result ^= reserved2_reg.reg;
    //result ^= mode2_reg.reg;

    result = tlv493d_calculate_parity(result);

    ESP_LOGW(TAG, "parity 0x%02x (%s)", result, uint8_to_binary(result));

    mode1_reg.bits.parity = result;

    ESP_GOTO_ON_ERROR(tlv493d_set_mode1_register((tlv493d_handle_t)dev, mode1_reg), err_handle, TAG, "write mode 1 register for init failed");

    ESP_LOGW(TAG, "mode 1 0x%02x (%s)", mode1_reg.reg, uint8_to_binary(mode1_reg.reg));
    ESP_LOGW(TAG, "mode 2 0x%02x (%s)", mode2_reg.reg, uint8_to_binary(mode2_reg.reg));

    /* attempt to write factory setting registers */
    //ESP_GOTO_ON_ERROR(i2c_tlv493d_set_factorysetting_registers(out_handle), err_handle, TAG, "write factory setting registers for init failed");

    /* attempt to configure modes */
    /*
    i2c_tlv493d_mode1_register_t mode1_reg;
    mode1_reg.bits.fast_mode_enabled = out_handle->fast_mode_enabled;
    mode1_reg.bits.irq_pin_enabled = out_handle->irq_pin_enabled;
    mode1_reg.bits.low_power_mode_enabled = out_handle->low_power_mode_enabled;
    mode1_reg.bits.i2c_slave_address = I2C_TLV493D_I2C_ADDRESS_00;
    ESP_GOTO_ON_ERROR(i2c_tlv493d_set_mode1_register(out_handle, mode1_reg), err_handle, TAG, "write mode 1 register for init failed");

    i2c_tlv493d_mode3_register_t mode3_reg;
    mode3_reg.bits.low_power_period = out_handle->low_power_period;
    mode3_reg.bits.parity_test_enabled = out_handle->parity_test_enabled;
    mode3_reg.bits.temperature_disabled = out_handle->temperature_disabled;
    ESP_GOTO_ON_ERROR(i2c_tlv493d_set_mode3_register(out_handle, mode3_reg), err_handle, TAG, "write mode 3 register for init failed");
    */


    /* attempt to reset ?? */


    /* app-start task delay  */
    //vTaskDelay(pdMS_TO_TICKS(I2C_TLV493D_APPSTART_DELAY_MS));
    vTaskDelay(pdMS_TO_TICKS(150));

    /* set device handle */
    *tlv493d_handle = (tlv493d_handle_t)dev;

    return ESP_OK;

    err_handle:
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}


esp_err_t tlv493d_get_data(tlv493d_handle_t handle, tlv493d_data_t *const data) {
    tlv493d_data_signal_t data_signal;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( tlv493d_get_fixed_magnetic_axes(handle, &data_signal), TAG, "unable to read raw data registers, get data failed" );

    return ESP_OK;
}


esp_err_t tlv493d_get_data_status(tlv493d_handle_t handle, bool *const ready) {
    tlv493d_bz_lsb_register_t bz_reg;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( tlv493d_get_bz_lsb_register(handle, &bz_reg), TAG, "unable to read bz lsb register, get data status failed" );

    *ready = bz_reg.bits.power_down_flag;

    return ESP_OK;
}

esp_err_t tlv493d_remove(tlv493d_handle_t handle) {
    tlv493d_device_t* device = (tlv493d_device_t*)handle;

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

esp_err_t tlv493d_delete(tlv493d_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    esp_err_t ret =  tlv493d_remove(handle);

    /* free handles */
    free(handle);

    return ret;
}