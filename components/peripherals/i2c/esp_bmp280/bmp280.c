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
 * @file bmp280.c
 *
 * ESP-IDF driver for BMP280 temperature and pressure sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/bmp280.h"
#include <string.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * possible BMP280 registers
 */

#define BMP280_REG_TEMP_XLSB            UINT8_C(0xFC)
#define BMP280_REG_TEMP_LSB             UINT8_C(0xFB)
#define BMP280_REG_TEMP_MSB             UINT8_C(0xFA)
#define BMP280_REG_TEMP                 (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB           UINT8_C(0xF9) 
#define BMP280_REG_PRESS_LSB            UINT8_C(0xF8)
#define BMP280_REG_PRESS_MSB            UINT8_C(0xF7)
#define BMP280_REG_PRESSURE             (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG               UINT8_C(0xF5) 
#define BMP280_REG_CTRL                 UINT8_C(0xF4)
#define BMP280_REG_STATUS               UINT8_C(0xF3)
#define BMP280_REG_CTRL_HUM             UINT8_C(0xF2)
#define BMP280_REG_RESET                UINT8_C(0xE0)
#define BMP280_REG_ID                   UINT8_C(0xD0)
#define BMP280_REG_CALIB                UINT8_C(0x88)
#define BMP280_REG_HUM_CALIB            UINT8_C(0x88)
#define BMP280_RESET_VALUE              UINT8_C(0xB6)

#define BMP280_TYPE_BMP280              UINT8_C(0x58)  //!< BMP280
#define BMP280_TYPE_BME280              UINT8_C(0x60)  //!< BME280

#define BMP280_DATA_POLL_TIMEOUT_MS     UINT16_C(250) // ? see datasheet tables 13 and 14, standby-time could be 2-seconds (2000ms)
#define BMP280_DATA_READY_DELAY_MS      UINT16_C(1)
#define BMP280_POWERUP_DELAY_MS         UINT16_C(25)  // start-up time is 2-ms
#define BMP280_APPSTART_DELAY_MS        UINT16_C(25)
#define BMP280_RESET_DELAY_MS           UINT16_C(25)
#define BMP280_CMD_DELAY_MS             UINT16_C(5)
#define BMP280_TX_RX_DELAY_MS           UINT16_C(10)


/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/**
 * @brief BMP280 status register (0xf3) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp280_status_register_u {
    struct {
        bool    image_update:1; /*!< bmp280 automatically set to 1 when NVM data are being copied to image registers and back to 0 when done (bit:0)  */
        uint8_t reserved1:2;    /*!< reserved (bit:1-2) */
        bool    measuring:1;    /*!< bmp280 automatically set to 1 whenever a conversion is running and back to 0 when results transferred to data registers  (bit:3) */
        uint8_t reserved2:4;    /*!< reserved (bit:4-7) */
    } bits;
    uint8_t reg;
} bmp280_status_register_t;

/**
 * @brief BMP280 control measurement register (0xf4) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp280_control_measurement_register_u {
    struct {
        bmp280_power_modes_t                power_mode:2;               /*!< bmp280 power mode of the device            (bit:0-1)  */
        bmp280_pressure_oversampling_t      pressure_oversampling:3;    /*!< bmp280 oversampling of pressure data       (bit:2-4) */
        bmp280_temperature_oversampling_t   temperature_oversampling:3; /*!< bmp280 oversampling of temperature data    (bit:5-7) */
    } bits;
    uint8_t reg;
} bmp280_control_measurement_register_t;

/**
 * @brief BMP280 configuration register (0xf5) structure definition.  The reset state is 0x00 for this register.
 */
typedef union __attribute__((packed)) bmp280_configuration_register_u {
    struct {
        bool                    spi_enabled:1;  /*!< bmp280 3-wire SPI interface enabled when true  (bit:0)  */
        uint8_t                 reserved:1;     /*!< bmp280 reserved                                (bit:1) */
        bmp280_iir_filters_t    iir_filter:3;   /*!< bmp280 time constant of the IIR filter         (bit:2-4) */
        bmp280_standby_times_t  standby_time:3; /*!< bmp280 inactive duration in normal mode        (bit:5-7) */
    } bits;
    uint8_t reg;
} bmp280_configuration_register_t;

/**
 * @brief BMP280 temperature and pressure calibration factors structure definition.
 */
typedef struct bmp280_cal_factors_s {
    /* temperature and pressure compensation */
    uint16_t                dig_T1;
    int16_t                 dig_T2;
    int16_t                 dig_T3;
    uint16_t                dig_P1;
    int16_t                 dig_P2;
    int16_t                 dig_P3;
    int16_t                 dig_P4;
    int16_t                 dig_P5;
    int16_t                 dig_P6;
    int16_t                 dig_P7;
    int16_t                 dig_P8;
    int16_t                 dig_P9;
    int32_t                 t_fine;
} bmp280_cal_factors_t;

/**
 * @brief BMP280 device descriptor structure definition.
 */
typedef struct bmp280_device_s {
    bmp280_config_t             config;             /*!< bmp280 device configuration */  
    hal_master_dev_handle_t     hal_handle;         /*!< bmp280 HAL communication bus handle */
    bmp280_cal_factors_t       *cal_factors;        /*!< bmp280 device calibration factors */
    uint8_t                     sensor_type;        /*!< sensor type, should be bmp280 */
} bmp280_device_t;

/*
* static constant declarations
*/

static const char *TAG = "bmp280";




/**
 * @brief Temperature compensation algorithm taken from datasheet.  See datasheet for details.
 *
 * @param[in] device BMP280 device descriptor.
 * @param[in] adc_temperature Raw adc temperature.
 * @return Compensated temperature in degrees Celsius.
 */
static inline float convert_adc_signal_to_temperature(bmp280_device_t *const device, const int32_t adc_temperature) {
    int32_t var1, var2;

    var1 = ((((adc_temperature >> 3) - ((int32_t)device->cal_factors->dig_T1 << 1))) * (int32_t)device->cal_factors->dig_T2) >> 11;
    var2 = (((((adc_temperature >> 4) - (int32_t)device->cal_factors->dig_T1) * ((adc_temperature >> 4) - (int32_t)device->cal_factors->dig_T1)) >> 12) * (int32_t)device->cal_factors->dig_T3) >> 14;
 
    device->cal_factors->t_fine = var1 + var2;

    var1 = (device->cal_factors->t_fine * 5 + 128) >> 8;

    return (float)var1 / 100.0f;
}

/**
 * @brief Pressure compensation algorithm taken from datasheet.  See datasheet for details.
 *
 * @param[in] device BMP280 device descriptor.
 * @param[in] adc_pressure Raw adc pressure.
 * @return Compensated pressure in pascal.
 */
static inline float convert_adc_signal_to_pressure(bmp280_device_t *const device, const int32_t adc_pressure) {
    int64_t var1, var2, p;

    var1 = (int64_t)device->cal_factors->t_fine - 128000;
    var2 = var1 * var1 * (int64_t)device->cal_factors->dig_P6;
    var2 = var2 + ((var1 * (int64_t)device->cal_factors->dig_P5) << 17);
    var2 = var2 + (((int64_t)device->cal_factors->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)device->cal_factors->dig_P3) >> 8) + ((var1 * (int64_t)device->cal_factors->dig_P2) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ((int64_t)device->cal_factors->dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }

    p = 1048576 - adc_pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)device->cal_factors->dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)device->cal_factors->dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)device->cal_factors->dig_P7 << 4);

    return (float)p / 256.0f;;
}

/**
 * @brief HAL read calibration factor registers from BMP280.  see datasheet for details.
 *
 * @param[in] device BMP280 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_cal_factor_registers(bmp280_device_t *const device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* bmp280 attempt to request T1-T3 calibration values from device */
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x88, &device->cal_factors->dig_T1) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x8a, (uint16_t *)&device->cal_factors->dig_T2) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x8c, (uint16_t *)&device->cal_factors->dig_T3) );
    /* bmp280 attempt to request P1-P9 calibration values from device */
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x8e, &device->cal_factors->dig_P1) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x90, (uint16_t *)&device->cal_factors->dig_P2) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x92, (uint16_t *)&device->cal_factors->dig_P3) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x94, (uint16_t *)&device->cal_factors->dig_P4) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x96, (uint16_t *)&device->cal_factors->dig_P5) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x98, (uint16_t *)&device->cal_factors->dig_P6) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x9a, (uint16_t *)&device->cal_factors->dig_P7) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x9c, (uint16_t *)&device->cal_factors->dig_P8) );
    ESP_ERROR_CHECK( hal_master_read_word_from(device->hal_handle, 0x9e, (uint16_t *)&device->cal_factors->dig_P9) );

    /*
    ESP_LOGD(TAG, "Calibration data received:");
    ESP_LOGD(TAG, "dig_T1=%u", handle->dev_cal_factors->dig_T1);
    ESP_LOGD(TAG, "dig_T2=%d", handle->dev_cal_factors->dig_T2);
    ESP_LOGD(TAG, "dig_T3=%d", handle->dev_cal_factors->dig_T3);
    ESP_LOGD(TAG, "dig_P1=%u", handle->dev_cal_factors->dig_P1);
    ESP_LOGD(TAG, "dig_P2=%d", handle->dev_cal_factors->dig_P2);
    ESP_LOGD(TAG, "dig_P3=%d", handle->dev_cal_factors->dig_P3);
    ESP_LOGD(TAG, "dig_P4=%d", handle->dev_cal_factors->dig_P4);
    ESP_LOGD(TAG, "dig_P5=%d", handle->dev_cal_factors->dig_P5);
    ESP_LOGD(TAG, "dig_P6=%d", handle->dev_cal_factors->dig_P6);
    ESP_LOGD(TAG, "dig_P7=%d", handle->dev_cal_factors->dig_P7);
    ESP_LOGD(TAG, "dig_P8=%d", handle->dev_cal_factors->dig_P8);
    ESP_LOGD(TAG, "dig_P9=%d", handle->dev_cal_factors->dig_P9);
    */

    return ESP_OK;
}

/**
 * @brief HAL read chip identification register from BMP280.
 * 
 * @param[in] device BMP280 device descriptor.
 * @param[out] reg BMP280 chip identification register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_chip_id_register(bmp280_device_t *const device, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read_byte_from(device->hal_handle, BMP280_REG_ID, reg), TAG, "read chip identifier register failed" );

    return ESP_OK;
}

/**
 * @brief HAL read status register from BMP280.
 * 
 * @param device BMP280 device descriptor.
 * @param[out] reg BMP280 status register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_status_register(bmp280_device_t *const device, bmp280_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read_byte_from(device->hal_handle, BMP280_REG_STATUS, &reg->reg), TAG, "read status register failed" );

    return ESP_OK;
}

/**
 * @brief HAL read control measurement register from BMP280.
 * 
 * @param[in] device BMP280 device descriptor.
 * @param[out] reg BMP280 control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_control_measurement_register(bmp280_device_t *const device, bmp280_control_measurement_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read_byte_from(device->hal_handle, BMP280_REG_CTRL, &reg->reg), TAG, "read control measurement register failed" );

    return ESP_OK;
}

/**
 * @brief HAL write control measurement register to BMP280. 
 * 
 * @param[in] device BMP280 device descriptor.
 * @param[in] reg BMP280 control measurement register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_set_control_measurement_register(bmp280_device_t *const device, const bmp280_control_measurement_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write_byte_to(device->hal_handle, BMP280_REG_CTRL, reg.reg), TAG, "write control measurement register failed" );

    return ESP_OK;
}

/**
 * @brief HAL read configuration register from BMP280.
 * 
 * @param[in] device BMP280 device descriptor.
 * @param[out] reg BMP280 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_config_register(bmp280_device_t *const device, bmp280_configuration_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( device && reg );

    /* attempt read transaction */
    ESP_RETURN_ON_ERROR( hal_master_read_byte_from(device->hal_handle, BMP280_REG_CONFIG, &reg->reg), TAG, "read configuration register failed" );

    return ESP_OK;
}

/**
 * @brief HAL write configuration register to BMP280. 
 * 
 * @param[in] device BMP280 device descriptor.
 * @param[in] reg BMP280 configuration register.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_set_config_register(bmp280_device_t *const device, const bmp280_configuration_register_t reg) {
    bmp280_configuration_register_t config = { .reg = reg.reg};

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* set reserved to 0 */
    config.bits.reserved = 0;

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write_byte_to(device->hal_handle, BMP280_REG_CONFIG, config.reg), TAG, "write configuration register failed" );

    return ESP_OK;
}

/**
 * @brief HAL write reset register to reset BMP280 with restart delay.
 * 
 * @param device BMP280 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_set_reset_register(bmp280_device_t *device) {
    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt write transaction */
    ESP_RETURN_ON_ERROR( hal_master_write_byte_to(device->hal_handle, BMP280_REG_RESET, BMP280_RESET_VALUE), TAG, "write reset register failed" );

    /* wait until finished copying NVP data */
    // forced delay before next transaction - see datasheet for details
    vTaskDelay(pdMS_TO_TICKS(BMP280_RESET_DELAY_MS)); // check is busy in timeout loop...

    return ESP_OK;
}

/**
 * @brief HAL to get raw adc temperature and pressure signals from BMP280.
 * 
 * @param device BMP280 device descriptor.
 * @param[out] temperature Raw adc temperature signal.
 * @param[out] pressure Raw adc pressure signal.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_get_adc_signals(bmp280_device_t *device, int32_t *const temperature, int32_t *const pressure) {
    esp_err_t        ret           = ESP_OK;
    const uint64_t   start_time    = esp_timer_get_time();
    bool             data_is_ready = false;
    bit48_uint8_buffer_t rx;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && pressure );

    /* attempt to poll until data is available or timeout */
    do {
        bmp280_status_register_t status_reg = { 0 };

        /* attempt to read device status register */
        ESP_GOTO_ON_ERROR( hal_get_status_register(device, &status_reg), err, TAG, "read status register for get fixed measurement failed" );

        /* set data is ready flag */
        data_is_ready = !status_reg.bits.measuring;

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(BMP280_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, BMP280_DATA_POLL_TIMEOUT_MS * 1000))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    // need to read in one sequence to ensure they match.
    ESP_GOTO_ON_ERROR( hal_master_read_from(device->hal_handle, BMP280_REG_PRESSURE, rx, BIT48_UINT8_BUFFER_SIZE), err, TAG, "read temperature and pressure data failed" );

    /* concat adc pressure & temperature bytes */
    const int32_t adc_press = rx[0] << 12 | rx[1] << 4 | rx[2] >> 4;
    const int32_t adc_temp  = rx[3] << 12 | rx[4] << 4 | rx[5] >> 4;

    /* set output parameters */
    *temperature = adc_temp;
    *pressure    = adc_press;

    return ESP_OK;

    err:
        return ret;
}

/**
 * @brief HAL to setup and configuration of BMP280 registers.
 * 
 * @param device BMP280 device descriptor.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t hal_setup_registers(bmp280_device_t *const device) {
    bmp280_configuration_register_t       config_reg = { 0 };
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read calibration factors from device */
    ESP_RETURN_ON_ERROR( hal_get_cal_factor_registers(device), TAG, "read calibration factors for get registers failed" );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "read configuration register for setup failed");

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for setup failed");

    /* initialize configuration register from configuration params */
    config_reg.bits.standby_time = device->config.standby_time;
    config_reg.bits.iir_filter   = device->config.iir_filter;

    /* initialize control measurement register from configuration params */
    if (device->config.power_mode == BMP280_POWER_MODE_FORCED) {
        // initial mode for forced is sleep
        ctrl_meas_reg.bits.power_mode               = BMP280_POWER_MODE_SLEEP;
        ctrl_meas_reg.bits.temperature_oversampling = device->config.temperature_oversampling;
        ctrl_meas_reg.bits.pressure_oversampling    = device->config.pressure_oversampling;
    } else {
        ctrl_meas_reg.bits.power_mode               = device->config.power_mode;
        ctrl_meas_reg.bits.temperature_oversampling = device->config.temperature_oversampling;
        ctrl_meas_reg.bits.pressure_oversampling    = device->config.pressure_oversampling;
    }

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "write configuration register for setup failed");

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( hal_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for setup failed");

    return ESP_OK;
}

esp_err_t bmp280_init(const void* master_handle, const bmp280_config_t *bmp280_config, bmp280_handle_t *bmp280_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && bmp280_config );

    /* validate memory availability for handle */
    esp_err_t ret = ESP_OK;
    bmp280_device_t* device = (bmp280_device_t*)calloc(1, sizeof(bmp280_device_t));
    ESP_GOTO_ON_FALSE(device, ESP_ERR_NO_MEM, err, TAG, "no memory for bmp280 device for init");

    /* validate memory availability for handle calibration factors */
    device->cal_factors = (bmp280_cal_factors_t*)calloc(1, sizeof(bmp280_cal_factors_t));
    ESP_GOTO_ON_FALSE(device->cal_factors, ESP_ERR_NO_MEM, err_handle, TAG, "no memory for bmp280 device calibration factors for init");

    /* copy configuration */
    device->config = *bmp280_config;

    /* validate hal master bus interface */
    ESP_GOTO_ON_FALSE( bmp280_config->hal_bif == HAL_MASTER_BIF_I2C, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid HAL master bus interface, device handle initialization failed");

    /* cast i2c_master_bus_handle_t to void pointer */
    i2c_master_bus_handle_t i2c_master_handle = (i2c_master_bus_handle_t)master_handle;

    /* validate i2c master bus handle */
    ESP_GOTO_ON_FALSE( i2c_master_handle, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid master bus handle, device handle initialization failed");

    /* cast hal_config to i2c_device_config_t pointer */
    const i2c_device_config_t* i2c_dev_conf_ptr = (const i2c_device_config_t*)device->config.hal_config;

    /* validate i2c device configuration */
    ESP_GOTO_ON_FALSE(i2c_dev_conf_ptr, ESP_ERR_INVALID_ARG, err_handle, TAG, "invalid i2c device configuration, device handle initialization failed");
    
    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = i2c_dev_conf_ptr->device_address,
        .scl_speed_hz    = i2c_dev_conf_ptr->scl_speed_hz
    };

    /* delay task before transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP280_POWERUP_DELAY_MS));

    /* create hal master device */
    ret = hal_master_new_i2c_device(i2c_master_handle, &i2c_dev_conf, &device->hal_handle);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "hal_master_new_i2c_device failed, device handle initialization failed");

    /* validate device exists on the hal master communication bus */
    ret = hal_master_probe(device->hal_handle, i2c_dev_conf_ptr->device_address);
    ESP_GOTO_ON_ERROR(ret, err_handle, TAG, "device does not exist on the hal master communication bus, device handle initialization failed");

    /* delay before next transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP280_CMD_DELAY_MS));

    /* read and validate device type */
    ESP_GOTO_ON_ERROR( hal_get_chip_id_register(device, &device->sensor_type), err_handle, TAG, "read chip identifier for init failed");
    if(device->sensor_type != BMP280_TYPE_BMP280) {
        ESP_GOTO_ON_FALSE(false, ESP_ERR_INVALID_VERSION, err_handle, TAG, "detected an invalid chip type for init, got: %02x", device->sensor_type);
    }

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( hal_set_reset_register(device), TAG, "write reset register for init failed" );

    /* attempt to setup device */
    ESP_RETURN_ON_ERROR( hal_setup_registers(device), TAG, "unable to setup device, init failed" );

    /* set output parameter */
    *bmp280_handle = (bmp280_handle_t)device;

    /* delay task before transaction */
    vTaskDelay(pdMS_TO_TICKS(BMP280_APPSTART_DELAY_MS));

    return ESP_OK;

    /* something went wrong - after device descriptor instantiation */
    err_handle:
        /* remove device from hal master communication bus */
        if (device->hal_handle) {
            hal_master_delete(device->hal_handle);
        }

        /* free device handle */
        free(device);
    err:
        return ret;
}

esp_err_t bmp280_get_measurements(bmp280_handle_t handle, float *const temperature, float *const pressure) {
    int32_t adc_press, adc_temp;
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device && temperature && pressure );

    // need to read in one sequence to ensure they match.
    ESP_RETURN_ON_ERROR( hal_get_adc_signals(device, &adc_temp, &adc_press), TAG, "read temperature and pressure adc signals failed" );

    /* compensate adc temperature & pressure and set output parameters */
    *temperature = convert_adc_signal_to_temperature(device, adc_temp);
    *pressure    = convert_adc_signal_to_pressure(device, adc_press);

    return ESP_OK;
}

esp_err_t bmp280_get_data_status(bmp280_handle_t handle, bool *const ready) {
    bmp280_status_register_t status_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read device status register */
    ESP_RETURN_ON_ERROR( hal_get_status_register(device, &status_reg), TAG, "read status register (data ready state) failed" );

    /* set ready state */
    *ready = !status_reg.bits.measuring;

    return ESP_OK;
}

esp_err_t bmp280_get_power_mode(bmp280_handle_t handle, bmp280_power_modes_t *const power_mode) {
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get power mode failed" );

    /* set power mode */
    *power_mode = ctrl_meas_reg.bits.power_mode;

    return ESP_OK;
}

esp_err_t bmp280_set_power_mode(bmp280_handle_t handle, const bmp280_power_modes_t power_mode) {
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for set power mode failed" );

    /* initialize control measurement register */
    ctrl_meas_reg.bits.power_mode = power_mode;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( hal_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for set power mode failed" );

    return ESP_OK;
}

esp_err_t bmp280_get_pressure_oversampling(bmp280_handle_t handle, bmp280_pressure_oversampling_t *const oversampling) {
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get pressure oversampling failed" );

    /* set oversampling */
    *oversampling = ctrl_meas_reg.bits.pressure_oversampling;

    return ESP_OK;
}

esp_err_t bmp280_set_pressure_oversampling(bmp280_handle_t handle, const bmp280_pressure_oversampling_t oversampling) {
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for set pressure oversampling failed" );

    /* initialize control measurement register */
    ctrl_meas_reg.bits.pressure_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( hal_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for set pressure oversampling failed" );

    return ESP_OK;
}

esp_err_t bmp280_get_temperature_oversampling(bmp280_handle_t handle, bmp280_temperature_oversampling_t *const oversampling) {
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for get temperature oversampling failed" );

    /* set oversampling */
    *oversampling = ctrl_meas_reg.bits.temperature_oversampling;

    return ESP_OK;
}

esp_err_t bmp280_set_temperature_oversampling(bmp280_handle_t handle, const bmp280_temperature_oversampling_t oversampling) {
    bmp280_control_measurement_register_t ctrl_meas_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read control measurement register */
    ESP_RETURN_ON_ERROR( hal_get_control_measurement_register(device, &ctrl_meas_reg), TAG, "read control measurement register for set temperature oversampling failed" );

    /* initialize control measurement register */
    ctrl_meas_reg.bits.temperature_oversampling = oversampling;

    /* attempt to write control measurement register */
    ESP_RETURN_ON_ERROR( hal_set_control_measurement_register(device, ctrl_meas_reg), TAG, "write control measurement register for set temperature oversampling failed" );

    return ESP_OK;
}

esp_err_t bmp280_get_standby_time(bmp280_handle_t handle, bmp280_standby_times_t *const standby_time) {
    bmp280_configuration_register_t config_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "read configuration register for get stand-by time failed" );

    /* set standby time */
    *standby_time = config_reg.bits.standby_time;

    return ESP_OK;
}

esp_err_t bmp280_set_standby_time(bmp280_handle_t handle, const bmp280_standby_times_t standby_time) {
    bmp280_configuration_register_t config_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "read configuration register for set stand-by time failed" );

    /* initialize configuration register */
    config_reg.bits.standby_time = standby_time;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "write configuration register for set stand-by time failed" );

    return ESP_OK;
}

esp_err_t bmp280_get_iir_filter(bmp280_handle_t handle, bmp280_iir_filters_t *const iir_filter) {
    bmp280_configuration_register_t config_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "read configuration register for get IIR filter failed" );

    /* set standby time */
    *iir_filter = config_reg.bits.iir_filter;

    return ESP_OK;
}

esp_err_t bmp280_set_iir_filter(bmp280_handle_t handle, const bmp280_iir_filters_t iir_filter) {
    bmp280_configuration_register_t config_reg = { 0 };
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to read configuration register */
    ESP_RETURN_ON_ERROR( hal_get_config_register(device, &config_reg), TAG, "read configuration register for set IIR filter failed" );

    /* initialize configuration register */
    config_reg.bits.iir_filter = iir_filter;

    /* attempt to write configuration register */
    ESP_RETURN_ON_ERROR( hal_set_config_register(device, config_reg), TAG, "write configuration register for set IIR filter failed" );

    return ESP_OK;
}

esp_err_t bmp280_reset(bmp280_handle_t handle) {
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt to reset device */
    ESP_RETURN_ON_ERROR( hal_set_reset_register(device), TAG, "write reset register for reset failed" );

    /* attempt to setup device */
    ESP_RETURN_ON_ERROR( hal_setup_registers(device), TAG, "unable to setup device, reset failed" );

    return ESP_OK;
}

esp_err_t bmp280_remove(bmp280_handle_t handle) {
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* remove device from hal bus */
    return hal_master_remove(device->hal_handle);
}

esp_err_t bmp280_delete(bmp280_handle_t handle){
    bmp280_device_t* device = (bmp280_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* delete device handle */
    if (device->hal_handle) {
        hal_master_delete(device->hal_handle);
    }

    free(device);
    return ESP_OK;
}

const char* bmp280_get_fw_version(void) {
    return (const char*)BMP280_FW_VERSION_STR;
}

int32_t bmp280_get_fw_version_number(void) {
    return (int32_t)BMP280_FW_VERSION_INT32;
}