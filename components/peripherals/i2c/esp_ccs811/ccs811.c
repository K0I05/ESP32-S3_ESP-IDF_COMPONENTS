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
 * @file ccs811.c
 *
 * ESP-IDF driver for CCS811 Air Quality sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/ccs811.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/*
 * CCS811 definitions
*/

#define CCS811_HW_ID                    UINT8_C(0x81)   //!< ccs811 I2C hardware identification (0x81)

#define CCS811_REG_STATUS_R             UINT8_C(0x00)   //!< ccs811 I2C status register (1-byte)
#define CCS811_REG_MEAS_MODE_RW         UINT8_C(0x01)   //!< ccs811 I2C measurement mode and conditions register (1-byte)
#define CCS811_REG_ALG_RESULT_DATA_R    UINT8_C(0x02)   //!< ccs811 I2C algorithm results (up to 8-bytes)
#define CCS811_REG_RAW_DATA_R           UINT8_C(0x03)   //!< ccs811 I2C raw ADC data values (2-bytes)
#define CCS811_REG_ENV_DATA_W           UINT8_C(0x05)   //!< ccs811 I2C temperature and humidity compensation (4-bytes)
#define CCS811_REG_NTC_R                UINT8_C(0x06)   //!< ccs811 I2C temperature and humidity compensation (4-bytes)
#define CCS811_REG_THRESHOLDS_W         UINT8_C(0x10)   //!< ccs811 I2C interrupt threshold when in operation (4-bytes)
#define CCS811_REG_BASELINE_RW          UINT8_C(0x11)   //!< ccs811 I2C encoded current baseline (2-bytes)
#define CCS811_REG_HW_ID_R              UINT8_C(0x20)   //!< ccs811 I2C hardware identification register (1-byte), value is 0x81
#define CCS811_REG_HW_VERSION_R         UINT8_C(0x21)   //!< ccs811 I2C hardware version register (1-byte), value is 0x1x
#define CCS811_REG_FW_BOOT_VERSION_R    UINT8_C(0x23)   //!< ccs811 I2C firmware boot version register (2-bytes)
#define CCS811_REG_FW_APP_VERSION_R     UINT8_C(0x24)   //!< ccs811 I2C firmware application version register (2-bytes)
#define CCS811_REG_INTERNAL_STATE_R     UINT8_C(0xa0)   //!< ccs811 I2C internal status register (1-byte)
#define CCS811_REG_ERROR_ID_R           UINT8_C(0xe0)   //!< ccs811 I2C error source register from internal status register (1-byte)
#define CCS811_REG_APP_START_W          UINT8_C(0xf4)   //!< ccs811 I2C application start (1-byte)
#define CCS811_REG_SW_RESET_W           UINT8_C(0xff)   //!< ccs811 I2C software reset when correct 4-bytes written (0x11 0xe5 0x72 0x8a)

#define CCS811_ECO2_RANGE_MIN           (400)           //!< ccs811 eCO2 minimum in ppm
#define CCS811_ECO2_RANGE_MAX           (32768)         //!< ccs811 eCO2 maximum in ppm
#define CCS811_ETVOC_RANGE_MIN          (0)             //!< ccs811 eTVOC minimum in ppb
#define CCS811_ETVOC_RANGE_MAX          (29206)         //!< ccs811 eTVOC maximum in ppb
#define CCS811_TEMPERATURE_RANGE_MIN    (-25)           //!< ccs811 temperature minimum in degrees Celsius
#define CCS811_TEMPERATURE_RANGE_MAX    (50)            //!< ccs811 temperature maximum in degrees Celsius
#define CCS811_HUMIDITY_RANGE_MIN       (0)             //!< ccs811 relative humidity minimum in percent
#define CCS811_HUMIDITY_RANGE_MAX       (100)           //!< ccs811 relative humidity maximum in percent

#define CCS811_POWERUP_DELAY_MS         UINT16_C(25)    //!< ccs811 I2C start-up delay before device accepts transactions
#define CCS811_APPSTART_DELAY_MS        UINT16_C(25)            
#define CCS811_RESET_DELAY_MS           UINT16_C(250)   //!< ccs811 I2C software reset delay before device accepts transactions
#define CCS811_WAKE_DELAY_MS            UINT16_C(5)     //!< ccs811 I2C wake-up delay from sleep before device accepts transactions
#define CCS811_DATA_READY_DELAY_MS      UINT16_C(10)
#define CCS811_DATA_POLL_TIMEOUT_MS     UINT16_C(100)
#define CCS811_ERASE_DELAY_MS           UINT16_C(500)   //!< ccs811 I2C erase delay before device accepts transactions
#define CCS811_VERIFY_DELAY_MS          UINT16_C(70)    //!< ccs811 I2C verification delay before device accepts transactions
#define CCS811_TX_RX_DELAY_MS           UINT16_C(10)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/*
 * macro definitions
*/
#define CCS811_SW_RESET_DATA    { 0x11, 0xe5, 0x72, 0x8a }
#define CCS811_ERASE_DATA       { 0xe7, 0xa7, 0xe6, 0x09 }

#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


/*
* static constant declarations
*/
static const char *TAG = "ccs811";

/**
 * @brief CCS811 I2C unknown error message.
 */
static const char *ccs811_unknown_msg = "Unknown error.";

/**
 * @brief CCS811 I2C unknown error code.
 */
static const char *ccs811_unknown_code = "UNKNOWN.";


/**
 * @brief CCS811 I2C error definition table structure.
 */
static const struct ccs811_error_row_s ccs811_error_definition_table[CCS811_ERROR_TABLE_SIZE] = {
  {"WRITE_REG_INVALID",  "The CCS811 received an I²C write request addressed to this station but with invalid register address ID"},
  {"READ_REG_INVALID",   "The CCS811 received an I²C read request to a mailbox ID that is invalid"},
  {"DRIVERMODE_INVALID", "The CCS811 received an I²C request to write an unsupported mode to driver mode"},
  {"MAX_RESISTANCE",     "The sensor resistance measurement has reached or exceeded the maximum range"},
  {"HEATER_FAULT",       "The Heater current in the CCS811 is not in range"},
  {"HEATER_SUPPLY",      "The Heater voltage is not being applied correctly"}
};

/**
 * @brief CCS811 I2C unknown measure mode.
 */
static const char* ccs811_unknown_measure_mode = "UNKNOWN.";

/**
 * @brief CCS811 I2C measure mode definition table structure.
 */
static const struct ccs811_measure_mode_row_s ccs811_measure_mode_definition_table[CCS811_MEASURE_MODE_TABLE_SIZE] = {
    {CCS811_DRIVE_MODE_IDLE,                    "Idle - measurements are disabled"},
    {CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ,      "Constant Power IAQ - iaq measurement every second"},
    {CCS811_DRIVE_MODE_PULSE_HEATING_IAQ,       "Pulse Heating IAQ - iaq measurement every 10-seconds"},
    {CCS811_DRIVE_MODE_LP_PULSE_HEATING_IAQ,    "Low-Power Pulse Heating IAQ - iaq measurement every 60-seconds"},
    {CCS811_DRIVE_MODE_CONSTANT_POWER,          "Constant Power - measurement every 250ms"}
};



/*
* functions and subroutines
*/

/**
 * @brief CCS811 I2C read from register address transaction.  This is a write and then read process.
 * 
 * @param handle CCS811 device handle.
 * @param reg_addr CCS811 register address to read from.
 * @param buffer CCS811 read transaction return buffer.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_read_from(ccs811_handle_t handle, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_read_from failed" );

    return ESP_OK;
}

/**
 * @brief CCS811 I2C read halfword from register address transaction.
 * 
 * @param handle CCS811 device handle.
 * @param reg_addr CCS811 register address to read from.
 * @param word CCS811 read transaction return word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_read_word_from(ccs811_handle_t handle, const uint8_t reg_addr, uint16_t *const word) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit16_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_read_word_from failed" );

    /* set output parameter */
    *word = (uint16_t)rx[0] | ((uint16_t)rx[1] << 8);

    return ESP_OK;
}

/**
 * @brief CCS811 I2C read byte from register address transaction.
 * 
 * @param handle CCS811 device handle.
 * @param reg_addr CCS811 register address to read from.
 * @param byte CCS811 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_read_byte_from(ccs811_handle_t handle, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief CCS811 I2C write command to register address transaction.
 * 
 * @param handle CCS811 device handle.
 * @param reg_addr CCS811 command register address to write to.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_write_command(ccs811_handle_t handle, const uint8_t reg_addr) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_write_command, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief CCS811 I2C write transaction.
 * 
 * @param handle CCS811 device handle.
 * @param buffer Buffer to write for write transaction.
 * @param size Length of buffer to write for write transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_write(ccs811_handle_t handle, const uint8_t *buffer, const uint8_t size) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_write, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief CCS811 I2C write halfword to register address transaction.
 * 
 * @param handle CCS811 device handle.
 * @param reg_addr CCS811 register address to write to.
 * @param word CCS811 write transaction input word.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_write_word_to(ccs811_handle_t handle, const uint8_t reg_addr, const uint16_t word) {
    const bit24_uint8_buffer_t tx = { reg_addr, (uint8_t)(word & 0xff), (uint8_t)((word >> 8) & 0xff) }; // register, lsb, msb

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT24_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_write_word_to, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief CCS811 I2C write byte to register address transaction.
 * 
 * @param handle CCS811 device handle.
 * @param reg_addr CCS811 register address to write to.
 * @param byte CCS811 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t ccs811_i2c_write_byte_to(ccs811_handle_t handle, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(handle->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "ccs811_i2c_write_byte_to, i2c write failed" );
                        
    return ESP_OK;
}


/**
 * @brief Gets CCS811 microsecond duration from device handle.  See datasheet for details.
 *
 * @param[in] ccs811_handle CCS811 device handle.
 * @return duration in microseconds.
 */
static inline uint64_t ccs811_get_duration_us(ccs811_handle_t handle) {
    ccs811_measure_mode_register_t mmode;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_get_measure_mode_register(handle, &mmode), TAG, "read measure mode register failed" );

    switch (mmode.bits.drive_mode) {
        case CCS811_DRIVE_MODE_IDLE:
            return 0;   // stand-by 
        case CCS811_DRIVE_MODE_CONSTANT_POWER_IAQ:
            return 1500000; // 1-second (1000-ms)
        case CCS811_DRIVE_MODE_PULSE_HEATING_IAQ:
            return 15000000; // 10-seconds (10000-ms)
        case CCS811_DRIVE_MODE_LP_PULSE_HEATING_IAQ:
            return 65000000; // 60-seconds (60000-ms)
        case CCS811_DRIVE_MODE_CONSTANT_POWER:
            return 300000; // 250-ms
        default:
            return 1500000;
    }
}

esp_err_t ccs811_get_status_register(ccs811_handle_t handle, ccs811_status_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_byte_from(handle, CCS811_REG_STATUS_R, &reg->reg), TAG, "read status register failed" );
    
    return ESP_OK;
}

esp_err_t ccs811_get_measure_mode_register(ccs811_handle_t handle, ccs811_measure_mode_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_byte_from(handle, CCS811_REG_MEAS_MODE_RW, &reg->reg), TAG, "read measure mode register failed" );
    
    return ESP_OK;
}

esp_err_t ccs811_set_measure_mode_register(ccs811_handle_t handle, const ccs811_measure_mode_register_t reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* set reserved bits */
    ccs811_measure_mode_register_t measure_mode = { .reg = reg.reg };
    measure_mode.bits.reserved1 = 0;
    measure_mode.bits.reserved2 = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_write_byte_to(handle, CCS811_REG_MEAS_MODE_RW, measure_mode.reg), TAG, "write measure mode register failed" );

    return ESP_OK;
}

esp_err_t ccs811_get_error_register(ccs811_handle_t handle, ccs811_error_code_register_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_byte_from(handle, CCS811_REG_ERROR_ID_R, &reg->reg), TAG, "read error identifier register failed" );
    
    return ESP_OK;
}

esp_err_t ccs811_set_environmental_data_register(ccs811_handle_t handle, const float temperature, const float humidity) {
    bit40_uint8_buffer_t tx = { CCS811_REG_ENV_DATA_W, 0, 0, 0, 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate temperature range (-25 to 50 C) */
    ESP_RETURN_ON_FALSE(!(temperature < CCS811_TEMPERATURE_RANGE_MIN), ESP_ERR_INVALID_ARG, TAG, "Temperature must within a range of -25 to 50 degrees Celsius");
    ESP_RETURN_ON_FALSE(!(temperature > CCS811_TEMPERATURE_RANGE_MAX), ESP_ERR_INVALID_ARG, TAG, "Temperature must within a range of -25 to 50 degrees Celsius");

	/* validate humidity range (0 to 100 %) */
    ESP_RETURN_ON_FALSE(!(humidity < CCS811_HUMIDITY_RANGE_MIN), ESP_ERR_INVALID_ARG, TAG, "Relative humidity must within a range of 0 to 100 percent");
    ESP_RETURN_ON_FALSE(!(humidity > CCS811_HUMIDITY_RANGE_MAX), ESP_ERR_INVALID_ARG, TAG, "Relative humidity must within a range of 0 to 100 percent");

    /* note: application note appears to be incorrect with value to register conversion */

    /* temperature with offset and humidity multipliers */
    uint32_t h_uint = humidity * 1000;              // 42.348 becomes 42348
	uint32_t t_uint = (temperature * 1000) + 25000;	// 23.2 becomes 23200 with 25 C offset

    /* set frame data */

    // correct rounding, see issue 8: https://github.com/sparkfun/Qwiic_BME280_CCS811_Combo/issues/8
	tx[1] = (h_uint + 250) / 500;
	tx[2] = 0; // CCS811 only supports increments of 0.5 so bits 7-0 will always be zero

    // correct rounding
	tx[3] = (t_uint + 250) / 500;
	tx[4] = 0;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_write(handle, tx, BIT40_UINT8_BUFFER_SIZE), TAG, "write environmental data failed" );

    /* set handle environmental data parameters */
    handle->dev_config.humidity = humidity;
    handle->dev_config.temperature = temperature;

    return ESP_OK;
}

esp_err_t ccs811_set_thresholds_register(ccs811_handle_t handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis) {
    bit48_uint8_buffer_t        tx                 = { CCS811_REG_THRESHOLDS_W, 0, 0, 0, 0, 0 };
    ccs811_threshold_value_t    low_to_med_value   = { .value = low_to_med };
    ccs811_threshold_value_t    med_to_high_value  = { .value = med_to_high };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate eCO2 threshold ranges (400 to 32768) */
    ESP_RETURN_ON_FALSE(!(low_to_med >= med_to_high),           ESP_ERR_INVALID_ARG, TAG, "Low to medium threshold must be less than medium to high threshold");
    ESP_RETURN_ON_FALSE(!(med_to_high <= low_to_med),           ESP_ERR_INVALID_ARG, TAG, "Medium to high threshold must be greater than low to medium threshold");
    ESP_RETURN_ON_FALSE(!(low_to_med < CCS811_ECO2_RANGE_MIN),  ESP_ERR_INVALID_ARG, TAG, "Low to medium threshold must within a range of 400 to 32768 ppm");
    ESP_RETURN_ON_FALSE(!(low_to_med > CCS811_ECO2_RANGE_MAX),  ESP_ERR_INVALID_ARG, TAG, "Low to medium threshold must within a range of 400 to 32768 ppm");
    ESP_RETURN_ON_FALSE(!(med_to_high < CCS811_ECO2_RANGE_MIN), ESP_ERR_INVALID_ARG, TAG, "Medium to high threshold must within a range of 400 to 32768 ppm");
    ESP_RETURN_ON_FALSE(!(med_to_high > CCS811_ECO2_RANGE_MAX), ESP_ERR_INVALID_ARG, TAG, "Medium to high threshold must within a range of 400 to 32768 ppm");

    /* set frame data */
    tx[1] = low_to_med_value.bits.hi_byte;
    tx[2] = low_to_med_value.bits.lo_byte;
    tx[3] = med_to_high_value.bits.hi_byte;
    tx[4] = med_to_high_value.bits.lo_byte;
    tx[5] = hysteresis;

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_write(handle, tx, BIT48_UINT8_BUFFER_SIZE), TAG, "write thresholds failed" );

    return ESP_OK;
}

esp_err_t ccs811_get_baseline_register(ccs811_handle_t handle, uint16_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

     /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_word_from(handle, CCS811_REG_BASELINE_RW, reg), TAG, "read baseline register failed" );

    return ESP_OK;
}

esp_err_t ccs811_set_baseline_register(ccs811_handle_t handle, const uint16_t baseline) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_write_word_to(handle, CCS811_REG_BASELINE_RW, baseline), TAG, "write baseline register failed" );

    return ESP_OK;
}

esp_err_t ccs811_get_hardware_identifier_register(ccs811_handle_t handle, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_byte_from(handle, CCS811_REG_HW_ID_R, reg), TAG, "read hardware identifier register failed" );
    
    return ESP_OK;
}

esp_err_t ccs811_get_hardware_version_register(ccs811_handle_t handle, uint8_t *const reg) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_byte_from(handle, CCS811_REG_HW_VERSION_R, reg), TAG, "read hardware version register failed" );
    
    return ESP_OK;
}

esp_err_t ccs811_start_application(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_write_command(handle, CCS811_REG_APP_START_W), TAG, "write application start register failed" );

    return ESP_OK;
}

/**
 * @brief Initializes CCS811 wake and reset GPIO.
 * 
 * @param handle 
 * @return esp_err_t 
 */
static inline esp_err_t ccs811_init_io(ccs811_handle_t handle) {
    gpio_config_t io_conf = {};
    uint64_t      gpio_pin_sel;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate and init gpio for reset and/or wake pins */
    if(handle->dev_config.reset_io_enabled == true && handle->dev_config.wake_io_enabled == true) {
        // validate reset io num
        ESP_RETURN_ON_ERROR(GPIO_IS_VALID_GPIO(handle->dev_config.reset_io_num), TAG, "reset gpio number is invalid, ccs811 device handle initialization failed");
        // validate wake io num
        ESP_RETURN_ON_ERROR(GPIO_IS_VALID_GPIO(handle->dev_config.wake_io_num), TAG, "wake gpio number is invalid, ccs811 device handle initialization failed");
        // set gpio pin bit mask
        gpio_pin_sel = ((1ULL<<handle->dev_config.reset_io_num) | (1ULL<<handle->dev_config.wake_io_num));
        // interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE; 
        // bit mask of the pins
        io_conf.pin_bit_mask = gpio_pin_sel;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // enable pull-up mode
        io_conf.pull_up_en = 1; 
        // configure gpio with the given settings
        ESP_RETURN_ON_ERROR( gpio_config(&io_conf), TAG, "set gpio configuration for reset and wake failed" );
    } else if(handle->dev_config.reset_io_enabled == true && handle->dev_config.wake_io_enabled == false) {
        // validate reset io num
        ESP_RETURN_ON_ERROR(GPIO_IS_VALID_GPIO(handle->dev_config.reset_io_num),  TAG, "reset gpio number is invalid, ccs811 device handle initialization failed");
        // set gpio pin bit mask
        gpio_pin_sel = (1ULL<<handle->dev_config.reset_io_num);
        // interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE; 
        // bit mask of the pins
        io_conf.pin_bit_mask = gpio_pin_sel;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // enable pull-up mode
        io_conf.pull_up_en = 1; 
        // configure gpio with the given settings
        ESP_RETURN_ON_ERROR( gpio_config(&io_conf), TAG, "set gpio configuration for reset failed" );
    } else if(handle->dev_config.reset_io_enabled == false && handle->dev_config.wake_io_enabled == true) {
        // validate wake io num
        ESP_RETURN_ON_ERROR(GPIO_IS_VALID_GPIO(handle->dev_config.wake_io_num), TAG, "wake gpio number is invalid, ccs811 device handle initialization failed");
        // set gpio pin bit mask
        gpio_pin_sel = (1ULL<<handle->dev_config.wake_io_num);
        // interrupt disabled
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // bit mask of the pins
        io_conf.pin_bit_mask = gpio_pin_sel;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // enable pull-up mode 
        io_conf.pull_up_en = 1;
        // configure gpio with the given settings
        ESP_RETURN_ON_ERROR( gpio_config(&io_conf), TAG, "set gpio configuration for wake failed" );
    }

    /* validate reset gpio to set io state */
    if(handle->dev_config.reset_io_enabled == true) {
        /* active low for reset gpio */
        ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.reset_io_num, 1), TAG, "set reset gpio level high failed (gpio: %i)", handle->dev_config.reset_io_num );
    }

    /* validate wake gpio to wake the device for i2c transactions */
    if(handle->dev_config.wake_io_enabled == true) {
        /* active low for wake gpio */
        ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.wake_io_num, 0), TAG, "set wake gpio level low failed (gpio: %i)", handle->dev_config.wake_io_num );
    }

    return ESP_OK;
}

static inline esp_err_t ccs811_setup(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read status register */
    ccs811_status_register_t status_reg;
    ESP_RETURN_ON_ERROR(ccs811_get_status_register(handle, &status_reg), TAG, "read status register failed");

    /* validate application firmware mode */
    if(status_reg.bits.firmware_mode != CCS811_FW_MODE_APP) {
        /* validate bootloader mode */
        ESP_RETURN_ON_FALSE(status_reg.bits.app_valid, ESP_ERR_INVALID_STATE, TAG, "no valid application for i2c ccs811 device");

        /* attempt tp switch to application mode - start application */
        ESP_RETURN_ON_ERROR(ccs811_start_application(handle), TAG, "application start failed");

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(CCS811_RESET_DELAY_MS));

        /* attempt to read status register */
        ESP_RETURN_ON_ERROR(ccs811_get_status_register(handle, &status_reg), TAG, "read status register failed");

        /* validate application firmware mode switch */
        ESP_RETURN_ON_FALSE(status_reg.bits.firmware_mode == CCS811_FW_MODE_APP, ESP_ERR_INVALID_STATE, TAG, "unable to start application for i2c ccs811 device");
    }

    /* attempt to read hardware identifier */
    ESP_RETURN_ON_ERROR(ccs811_get_hardware_identifier_register(handle, &handle->hardware_id), TAG, "read hardware identifier failed");

    /* attempt to read hardware version */
    ESP_RETURN_ON_ERROR(ccs811_get_hardware_version_register(handle, &handle->hardware_version), TAG, "read hardware version failed");

    ccs811_measure_mode_register_t measure_mode_reg;

    /* attempt to read measure mode register */
    ESP_RETURN_ON_ERROR(ccs811_get_measure_mode_register(handle, &measure_mode_reg), TAG, "read measure mode register failed");

    measure_mode_reg.bits.drive_mode             = handle->dev_config.drive_mode;
    measure_mode_reg.bits.irq_data_ready_enabled = handle->dev_config.irq_data_ready_enabled;
    measure_mode_reg.bits.irq_threshold_enabled  = handle->dev_config.irq_threshold_enabled;

    /* attempt to write measure mode register */
    ESP_RETURN_ON_ERROR(ccs811_set_measure_mode_register(handle, measure_mode_reg), TAG, "write measure mode register failed");

    return ESP_OK;
}

esp_err_t ccs811_init(i2c_master_bus_handle_t master_handle, const ccs811_config_t *ccs811_config, ccs811_handle_t *ccs811_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && ccs811_config );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(CCS811_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, ccs811_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, ccs811 device handle initialization failed", ccs811_config->i2c_address);

    /* validate memory availability for handle */
    ccs811_handle_t out_handle;
    out_handle = (ccs811_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c ccs811 device");

    /* copy configuration */
    out_handle->dev_config = *ccs811_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = out_handle->dev_config.i2c_address,
        .scl_speed_hz       = out_handle->dev_config.i2c_clock_speed,
    };

    /* validate device handle */
    if (out_handle->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &out_handle->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* attempt to init gpio wake and reset */
    ESP_GOTO_ON_ERROR(ccs811_init_io(out_handle), err_handle, TAG, "init wake and reset GPIO failed");

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* attempt to soft-reset */
    ESP_GOTO_ON_ERROR(ccs811_reset(out_handle), err_handle, TAG, "soft-reset failed");

    
    /* set device handle */
    *ccs811_handle = out_handle;

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(CCS811_APPSTART_DELAY_MS));

    return ESP_OK;

    err_handle:
        if (out_handle && out_handle->i2c_handle) {
            i2c_master_bus_rm_device(out_handle->i2c_handle);
        }
        free(out_handle);
    err:
        return ret;
}

esp_err_t ccs811_get_measurement(ccs811_handle_t handle, uint16_t *eco2, uint16_t *etvoc) {
    esp_err_t       ret             = ESP_OK;
    uint64_t        start_time      = 0;
    bool            data_is_ready   = false;
    bool            has_error       = false;
    bit64_uint8_buffer_t   rx              = { };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* set start time for timeout monitoring */
    start_time = esp_timer_get_time(); 

    /* attempt to wait until data is available */
    do {
        /* attempt to check if data is ready */
        ESP_GOTO_ON_ERROR( ccs811_get_data_status(handle, &data_is_ready), err, TAG, "data ready read failed." );

        /* delay task before next i2c transaction */
        vTaskDelay(pdMS_TO_TICKS(CCS811_DATA_READY_DELAY_MS));

        /* validate timeout condition */
        if (ESP_TIMEOUT_CHECK(start_time, ccs811_get_duration_us(handle)))
            return ESP_ERR_TIMEOUT;
    } while (data_is_ready == false);

    /* attempt to read error state */
    ESP_GOTO_ON_ERROR( ccs811_get_error_status(handle, &has_error), err, TAG, "error read failed." );

    /* validate error state */
    if(has_error == true) {
        ccs811_error_code_register_t err_reg;

        /* attempt to read error register */
        ESP_GOTO_ON_ERROR( ccs811_get_error_register(handle, &err_reg), err, TAG, "read error register failed." );

        /* validate error state */
        ESP_GOTO_ON_FALSE(!has_error, ESP_ERR_INVALID_STATE, err, TAG, "error for i2c ccs811 device (%s)", ccs811_err_to_code(err_reg));
    }

    /* attempt i2c write and then read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_from(handle, CCS811_REG_ALG_RESULT_DATA_R, rx, BIT64_UINT8_BUFFER_SIZE), TAG, "read alg result data failed" );

    /* set eco2 and etvoc values */
    *eco2  = rx[1] | (rx[0] << 8);  // big endian order (msb | lsb)
    *etvoc = rx[3] | (rx[2] << 8);  // big endian order (msb | lsb)

    // eco2_data = ((uint16_t)i2c_buf[0] << 8) | ((uint32_t)i2c_buf[1] << 0));

    //ESP_LOGW(TAG, "eco2  hi-byte 0x%02x | lo-byte 0x%02x (value: %d)", alg_result_data[0], alg_result_data[1], eco2_val);
    //ESP_LOGW(TAG, "etvoc hi-byte 0x%02x | lo-byte 0x%02x (value: %d)", alg_result_data[2], alg_result_data[3], etvoc_val);  

    return ESP_OK;

    err:
        return ret;
}

esp_err_t ccs811_set_environmental_data(ccs811_handle_t handle, const float temperature, const float humidity) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_set_environmental_data_register(handle, temperature, humidity), TAG, "write environmental data failed" );

    return ESP_OK;
}

esp_err_t ccs811_set_thresholds(ccs811_handle_t handle, const uint16_t low_to_med, const uint16_t med_to_high, const uint8_t hysteresis) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_set_thresholds_register(handle, low_to_med, med_to_high, hysteresis), TAG, "write environmental data failed" );

    return ESP_OK;
}

esp_err_t ccs811_get_drive_mode(ccs811_handle_t handle, ccs811_drive_modes_t *const mode) {
    ccs811_measure_mode_register_t mmode;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read measure mode register */
    ESP_RETURN_ON_ERROR( ccs811_get_measure_mode_register(handle, &mmode), TAG, "read measure mode register failed" );

    /* set drive mode */
    *mode = mmode.bits.drive_mode;

    return ESP_OK;
}

esp_err_t ccs811_set_drive_mode(ccs811_handle_t handle, const ccs811_drive_modes_t mode) {
    ccs811_measure_mode_register_t mmode;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read measure mode register */
    ESP_RETURN_ON_ERROR( ccs811_get_measure_mode_register(handle, &mmode), TAG, "read measure mode register failed" );

    /* set drive mode */
    mmode.bits.drive_mode = mode;

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ccs811_set_measure_mode_register(handle, mmode), TAG, "write measure mode register failed" );

    return ESP_OK;
}

esp_err_t ccs811_get_firmware_mode(ccs811_handle_t handle, ccs811_firmware_modes_t *const mode) {
    ccs811_status_register_t status;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ccs811_get_status_register(handle, &status), TAG, "read status register (firmware mode state) failed" );

    /* set mode state */
    *mode = status.bits.firmware_mode;

    return ESP_OK;
}

esp_err_t ccs811_get_ntc_resistance(ccs811_handle_t handle, const uint32_t r_ref, uint32_t *const resistance) {
    bit32_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_read_from(handle, CCS811_REG_NTC_R, rx, BIT32_UINT8_BUFFER_SIZE), TAG, "read ntc register failed" );

    uint16_t v_ref = (uint16_t)(rx[0] << 8) | rx[1];
    uint16_t v_ntc = (uint16_t)(rx[2] << 8) | rx[3];

    *resistance = v_ntc * r_ref / v_ref;

    return ESP_OK;
}

esp_err_t ccs811_get_data_status(ccs811_handle_t handle, bool *const ready) {
    ccs811_status_register_t status;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ccs811_get_status_register(handle, &status), TAG, "read status register (data ready state) failed" );

    /* set ready state */
    *ready = status.bits.data_ready;

    return ESP_OK;
}

esp_err_t ccs811_get_error_status(ccs811_handle_t handle, bool *const error) {
    ccs811_status_register_t status;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt to read status register */
    ESP_RETURN_ON_ERROR( ccs811_get_status_register(handle, &status), TAG, "read status (error state) register failed" );

    /* set error state */
    *error = status.bits.error;

    return ESP_OK;
}

esp_err_t ccs811_reset(ccs811_handle_t handle) {
    bit40_uint8_buffer_t tx  = { 0 };

    const static uint8_t sw_reset[4] = CCS811_SW_RESET_DATA;

    /* set frame data */
    tx[0] = CCS811_REG_SW_RESET_W;
    tx[1] = sw_reset[0];
    tx[2] = sw_reset[1];
    tx[3] = sw_reset[2];
    tx[4] = sw_reset[3];

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( ccs811_i2c_write(handle, tx, BIT40_UINT8_BUFFER_SIZE), TAG, "unable to write soft-reset, reset failed" );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(CCS811_RESET_DELAY_MS));

    /* attempt setup */
    ESP_RETURN_ON_ERROR(ccs811_setup(handle), TAG, "unable to setup device, reset failed");

    return ESP_OK;
}

esp_err_t ccs811_io_wake(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate wake io state */
    ESP_RETURN_ON_FALSE(handle->dev_config.wake_io_enabled, ESP_ERR_INVALID_ARG, TAG, "wake gpio must be enabled");

    /* active low for wake - set wake gpio low */
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.wake_io_num, 0), TAG, "set wake gpio level low failed (gpio: %i)", handle->dev_config.wake_io_num );

    /* delay task before next i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(CCS811_WAKE_DELAY_MS));

    return ESP_OK;
}

esp_err_t ccs811_io_sleep(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate wake io state */
    ESP_RETURN_ON_FALSE(handle->dev_config.wake_io_enabled, ESP_ERR_INVALID_ARG, TAG, "wake gpio must be enabled");

    /* active high for sleep - set wake gpio high */
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.wake_io_num, 1), TAG, "set wake gpio level high failed (gpio: %i)", handle->dev_config.wake_io_num );

    return ESP_OK;
}

esp_err_t ccs811_io_reset(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* validate reset io state */
    ESP_RETURN_ON_FALSE(handle->dev_config.reset_io_enabled, ESP_ERR_INVALID_ARG, TAG, "reset gpio must be enabled");

    /* active low for reset - set reset gpio low */
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.reset_io_num, 0), TAG, "set reset gpio level low failed (gpio: %i)", handle->dev_config.reset_io_num );

    /* delay reset gpio in low state - for reset to take effect */
    vTaskDelay(pdMS_TO_TICKS(CCS811_RESET_DELAY_MS));

    /* set reset gpio high - normal state */
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.reset_io_num, 1), TAG, "set reset gpio level high failed (gpio: %i)", handle->dev_config.reset_io_num );

    return ESP_OK;
}

esp_err_t ccs811_remove(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from i2c master bus */
    return i2c_master_bus_rm_device(handle->i2c_handle);
}

esp_err_t ccs811_delete(ccs811_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( ccs811_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char *ccs811_err_to_message(ccs811_error_code_register_t error_reg) {
    /* attempt error message lookup */
    if(error_reg.bits.write_register_invalid == true) return ccs811_error_definition_table[0].msg;
    if(error_reg.bits.read_register_invalid == true) return ccs811_error_definition_table[1].msg;
    if(error_reg.bits.drive_mode_invalid == true) return ccs811_error_definition_table[2].msg;
    if(error_reg.bits.max_resistance_exceeded == true) return ccs811_error_definition_table[3].msg;
    if(error_reg.bits.heater_current_fault == true) return ccs811_error_definition_table[4].msg;
    if(error_reg.bits.heater_voltage_fault == true) return ccs811_error_definition_table[5].msg;

    return ccs811_unknown_msg;
}

const char *ccs811_err_to_code(const ccs811_error_code_register_t error_reg) {
    /* attempt error code lookup */
    if(error_reg.bits.write_register_invalid == true) return ccs811_error_definition_table[0].code;
    if(error_reg.bits.read_register_invalid == true) return ccs811_error_definition_table[1].code;
    if(error_reg.bits.drive_mode_invalid == true) return ccs811_error_definition_table[2].code;
    if(error_reg.bits.max_resistance_exceeded == true) return ccs811_error_definition_table[3].code;
    if(error_reg.bits.heater_current_fault == true) return ccs811_error_definition_table[4].code;
    if(error_reg.bits.heater_voltage_fault == true) return ccs811_error_definition_table[5].code;

    return ccs811_unknown_code;
}

const char *ccs811_measure_mode_description(const ccs811_drive_modes_t mode) {
    /* attempt measure mode lookup */
    for(int i = 0; i< CCS811_MEASURE_MODE_TABLE_SIZE; i++) {
        if(ccs811_measure_mode_definition_table[i].mode == mode) {
            return ccs811_measure_mode_definition_table[i].desc;
        }
    }

    return ccs811_unknown_measure_mode;
}

const char* ccs811_get_fw_version(void) {
    return (const char*)CCS811_FW_VERSION_STR;
}

int32_t ccs811_get_fw_version_number(void) {
    return (int32_t)CCS811_FW_VERSION_INT32;
}