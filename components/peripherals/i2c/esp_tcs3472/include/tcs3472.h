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
 * ESP-IDF driver for tcs3472 RGB sensor
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
#define I2C_TCS3472_CONFIG_DEFAULT {                                               \
            .i2c_address                = I2C_TCS3472_DEV_ADDR,                    \
            .i2c_clock_speed            = I2C_TCS3472_DEV_CLK_SPD, }

/*
 * TCS3472 enumerator and structure declarations
*/

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
        tcs3472_irq_rates_t irq_persistence:4; /*!< interrupt persistence, controls rate of interrupt to the host processor (bit:0-3)  */
        uint8_t reserved1:4;     /*!< reserved (bit:4-7) */
    } bits;
    uint8_t reg;
} tcs3472_persistence_register_t;





/**
 * @brief TCS3472 device configuration structure definition.
 */
typedef struct tcs3472_config_s {
    uint16_t                            i2c_address;            /*!< veml7700 i2c device address */
    uint32_t                            i2c_clock_speed;        /*!< veml7700 i2c device scl clock speed  */
} tcs3472_config_t;


/**
 * @brief TCS3472 opaque handle structure definition.
 */
typedef void* tcs3472_handle_t;




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
 * @brief Removes an TCS3472 device from master bus.
 *
 * @param[in] handle TCS3472 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t tcs3472_remove(tcs3472_handle_t handle);

/**
 * @brief Removes an TCS3472 device from master I2C bus and delete the handle.
 * 
 * @param handle TCS3472 device handle.
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
