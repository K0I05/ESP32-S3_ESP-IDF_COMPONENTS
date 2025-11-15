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
 * @file max31865.h
 * @defgroup drivers max31865
 * @{
 *
 * ESP-IDF driver for max31865 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __MAX31865_H__
#define __MAX31865_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <driver/spi_master.h>


#ifdef __cplusplus
extern "C"
{
#endif

/*
 * MAX31865 definitions
 */
#define I2C_MAX31865_SCL_SPEED_HZ          UINT32_C(100000) //!< max31865 I2C default clock frequency (100KHz)

#define SPI_MAX31865_DEV_ADDR              UINT8_C(0x38) //!< max31865 I2C address


/*
 * MAX31865 macro definitions
 */
/*
#define I2C_MAX31865_CONFIG_DEFAULT {                      \
    .dev_config.device_address = I2C_MAX31865_DEV_ADDR,    \
    .dev_config.scl_speed_hz   = I2C_MAX31865_SCL_SPEED_HZ,\
    .aht_type = I2C_AHTXX_AHT2X }
*/

typedef enum {
    SPI_MAX31865_FAULT_DETECT_FINISHED       = (0b00), /**<  */
    SPI_MAX31865_FAULT_DETECT_STILL_RUNNING  = (0b01),  /**<  */
    SPI_MAX31865_FAULT_DETECT_CYCLE1_RUNNING = (0b10),
    SPI_MAX31865_FAULT_DETECT_CYCLE2_RUNNING = (0b11)
} spi_max31865_fault_detection_cycles_t;

typedef enum {
    SPI_MAX31865_MODE_SINGLE = 0, /**< Single consersion mode, default */
    SPI_MAX31865_MODE_AUTO        /**< Automatic conversion mode at 50/60Hz rate */
} spi_max31865_modes_t;

typedef enum {
    SPI_MAX31865_FILTER_60HZ = 0, /**< 60Hz */
    SPI_MAX31865_FILTER_50HZ      /**< 50Hz */
} spi_max31865_filters_t;

typedef enum {
    SPI_MAX31865_2WIRE_OR_4WIRE = 0, /**< 2-wires or 4-wires */
    SPI_MAX31865_3WIRE = 1,          /**< 3-wires */
} spi_max31865_connection_types_t;

typedef enum {
    SPI_MAX31865_ITS90 = 0,    /**< ITS-90 */
    SPI_MAX31865_DIN43760,     /**< DIN43760 */
    SPI_MAX31865_US_INDUSTRIAL /**< US INDUSTRIAL */
} spi_max31865_standards_t;


/**
 * @brief MAX31865 SPI configuration register structure.
 *
 */
typedef union __attribute__((packed)) {
    struct {
        spi_max31865_filters_t                  filter:1;               /*!< filter  (bit:0)  */
        bool                                    clear_fault_status:1;   /*!<   (bit:1) */
        spi_max31865_fault_detection_cycles_t   fault_detection:2;      /*!<   (bit:2-3) */
        spi_max31865_connection_types_t         connection:1;           /*!<   (bit:4) */
        bool                                    one_shot_enabled:1;     /*!<   (bit:5) */
        spi_max31865_modes_t                    mode:1;                 /*!<   (bit:6) */
        bool                                    v_bias_enabled:1;       /*!<   (bit:7) */
    } bits;
    uint8_t reg;
} spi_max31865_configuration_register_t;

/**
 * @brief MAX31865 SPI fault status register structure.
 *
 */
typedef union __attribute__((packed)) {
    struct {
        uint8_t reserved:2;                 /*!< reserved  (bit:0-1)  */
        bool voltage_fault:1;               /*!<  (bit:2) */
        bool rtdin_open_fault:1;            /*!<  (bit:3) */
        bool refin_open_fault:1;            /*!<  (bit:4) */
        bool refin_fault:1;                 /*!<  (bit:5) */
        bool rtd_low_threshold_fault:1;     /*!<  (bit:6) */
        bool rtd_high_threshold_fault:1;    /*!<  (bit:7) */
    } bits;
    uint8_t reg;
} spi_max31865_fault_status_register_t;

/**
 * @brief MAX31865 SPI device configuration structure.
 */
typedef struct {
    int                                     clock_speed_hz;
    int                                     input_delay_ns;
    spi_max31865_modes_t                    mode;
    spi_max31865_connection_types_t         connection;
    bool                                    v_bias_enabled;
    spi_max31865_filters_t                  filter;
    spi_host_device_t                       host;           /*!< The SPI host used, set before calling `spi_eeprom_init()` */
    gpio_num_t                              cs_io_num;      /*!< CS gpio number, set before calling `spi_eeprom_init()` */
    gpio_num_t                              miso_io_num;    /*!< MISO gpio number, set before calling `spi_eeprom_init()` */
    bool                                    irq_enabled;    /*!< Whether to use polling or interrupt when waiting for write to be done. Set before calling `spi_eeprom_init()`. */
    gpio_num_t                              irq_io_num;     /*!< interrupt gpio number */
} spi_max31865_config_t;

/**
 * @brief MAX31865 SPI device structure.
 */
struct spi_max31865_t {
    SemaphoreHandle_t                       spi_ready_sem;  /*!< SPI semaphore for ready signal */
    spi_max31865_config_t                   spi_dev_config; /*!< SPI device configuration */
    spi_device_handle_t                     spi_dev_handle; /*!< SPI device handle */
    spi_max31865_configuration_register_t   config_reg;     /*!< status register */
    spi_max31865_standards_t                standard;       /*!< Temperature scale standard */
    float                                   r_ref;          /*!< Reference resistor value, Ohms */
    float                                   rtd_nominal;    /*!< RTD nominal resistance at 0 deg. C, Ohms (PT100 - 100 Ohms, PT1000 - 1000 Ohms) */
};

/**
 * @brief MAX31865 SPI device structure definitions.
 */
typedef struct spi_max31865_t spi_max31865_t;

/**
 * @brief MAX31865 SPI device handle structure.
 */
typedef struct spi_max31865_t *spi_max31865_handle_t;



esp_err_t spi_max31865_get_configuration_register(spi_max31865_handle_t max31865_handle);
esp_err_t spi_max31865_set_configuration_register(spi_max31865_handle_t max31865_handle, const spi_max31865_configuration_register_t config_reg);


/**
 * @brief Read fault status register from MAX31865.
 *
 * @param max31865_handle MAX31865 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t spi_max31865_get_fault_status_register(spi_max31865_handle_t max31865_handle);

/**
 * @brief Initializes an MAX31865 device onto the I2C master bus.
 *
 * @param[in] max31865_config Configuration of MAX31865 device.
 * @param[out] max31865_handle MAX31865 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t spi_max31865_init(const spi_max31865_config_t *max31865_config, spi_max31865_handle_t *max31865_handle);

/**
 * @brief Reads temperature from MAX31865.
 *
 * @param max31865_handle MAX31865 device handle.
 * @param temperature Temperature in degree Celsius.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t spi_max31865_get_measurement(spi_max31865_handle_t max31865_handle, float *const temperature);


/**
 * @brief Removes an MAX31865 device from master bus.
 *
 * @param[in] max31865_handle MAX31865 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t spi_max31865_remove(spi_max31865_handle_t max31865_handle);

/**
 * @brief Removes an MAX31865 device from master bus and frees handle.
 * 
 * @param max31865_handle MAX31865 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t spi_max31865_delete(spi_max31865_handle_t max31865_handle);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif // __MAX31865_H__
