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
 * @file as3935.h
 * @defgroup drivers as3935
 * @{
 *
 * ESP-IDF driver for as3935 sensor
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __AS3935_H__
#define __AS3935_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_types.h>
#include <esp_event.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <type_utils.h>
#include <driver/gpio.h>
#include "as3935_version.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief AS3935 definitions
*/
/* AS3935 I2C constants */
#define I2C_AS3935_DEV_CLK_SPD           UINT32_C(100000)       //!< as3935 I2C default clock frequency (100KHz)

/* AS3935 I2C addresses */
#define I2C_AS3935_DEV_ADDR_1            UINT8_C(0x01)   //!< as3935 I2C address when ADD1 pin floating/low and ADD0 pin high
#define I2C_AS3935_DEV_ADDR_2            UINT8_C(0x02)   //!< as3935 I2C address when ADD0 pin floating/low and ADD1 pin high
#define I2C_AS3935_DEV_ADDR_3            UINT8_C(0x03)   //!< as3935 I2C address when ADD0 and ADD1 pins high

/* AS3935 registers */
#define AS3935_REG_00                UINT8_C(0x00)   //!< as3935 I2C register to access AFE gain boost and power-down
#define AS3935_REG_01                UINT8_C(0x01)   //!< as3935 I2C register to access noise floor level and watchdog threshold (0-15)
#define AS3935_REG_02                UINT8_C(0x02)   //!< as3935 I2C register to access clearing statistics, minimum number of lightning, and spike rejection
#define AS3935_REG_03                UINT8_C(0x03)   //!< as3935 I2C register to access frequency division ratio and mask disturber
#define AS3935_REG_04                UINT8_C(0x04)   //!< as3935 I2C register to access energy of lightning (LSBYTE)
#define AS3935_REG_05                UINT8_C(0x05)   //!< as3935 I2C register to access energy of lightning (MSBYTE)
#define AS3935_REG_06                UINT8_C(0x06)   //!< as3935 I2C register to access energy of lightning (MMSBYTE)
#define AS3935_REG_07                UINT8_C(0x07)   //!< as3935 I2C register to access distance estimation of lightning
#define AS3935_REG_08                UINT8_C(0x08)   //!< as3935 I2C register to access internal tuning caps and display (LCO, SRCO, TRCO) on IRQ pin
#define AS3935_REG_RST               UINT8_C(0x96)   //!< as3935 I2C register to either calibrate or reset registers to default
/* AS3935 command */
#define AS3935_CMD_PRESET_DEFAULT    UINT8_C(0x3c)   //!< as3935 I2C command to set all registers in default mode
#define AS3935_CMD_CALIB_RCO         UINT8_C(0x3d)   //!< as3935 I2C command to automatically calibrate the internal RC oscillators

#define AS3935_POWERUP_DELAY_MS      (25)
#define AS3935_APPSTART_DELAY_MS     (25)
#define AS3935_STARTUP_DELAY_MS      (2)            //!< as3935 I2C LCO start-up delay in milliseconds
#define AS3935_INTERRUPT_DELAY_MS    (2)            //!< as3935 I2C interrupt delay in milliseconds
#define AS3935_CALIBRATION_DELAY_MS  (2)            //!< as3935 I2C calibration delay in milliseconds for RC oscillators
#define AS3935_TX_RX_DELAY_MS           UINT16_C(10)

/**
 * @brief declare of AS3935 monitor event base.
 */
ESP_EVENT_DECLARE_BASE(ESP_AS3935_EVENT);

/**
 * @brief CJMCU-3935 Board Wiring (I2C interface):
 *  ADD0 & ADD1 pin to VCC (3.3v) -> I2C address 0x03
 *  SI pin to VCC (3.3v) -> enable I2C interface
 *  SCL pin 10k-ohm pull-up resistor to VCC (3.3v)
 *  SCL pin to MCU SCL pin
 *  MOSI (SDA) pin to MCU SDA pin
 *  IRQ pin to MCU interrupt pin
 *  VCC pin to 3.3v
 *  GND pin to common ground
*/

/**
 * @brief AS3935 macro definitions
*/
#define AS3935_CONFIG_DEFAULT {                                             \
        .i2c_clock_speed                = I2C_AS3935_DEV_CLK_SPD,               \
        .i2c_address                    = I2C_AS3935_DEV_ADDR_3,                \
        .irq_io_enabled                 = true,                                 \
        .irq_io_num                     = GPIO_NUM_10,                          \
        .analog_frontend                = AS3935_AFE_INDOOR,                    \
        .min_lightning_strikes          = AS3935_MIN_LIGHTNING_9,               \
        .calibrate_rco                  = true,                                 \
        .disturber_detection_enabled    = true,                                 \
        .noise_level_threshold          = AS3935_NOISE_LEVEL_1140_95            \
    }


/**
 * @brief AS3935 monitor handle definition
 *
 */
typedef void *as3935_monitor_handle_t;

/**
 * @brief AS3935 enumerator and structure declarations
*/

/**
 * @brief AS3935 analog frontends (AFE) REG0x00[5:1] enumerator.
 */
typedef enum as3935_analog_frontends_e {
    AS3935_AFE_INDOOR  = (0b10010),
    AS3935_AFE_OUTDOOR = (0b01110)
} as3935_analog_frontends_t;

/**
 * @brief AS3935 power states REG0x00[0] enumerator.
 */
typedef enum as3935_power_states_e {
    AS3935_POWER_OFF  = 1,
    AS3935_POWER_ON   = 0
} as3935_power_states_t;

/**
 * @brief AS3935 clear statistics states REG0x02[6] enumerator.
 */
typedef enum as3935_clear_statistics_states_e {
    AS3935_CLEAR_STATS_DISABLED  = 1,
    AS3935_CLEAR_STATS_ENABLED   = 0
} as3935_clear_statistics_states_t;

/**
 * @brief AS3935 disturber detection states REG0x03[5] enumerator.
 */
typedef enum as3935_disturber_detection_states_e {
    AS3935_DISTURBER_DETECTION_ENABLED  = 0,
    AS3935_DISTURBER_DETECTION_DISABLED = 1
} as3935_disturber_detection_states_t;

/**
 * @brief AS3935 CO IRQ pin states REG0x08[5]|[6]|[7] enumerator.
 */
typedef enum as3935_co_irq_pin_states_e {
    AS3935_CO_IRQ_PIN_ENABLED   = 1,
    AS3935_CO_IRQ_PIN_DISABLED  = 0
} as3935_co_irq_pin_states_t;

/**
* @brief AS3935 watchdog thresholds REG0x01[3:0] enumerator.
*/
typedef enum as3935_watchdog_thresholds_e {
    AS3935_WD_THRESHOLD_0  = (0b0000),
    AS3935_WD_THRESHOLD_1  = (0b0001),
    AS3935_WD_THRESHOLD_2  = (0b0010),
    AS3935_WD_THRESHOLD_3  = (0b0011),
    AS3935_WD_THRESHOLD_4  = (0b0100),
    AS3935_WD_THRESHOLD_5  = (0b0101),
    AS3935_WD_THRESHOLD_6  = (0b0110),
    AS3935_WD_THRESHOLD_7  = (0b0111),
    AS3935_WD_THRESHOLD_8  = (0b1000),
    AS3935_WD_THRESHOLD_9  = (0b1001),
    AS3935_WD_THRESHOLD_10 = (0b1010)
} as3935_watchdog_thresholds_t;

/**
* @brief AS3935 noise floor generator and evaluation REG0x01[6:4] enumerator.
*/
typedef enum as3935_noise_levels_e {
    AS3935_NOISE_LEVEL_390_28   = (0b000),  /*!< 390 uVrms outdoor and 28 uVrms indoor */
    AS3935_NOISE_LEVEL_630_45   = (0b001),  /*!< 630 uVrms outdoor and 45 uVrms indoor */
    AS3935_NOISE_LEVEL_860_62   = (0b010),  /*!< 860 uVrms outdoor and 63 uVrms indoor (default) */
    AS3935_NOISE_LEVEL_1100_78  = (0b011),  /*!< 1100 uVrms outdoor and 78 uVrms indoor */
    AS3935_NOISE_LEVEL_1140_95  = (0b100),  /*!< 1140 uVrms outdoor and 95 uVrms indoor */
    AS3935_NOISE_LEVEL_1570_112 = (0b101),  /*!< 1570 uVrms outdoor and 112 uVrms indoor */
    AS3935_NOISE_LEVEL_1800_130 = (0b110),  /*!< 1800 uVrms outdoor and 130 uVrms indoor */
    AS3935_NOISE_LEVEL_2000_146 = (0b111)   /*!< 2000 uVrms outdoor and 146 uVrms indoor */
} as3935_noise_levels_t;

/**
* @brief AS3935 interrupt states REG0x03[3:0] enumerator.
*/
typedef enum as3935_interrupt_states_e {
    AS3935_INT_NOISE     = (0b0001),    /*!< noise level too high */
    AS3935_INT_DISTURBER = (0b0100),    /*!< disturber detected */
    AS3935_INT_LIGHTNING = (0b1000),    /*!< lightning detected */
    AS3935_INT_NONE      = (0b0000)     /*!< no interrupt */
} as3935_interrupt_states_t;

/**
* @brief AS3935 minimum number of lightning detections REG0x02[5:4] enumerator.
*/
typedef enum as3935_minimum_lightnings_e {
    AS3935_MIN_LIGHTNING_1  = (0b00),
    AS3935_MIN_LIGHTNING_5  = (0b01),
    AS3935_MIN_LIGHTNING_9  = (0b10),
    AS3935_MIN_LIGHTNING_16 = (0b11)
} as3935_minimum_lightnings_t;

/**
* @brief AS3935 frequency division ratio for antenna tunning REG0x03[7:3] LCO_FDIV enumerator.
*/
typedef enum as3935_frequency_division_ratios_e {
    AS3935_FREQ_DIV_RATIO_16  = (0b00),
    AS3935_FREQ_DIV_RATIO_32  = (0b01),
    AS3935_FREQ_DIV_RATIO_64  = (0b10),
    AS3935_FREQ_DIV_RATIO_128 = (0b11)
} as3935_frequency_division_ratios_t;

/**
* @brief AS3935 lightning estimated distances REG0x07[5:0] enumerator.
*/
typedef enum as3935_lightning_distances_e {
    AS3935_L_DISTANCE_OVERHEAD  = (0b000001),
    AS3935_L_DISTANCE_5KM       = (0b000101),
    AS3935_L_DISTANCE_6KM       = (0b000110),
    AS3935_L_DISTANCE_8KM       = (0b001000),
    AS3935_L_DISTANCE_10KM      = (0b001010),
    AS3935_L_DISTANCE_12KM      = (0b001100),
    AS3935_L_DISTANCE_14KM      = (0b001110),
    AS3935_L_DISTANCE_17KM      = (0b010001),
    AS3935_L_DISTANCE_20KM      = (0b010100),
    AS3935_L_DISTANCE_24KM      = (0b011000),
    AS3935_L_DISTANCE_27KM      = (0b011011),
    AS3935_L_DISTANCE_31KM      = (0b011111),
    AS3935_L_DISTANCE_34KM      = (0b100010),
    AS3935_L_DISTANCE_37KM      = (0b100101),
    AS3935_L_DISTANCE_40KM      = (0b101000),
    AS3935_L_DISTANCE_OO_RANGE  = (0b111111)
} as3935_lightning_distances_t;

/**
 * @brief AS3935 oscillator calibration status results enumerator.
 */
typedef enum as3935_rco_calibration_results_e {
    AS3935_RCO_CALIBRATION_SUCCESSFUL,
    AS3935_RCO_CALIBRATION_UNSUCCESSFUL,
    AS3935_RCO_CALIBRATION_INCOMPLETE,
} as3935_rco_calibration_results_t;

/**
 * @brief AS3935 tuning mode oscillators enumerator.
 */
typedef enum as3935_oscillator_modes_e {
    AS3935_OSCILLATOR_ANTENNA_LC,
    AS3935_OSCILLATOR_TIMER_RC,
    AS3935_OSCILLATOR_SYSTEM_RC,
} as3935_oscillator_modes_t;

/**
 * @brief AS3935 register 0x00 structure.
 */
typedef union __attribute__((packed)) as3935_0x00_register_u {
    struct REG_0x00_BIT_TAG {
        as3935_power_states_t       power_state:1;  /*!< power on or off                        (bit:0)   */
        as3935_analog_frontends_t   analog_frontend:5; /*!< analog front-end (AFE) and watchdog (bit:1-5) */
        uint8_t                     reserved:2;     /*!< reserved and set to 0                  (bit:6-7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} as3935_0x00_register_t;

/**
 * @brief AS3935 register 0x01 structure.
 */
typedef union __attribute__((packed)) as3935_0x01_register_u {
    struct REG_0x01_BIT_TAG {
        as3935_watchdog_thresholds_t watchdog_threshold:4; /*!< watchdog threshold  (bit:0-3) */
        as3935_noise_levels_t noise_floor_level:3;   /*!< noise floor level        (bit:4-6) */
        uint8_t         reserved:1;             /*!< reserved and set to 0             (bit:7)   */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} as3935_0x01_register_t;

/**
 * @brief AS3935 register 0x02 structure.
 */
typedef union __attribute__((packed)) as3935_0x02_register_u {
    struct REG_0x02_BIT_TAG {
        uint8_t                     spike_rejection:4;      /*!< spike rejection                (bit:0-3) */
        as3935_minimum_lightnings_t min_num_lightning:2; /*!< minimum number of lightning   (bit:4-5) */
        as3935_clear_statistics_states_t clear_stats_state:1; /*!< clear statistics         (bit:6)   */
        uint8_t                     reserved:1;             /*!< reserved and set to 0          (bit:7)   */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} as3935_0x02_register_t;

/**
 * @brief AS3935 register 0x03 structure.
 */
typedef union __attribute__((packed)) as3935_0x03_register_u {
    struct REG_0x03_BIT_TAG {
        as3935_interrupt_states_t           irq_state:4;    /*!< lightning event interrupt                  (bit:0-3) */
        uint8_t                             reserved:1;     /*!< reserved and set to 0                      (bit:4)   */
        as3935_disturber_detection_states_t disturber_detection_state:1;    /*!< disturber detection state  (bit:5)   */
        as3935_frequency_division_ratios_t  freq_div_ratio:2;   /*!< lco frequency division                 (bit:6-7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} as3935_0x03_register_t;

/**
 * @brief AS3935 register 0x07 structure.
 */
typedef union __attribute__((packed)) as3935_0x07_register_u {
    struct REG_0x07_BIT_TAG {
        as3935_lightning_distances_t  lightning_distance:6; /*!< lightning distance estimation (bit:0-5) */
        uint8_t                       reserved:2;     /*!< reserved and set to 0               (bit:6-7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} as3935_0x07_register_t;

/**
 * @brief AS3935 register 0x08 structure.
 */
typedef union __attribute__((packed)) as3935_0x08_register_u {
    struct REG_0x08_BIT_TAG {
        uint8_t   tuning_capacitors:4;                  /*!< internal tuning capacitors from 0 to 120pF in steps of 8pF (0-15) (bit:0-3) */
        uint8_t   reserved:1;                           /*!< reserved and set to 0           (bit:4) */
        as3935_co_irq_pin_states_t display_trco_state:1; /*!< display trco on irq pin    (bit:5) */
        as3935_co_irq_pin_states_t display_srco_state:1; /*!< display srco on irq pin    (bit:6) */
        as3935_co_irq_pin_states_t display_lco_state:1;  /*!< display lco on irq pin     (bit:7) */
    } bits;          /*!< represents the 8-bit control register parts in bits. */
    uint8_t reg;    /*!< represents the 8-bit control register as `uint8_t` */
} as3935_0x08_register_t;


/**
 * @brief AS3935 device configuration structure definition.
 */
typedef struct as3935_config_s {
    uint16_t                            i2c_address;                /*!< as3935 i2c device address */
    uint32_t                            i2c_clock_speed;            /*!< as3935 i2c device scl clock speed in hz */
    bool                                irq_io_enabled;             /*!< as3935 interrupt pin enabled when true */
    uint32_t                            irq_io_num;                 /*!< as3935 pin number for mcu interrupt */
    as3935_analog_frontends_t           analog_frontend;            /*!< as3935 indoor or outdoor exposure */
    as3935_minimum_lightnings_t         min_lightning_strikes;      /*!< as3935 minimum number of lightning strikes before an event is raised */
    bool                                calibrate_rco;              /*!< as3935 rco is calibrated when true */
    bool                                disturber_detection_enabled;/*!< as3935 disturber detection is enabled when true */
    as3935_noise_levels_t               noise_level_threshold;      /*!< as3935 noise level threshold */
} as3935_config_t;


/**
 * @brief AS3935 opaque handle structure definition.
*/
typedef void* as3935_handle_t;

typedef gpio_isr_t as3935_isr_t;

/**
 * @brief AS3935 device event object structure.
 */
typedef struct as3935_monitor_base_s {
    as3935_lightning_distances_t    lightning_distance;
    uint32_t                        lightning_energy; 
} as3935_monitor_base_t;

/**
 * @brief esp AS3935 device state machine structure.
*/
typedef struct as3935_monitor_context_s {
    uint32_t                irq_io_num;          /*!< as3935 interrupt pin to mcu */
    as3935_monitor_base_t   base;                /*!< as3935 device parent class */     
    esp_event_loop_handle_t event_loop_handle;   /*!< as3935 event loop handle */
    QueueHandle_t           event_queue_handle;  /*!< as3935 event queue handle */ 
    TaskHandle_t            task_monitor_handle; /*!< as3935 task monitor handle */ 
    as3935_handle_t         as3935_handle;       /*!< as3935 handle */
    SemaphoreHandle_t       i2c_mutex_handle;    /*!< I2C master bus mutex handle */
} as3935_monitor_context_t;




/**
 * @brief initialize AS3935 monitor instance.
 * 
 * @param[in] master_handle i2c master bus handle.
 * @param[in] as3935_config AS3935 configuration.
 * @return as3935_monitor_handle_t AS3935 monitor handle.
 */
esp_err_t as3935_monitor_init(i2c_master_bus_handle_t master_handle, const as3935_config_t *as3935_config, as3935_monitor_handle_t *monitor_handle);

//as3935_monitor_handle_t as3935_monitor_init(i2c_master_bus_handle_t master_handle, const as3935_config_t *as3935_config);

/**
 * @brief de-initialize AS3935 monitor instance.
 * 
 * @param[in] monitor_handle AS3935 monitor handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_monitor_deinit(as3935_monitor_handle_t monitor_handle);

/**
 * @brief adds user defined event handler for AS3935 monitor.
 * 
 * @param[in] monitor_handle AS3935 monitor handle.
 * @param[in] event_handler user defined event handler.
 * @param[out] handler_args handler specific arguments.
 * @return esp_err_t
 *  - ESP_OK on success.
 *  - ESP_ERR_NO_MEM when unable to allocate memory for the handler.
 *  - ESP_ERR_INVALIG_ARG when invalid combination of event base and event id is provided.
 */
esp_err_t as3935_monitor_add_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler, void *handler_args);

/**
 * @brief removes user defined event handler for AS3935 monitor.
 * 
 * @param[in] monitor_handle AS3935 monitor handle.
 * @param[in] event_handler user defined event handler.
 * @return esp_err_t
 *  - ESP_OK on success.
 *  - ESP_ERR_INVALIG_ARG when invalid combination of event base and event id is provided.
 */
esp_err_t as3935_monitor_remove_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler);

/**
 * @brief gets 0x00 register from AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_get_0x00_register(as3935_handle_t handle, as3935_0x00_register_t *const reg);

/**
 * @brief sets 0x00 register on AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[in] reg AS3935 0x00 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_set_0x00_register(as3935_handle_t handle, const as3935_0x00_register_t reg);

/**
 * @brief gets 0x01 register from AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[out] reg AS3935 0x01 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_get_0x01_register(as3935_handle_t handle, as3935_0x01_register_t *const reg);

/**
 * @brief sets 0x01 register on AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[in] reg AS3935 0x01 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_set_0x01_register(as3935_handle_t handle, const as3935_0x01_register_t reg);

/**
 * @brief gets 0x02 register from AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[out] reg AS3935 0x02 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_get_0x02_register(as3935_handle_t handle, as3935_0x02_register_t *const reg);

/**
 * @brief sets 0x02 register on AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[in] reg AS3935 0x02 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_set_0x02_register(as3935_handle_t handle, const as3935_0x02_register_t reg);

/**
 * @brief gets 0x03 register from AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[out] reg AS3935 0x03 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_get_0x03_register(as3935_handle_t handle, as3935_0x03_register_t *const reg);

/**
 * @brief sets 0x03 register on AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[in] reg AS3935 0x03 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_set_0x03_register(as3935_handle_t handle, const as3935_0x03_register_t reg);

/**
 * @brief gets 0x08 register from AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[out] reg AS3935 0x08 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_get_0x08_register(as3935_handle_t handle, as3935_0x08_register_t *const reg);

/**
 * @brief sets 0x08 register on AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[in] reg AS3935 0x08 register structure.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_set_0x08_register(as3935_handle_t handle, const as3935_0x08_register_t reg);

/**
 * @brief initializes an AS3935 device onto the I2C master bus.
 *
 * @param[in] master_handle I2C master bus handle.
 * @param[in] as3935_config AS3935 device configuration.
 * @param[out] as3935_handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_init(i2c_master_bus_handle_t master_handle, const as3935_config_t *as3935_config, as3935_handle_t *as3935_handle);

esp_err_t as3935_register_isr(as3935_handle_t handle, const as3935_isr_t isr);

/**
 * @brief resets AS3935 to defaults.
 * 
 * @param[in] handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_reset_to_defaults(as3935_handle_t handle);

/**
 * @brief calibrates AS3935 RC oscillator.
 * 
 * @param[in] handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_calibrate_rco(as3935_handle_t handle); // crash!!!

/**
 * @brief clears AS3935 lightning statistics.
 * 
 * @param[in] handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_clear_lightning_statistics(as3935_handle_t handle);

esp_err_t as3935_enable_power(as3935_handle_t handle);
esp_err_t as3935_get_analog_frontend(as3935_handle_t handle, as3935_analog_frontends_t *const analog_frontend);
esp_err_t as3935_get_watchdog_threshold(as3935_handle_t handle, as3935_watchdog_thresholds_t *const watchdog_threshold);
esp_err_t as3935_get_noise_floor_threshold(as3935_handle_t handle, as3935_noise_levels_t *const noise_level);
esp_err_t as3935_get_spike_rejection(as3935_handle_t handle, uint8_t *const spike_rejection);
esp_err_t as3935_get_minimum_lightnings(as3935_handle_t handle, as3935_minimum_lightnings_t *const min_lightnings);
esp_err_t as3935_enable_disturber_detection(as3935_handle_t handle);
esp_err_t as3935_get_frequency_division_ratio(as3935_handle_t handle, as3935_frequency_division_ratios_t *const ratio);
esp_err_t as3935_get_display_oscillator_on_irq(as3935_handle_t handle, as3935_oscillator_modes_t oscillator_mode, bool *const enabled);
esp_err_t as3935_get_internal_capacitors(as3935_handle_t handle, uint8_t *const value);

esp_err_t as3935_disable_power(as3935_handle_t handle);
esp_err_t as3935_set_analog_frontend(as3935_handle_t handle, const as3935_analog_frontends_t analog_frontend);
esp_err_t as3935_set_watchdog_threshold(as3935_handle_t handle, const as3935_watchdog_thresholds_t watchdog_threshold);
esp_err_t as3935_set_noise_floor_threshold(as3935_handle_t handle, const as3935_noise_levels_t noise_level);
esp_err_t as3935_set_spike_rejection(as3935_handle_t handle, const uint8_t spike_rejection);
esp_err_t as3935_set_minimum_lightnings(as3935_handle_t handle, const as3935_minimum_lightnings_t min_lightnings);
esp_err_t as3935_disable_disturber_detection(as3935_handle_t handle);
esp_err_t as3935_set_frequency_division_ratio(as3935_handle_t handle, const as3935_frequency_division_ratios_t ratio);
esp_err_t as3935_set_display_oscillator_on_irq(as3935_handle_t handle, const as3935_oscillator_modes_t oscillator_mode, const bool enabled);
esp_err_t as3935_set_internal_capacitors(as3935_handle_t handle, const uint8_t value);

/**
 * @brief gets interrupt state of AS3935.
 * 
 * @param[in] handle AS3935 device handle.
 * @param[out] state interrupt state of AS3935.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_get_interrupt_state(as3935_handle_t handle, as3935_interrupt_states_t *const state);
esp_err_t as3935_get_lightning_energy(as3935_handle_t handle, uint32_t *const energy);
esp_err_t as3935_get_lightning_distance(as3935_handle_t handle, as3935_lightning_distances_t *const distance);
esp_err_t as3935_get_lightning_distance_km(as3935_handle_t handle, uint8_t *const distance);
esp_err_t as3935_get_lightning_event(as3935_handle_t handle, as3935_lightning_distances_t *const distance, uint32_t *const energy);

/**
 * @brief Removes an AS3935 device from I2C master bus.
 *
 * @param[in] handle as3935 device handle
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_remove(as3935_handle_t handle);

/**
 * @brief Removes an AS3935 device from master bus and frees handle.
 * 
 * @param handle AS3935 device handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t as3935_delete(as3935_handle_t handle);

/**
 * @brief Converts AS3935 firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* AS3935 firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* as3935_get_fw_version(void);

/**
 * @brief Converts AS3935 firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t AS3935 firmware version number.
 */
int32_t as3935_get_fw_version_number(void);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __AS3935_H__
