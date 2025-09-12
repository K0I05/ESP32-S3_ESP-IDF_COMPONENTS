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
 * @file as3935.c
 *
 * ESP-IDF driver for AS3935 lightning detection sensor
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/as3935.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <esp_log.h>
#include <esp_check.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * @brief AS3935 definitions
*/
/* AS3935 localized constants */
#define AS3935_IRQ_FLAG_DEFAULT         (0)
#define AS3935_MUTEX_WAIT_TIME          (50)  // ticks
#define AS3935_EVENT_LOOP_POOL_DELAY_MS (50)  // milliseconds
#define AS3935_EVENT_LOOP_POST_DELAY_MS (100) // milliseconds
#define AS3935_EVENT_LOOP_QUEUE_SIZE    (16)
#define AS3935_EVENT_TASK_NAME          "as3935_evt_tsk"
#define AS3935_EVENT_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 5)
#define AS3935_EVENT_TASK_PRIORITY      (tskIDLE_PRIORITY + 6)

#define I2C_XFR_TIMEOUT_MS      (500)          //!< I2C transaction timeout in milliseconds

/** 
 * macro definitions
*/
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define ENSURE_TRUE(ACTION) do { BaseType_t __res = (ACTION); assert(__res == pdTRUE); (void)__res; } while (0)

/**
 * @brief AS3935 device structure definition.
 */
typedef struct as3935_device_s {
    as3935_config_t             config;         /*!< as3935 configuration */
    i2c_master_dev_handle_t     i2c_handle;     /*!< as3935 I2C device handle */
} as3935_device_t;

/**
 * @brief AS3935 monitor event base definition.
 */
ESP_EVENT_DEFINE_BASE(ESP_AS3935_EVENT);


/**
 * static constant declarations
 */
static const char *TAG = "as3935";


/** 
 * functions and subroutines
*/

/**
 * @brief AS3935 I2C HAL read from register address transaction.  This is a write and then read process.
 * 
 * @param device AS3935 device descriptor.
 * @param reg_addr AS3935 register address to read from.
 * @param buffer Buffer to store results from read transaction.
 * @param size Length of buffer to store results from read transaction.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as3935_i2c_read_from(as3935_device_t *const device, const uint8_t reg_addr, uint8_t *buffer, const uint8_t size) {
    const bit8_uint8_buffer_t tx = { reg_addr };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, buffer, size, I2C_XFR_TIMEOUT_MS), TAG, "as3935_i2c_read_from failed" );

    return ESP_OK;
}


/**
 * @brief AS3935 I2C HAL read byte from register address transaction.
 * 
 * @param device AS3935 device descriptor.
 * @param reg_addr AS3935 register address to read from.
 * @param byte AS3935 read transaction return byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as3935_i2c_read_byte_from(as3935_device_t *const device, const uint8_t reg_addr, uint8_t *const byte) {
    const bit8_uint8_buffer_t tx = { reg_addr };
    bit8_uint8_buffer_t rx = { 0 };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    ESP_RETURN_ON_ERROR( i2c_master_transmit_receive(device->i2c_handle, tx, BIT8_UINT8_BUFFER_SIZE, rx, BIT8_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "as3935_i2c_read_byte_from failed" );

    /* set output parameter */
    *byte = rx[0];

    return ESP_OK;
}

/**
 * @brief AS3935 I2C HAL write byte to register address transaction.
 * 
 * @param device AS3935 device descriptor.
 * @param reg_addr AS3935 register address to write to.
 * @param byte AS3935 write transaction input byte.
 * @return esp_err_t ESP_OK on success.
 */
static inline esp_err_t as3935_i2c_write_byte_to(as3935_device_t *const device, const uint8_t reg_addr, const uint8_t byte) {
    const bit16_uint8_buffer_t tx = { reg_addr, byte };

    /* validate arguments */
    ESP_ARG_CHECK( device );

    /* attempt i2c write transaction */
    ESP_RETURN_ON_ERROR( i2c_master_transmit(device->i2c_handle, tx, BIT16_UINT8_BUFFER_SIZE, I2C_XFR_TIMEOUT_MS), TAG, "i2c_master_transmit, i2c write failed" );
                        
    return ESP_OK;
}

/**
 * @brief Converts lightning distance enumerator to a distance in kilometers.
 * 
 * @param distance AS3935 lightning distance enumerator.
 * @return uint8_t Lightning distance in kilometers.
 */
static inline uint8_t as3935_convert_distance_km(as3935_lightning_distances_t distance) {
    switch(distance) {
        case AS3935_L_DISTANCE_OVERHEAD:
            return 0;
        case AS3935_L_DISTANCE_5KM:
            return 5;
        case AS3935_L_DISTANCE_6KM:
            return 6;
        case AS3935_L_DISTANCE_8KM:
            return 8;
        case AS3935_L_DISTANCE_10KM:
            return 10;
        case AS3935_L_DISTANCE_12KM:
            return 12;
        case AS3935_L_DISTANCE_14KM:
            return 14;
        case AS3935_L_DISTANCE_17KM:
            return 17;
        case AS3935_L_DISTANCE_20KM:
            return 20;
        case AS3935_L_DISTANCE_24KM:
            return 24;
        case AS3935_L_DISTANCE_27KM:
            return 27;
        case AS3935_L_DISTANCE_31KM:
            return 31;
        case AS3935_L_DISTANCE_34KM:
            return 34;
        case AS3935_L_DISTANCE_37KM:
            return 37;
        case AS3935_L_DISTANCE_40KM:
            return 40;
        case AS3935_L_DISTANCE_OO_RANGE:
            return 255;
        default:
            return 255;
    }
}

static inline void IRAM_ATTR as3935_monitor_gpio_isr_handler( void *pvParameters ) {
    as3935_monitor_context_t *as3935_monitor_context = (as3935_monitor_context_t *)pvParameters;
    xQueueSendFromISR(as3935_monitor_context->event_queue_handle, &as3935_monitor_context->irq_io_num, NULL);
}

static inline void as3935_monitor_task_entry( void *pvParameters ) {
    as3935_monitor_context_t *as3935_monitor_context = (as3935_monitor_context_t *)pvParameters;
    uint32_t io_num;

    for (;;) {
        if (xQueueReceive(as3935_monitor_context->event_queue_handle, &io_num, portMAX_DELAY)) {
            
            /* wait at least 2ms before reading the interrupt register */
            vTaskDelay(pdMS_TO_TICKS(AS3935_INTERRUPT_DELAY_MS));
            
            /* ensure i2c master bus mutex is available before reading as3935 registers */
            ENSURE_TRUE( xSemaphoreTake(as3935_monitor_context->i2c_mutex_handle, AS3935_MUTEX_WAIT_TIME) );
            
            as3935_interrupt_states_t irq_state;
            if(as3935_get_interrupt_state(as3935_monitor_context->as3935_handle, &irq_state) != 0) {
                ESP_LOGE(TAG, "as3935 device read interrupt state (register 0x03) failed");
            } else {
                if(irq_state == AS3935_INT_NOISE) {
                    /* set parent device fields to defaults */
                    as3935_monitor_context->base.lightning_distance = AS3935_L_DISTANCE_OO_RANGE;
                    as3935_monitor_context->base.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, AS3935_INT_NOISE,
                                  &(as3935_monitor_context->base), sizeof(as3935_monitor_base_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                } else if(irq_state == AS3935_INT_DISTURBER) {
                    /* set parent device fields to defaults */
                    as3935_monitor_context->base.lightning_distance = AS3935_L_DISTANCE_OO_RANGE;
                    as3935_monitor_context->base.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, AS3935_INT_DISTURBER,
                                  &(as3935_monitor_context->base), sizeof(as3935_monitor_base_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                } else if(irq_state == AS3935_INT_LIGHTNING) {
                    uint32_t lightning_energy;
                    as3935_lightning_distances_t lightning_distance;

                    if(as3935_get_lightning_event(as3935_monitor_context->as3935_handle, &lightning_distance, &lightning_energy) != 0) {
                        ESP_LOGE(TAG, "as3935 device read lightning distance and energy failed");
                    } else {
                        /* set parent device fields to defaults */
                        as3935_monitor_context->base.lightning_distance = lightning_distance;
                        as3935_monitor_context->base.lightning_energy   = lightning_energy;

                        /* send signal to notify that one unknown statement has been met */
                        esp_event_post_to(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, AS3935_INT_LIGHTNING,
                                  &(as3935_monitor_context->base), sizeof(as3935_monitor_base_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                    }
                } else if(irq_state == AS3935_INT_NONE) {
                    /* set parent device fields to defaults */
                    as3935_monitor_context->base.lightning_distance = AS3935_L_DISTANCE_OO_RANGE;
                    as3935_monitor_context->base.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, AS3935_INT_NONE,
                                  &(as3935_monitor_context->base), sizeof(as3935_monitor_base_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                } else {
                    /* set parent device fields to defaults */
                    as3935_monitor_context->base.lightning_distance = AS3935_L_DISTANCE_OO_RANGE;
                    as3935_monitor_context->base.lightning_energy   = 0;

                    /* send signal to notify that one unknown statement has been met */
                    esp_event_post_to(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, 200,
                                  &(as3935_monitor_context->base), sizeof(as3935_monitor_base_t), pdMS_TO_TICKS(AS3935_EVENT_LOOP_POST_DELAY_MS));
                }
            }
            /* ensure i2c master bus mutex is released */
            ENSURE_TRUE( xSemaphoreGive(as3935_monitor_context->i2c_mutex_handle) );
        }
        /* drive the event loop */
        esp_event_loop_run(as3935_monitor_context->event_loop_handle, pdMS_TO_TICKS(AS3935_EVENT_LOOP_POOL_DELAY_MS));
    }
    vTaskDelete( NULL );
}

esp_err_t as3935_monitor_init(i2c_master_bus_handle_t master_handle, const as3935_config_t *as3935_config, as3935_monitor_handle_t *monitor_handle) {
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE; 
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL<<as3935_config->irq_io_num);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure gpio with the given settings
    gpio_config(&io_conf);

    /* create as3935 device state object */
    as3935_monitor_context_t *as3935_monitor_context = calloc(1, sizeof(as3935_monitor_context_t));
    if (!as3935_monitor_context) {
        ESP_LOGE(TAG, "calloc memory for esp_as3935_device_t failed");
        goto err_device;
    }

    /* create i2c mutex handle */
    as3935_monitor_context->i2c_mutex_handle = xSemaphoreCreateMutex();
    if(as3935_monitor_context->i2c_mutex_handle == NULL) {
        ESP_LOGE(TAG, "create i2c mutex failed");
        goto err_emutex;
    }

    /* copy config to as3935 state object */
    as3935_monitor_context->irq_io_num = as3935_config->irq_io_num;

    /* create event loop handle */
    esp_event_loop_args_t loop_args = {
        .queue_size = AS3935_EVENT_LOOP_QUEUE_SIZE,
        .task_name = NULL
    };
    if (esp_event_loop_create(&loop_args, &as3935_monitor_context->event_loop_handle) != ESP_OK) {
        ESP_LOGE(TAG, "create event loop failed");
        goto err_eloop;
    }

    /* create a event queue to handle gpio event from isr */
    as3935_monitor_context->event_queue_handle = xQueueCreate(10, sizeof(uint32_t));
    if (!as3935_monitor_context->event_queue_handle) {
        ESP_LOGE(TAG, "create event queue handle failed");
        goto err_equeue;
    }

    /* create i2c as3935 handle */
    esp_err_t dev_err = as3935_init(master_handle, as3935_config, &as3935_monitor_context->as3935_handle);
    if(dev_err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_bus_device_create as3935 handle initialization failed %s", esp_err_to_name(dev_err));
        goto err_i2c_as3935_init;
    }

    /* create as3935 monitor task handle */
    BaseType_t err = xTaskCreatePinnedToCore( 
        as3935_monitor_task_entry, 
        AS3935_EVENT_TASK_NAME, 
        AS3935_EVENT_TASK_STACK_SIZE, 
        as3935_monitor_context, 
        AS3935_EVENT_TASK_PRIORITY,
        &as3935_monitor_context->task_monitor_handle, 
        APP_CPU_NUM );
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "create as3935 monitor task on CPU(1) failed");
        goto err_task_create;
    }
    
    
    ESP_LOGI(TAG, "as3935 device init OK");
    *monitor_handle = as3935_monitor_context;

    return ESP_OK;

    /* error handling */
    err_task_create:
        vTaskDelete(as3935_monitor_context->task_monitor_handle);
    err_i2c_as3935_init:
        as3935_remove(as3935_monitor_context->as3935_handle);
    err_equeue:
        vQueueDelete(as3935_monitor_context->event_queue_handle);
    err_eloop:
        esp_event_loop_delete(as3935_monitor_context->event_loop_handle);
    err_emutex:
        vSemaphoreDelete(as3935_monitor_context->i2c_mutex_handle);
    err_device:
        free(as3935_monitor_context);
        return ESP_ERR_INVALID_STATE;
}

esp_err_t as3935_monitor_deinit(as3935_monitor_handle_t monitor_handle) {
    as3935_monitor_context_t *as3935_monitor_context = (as3935_monitor_context_t *)monitor_handle;

    /* free-up resources */
    vTaskDelete(as3935_monitor_context->task_monitor_handle);
    esp_event_loop_delete(as3935_monitor_context->event_loop_handle);
    vQueueDelete(as3935_monitor_context->event_queue_handle);
    vSemaphoreDelete(as3935_monitor_context->i2c_mutex_handle);
    esp_err_t err = as3935_remove(as3935_monitor_context->as3935_handle);
    free(as3935_monitor_context);

    return err;
}

esp_err_t as3935_monitor_add_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler, void *handler_args) {
    as3935_monitor_context_t *as3935_monitor_context = (as3935_monitor_context_t *)monitor_handle;

    /* install as3935 monitor gpio isr service */
    gpio_install_isr_service(AS3935_IRQ_FLAG_DEFAULT);

    /* hook as3935 monitor isr handler for specific gpio pin and as3935 state object */
    gpio_isr_handler_add(as3935_monitor_context->irq_io_num, as3935_monitor_gpio_isr_handler, (void *)as3935_monitor_context);

    /* hook esp event handler for caller */
    return esp_event_handler_register_with(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, ESP_EVENT_ANY_ID,
                                           event_handler, handler_args);
}

esp_err_t as3935_monitor_remove_handler(as3935_monitor_handle_t monitor_handle, esp_event_handler_t event_handler) {
    as3935_monitor_context_t *as3935_monitor_context = (as3935_monitor_context_t *)monitor_handle;

    /* remove isr handler for gpio number */
    gpio_isr_handler_remove(as3935_monitor_context->irq_io_num);

    /* remove esp event handler from caller */
    return esp_event_handler_unregister_with(as3935_monitor_context->event_loop_handle, ESP_AS3935_EVENT, ESP_EVENT_ANY_ID, event_handler);
}

esp_err_t as3935_get_0x00_register(as3935_handle_t handle, as3935_0x00_register_t *const reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    ESP_ERROR_CHECK( as3935_i2c_read_byte_from(dev, AS3935_REG_00, &reg->reg) );

    return ESP_OK;
}

esp_err_t as3935_set_0x00_register(as3935_handle_t handle, const as3935_0x00_register_t reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    as3935_0x00_register_t reg_0x00 = { .reg = reg.reg };
    reg_0x00.bits.reserved = 0;

    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_REG_00, reg_0x00.reg) );

    return ESP_OK;
}

esp_err_t as3935_get_0x01_register(as3935_handle_t handle, as3935_0x01_register_t *const reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    ESP_ERROR_CHECK( as3935_i2c_read_byte_from(dev, AS3935_REG_01, &reg->reg) );

    return ESP_OK;
}

esp_err_t as3935_set_0x01_register(as3935_handle_t handle, const as3935_0x01_register_t reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    as3935_0x01_register_t reg_0x01 = { .reg = reg.reg };
    reg_0x01.bits.reserved = 0;

    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_REG_01, reg_0x01.reg) );

    return ESP_OK;
}

esp_err_t as3935_get_0x02_register(as3935_handle_t handle, as3935_0x02_register_t *const reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    ESP_ERROR_CHECK( as3935_i2c_read_byte_from(dev, AS3935_REG_02, &reg->reg) );

    return ESP_OK;
}

esp_err_t as3935_set_0x02_register(as3935_handle_t handle, const as3935_0x02_register_t reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    as3935_0x02_register_t reg_0x02 = { .reg = reg.reg };
    reg_0x02.bits.reserved = 0;

    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_REG_02, reg_0x02.reg) );

    return ESP_OK;
}

esp_err_t as3935_get_0x03_register(as3935_handle_t handle, as3935_0x03_register_t *const reg) {
    const uint8_t rx_retry_max  = 5;
    uint8_t rx_retry_count      = 0;
    esp_err_t ret               = ESP_OK;
    as3935_device_t* dev        = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    // retry to overcome unexpected nack
    do {
        /* attempt i2c read transaction */
        ret = as3935_i2c_read_byte_from(dev, AS3935_REG_03, &reg->reg);

        /* delay before next retry attempt */
        vTaskDelay(pdMS_TO_TICKS(1));
    } while (++rx_retry_count <= rx_retry_max && ret != ESP_OK);
    //
    /* attempt i2c read transaction */
    ESP_RETURN_ON_ERROR( ret, TAG, "unable to read to i2c device handle, get register 0x03 failed" );

    return ESP_OK;
}

esp_err_t as3935_set_0x03_register(as3935_handle_t handle, const as3935_0x03_register_t reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    as3935_0x03_register_t reg_0x03 = { .reg = reg.reg };
    reg_0x03.bits.reserved = 0;

    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_REG_03, reg_0x03.reg) );

    return ESP_OK;
}

esp_err_t as3935_get_0x08_register(as3935_handle_t handle, as3935_0x08_register_t *const reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    ESP_ERROR_CHECK( as3935_i2c_read_byte_from(dev, AS3935_REG_08, &reg->reg) );

    return ESP_OK;
}

esp_err_t as3935_set_0x08_register(as3935_handle_t handle, const as3935_0x08_register_t reg) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    as3935_0x08_register_t reg_0x08 = { .reg = reg.reg };
    reg_0x08.bits.reserved = 0;

    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_REG_08, reg_0x08.reg) );

    return ESP_OK;
}

esp_err_t as3935_setup(as3935_handle_t handle) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    /* set noise level threshold */
    ESP_ERROR_CHECK( as3935_set_noise_floor_threshold(handle, dev->config.noise_level_threshold) );

    /* set analog frontend */
    ESP_ERROR_CHECK( as3935_set_analog_frontend(handle, dev->config.analog_frontend) );

    /* set minimum number of lightning strikes */
    ESP_ERROR_CHECK( as3935_set_minimum_lightnings(handle, dev->config.min_lightning_strikes) );

    /* set disturber detection */
    if(dev->config.disturber_detection_enabled == true) {
        ESP_ERROR_CHECK( as3935_enable_disturber_detection(handle) );
    } else {
        ESP_ERROR_CHECK( as3935_disable_disturber_detection(handle) );
    }

    /* calibrate rco */
    if(dev->config.calibrate_rco == true) {
        ESP_ERROR_CHECK( as3935_calibrate_rco(handle) );
    }

    return ESP_OK;
}

esp_err_t as3935_init(i2c_master_bus_handle_t master_handle, const as3935_config_t *as3935_config, as3935_handle_t *as3935_handle) {
    /* validate arguments */
    ESP_ARG_CHECK( master_handle && as3935_config );

    /* delay task before i2c transaction */
    vTaskDelay(pdMS_TO_TICKS(AS3935_POWERUP_DELAY_MS));

    /* validate device exists on the master bus */
    esp_err_t ret = i2c_master_probe(master_handle, as3935_config->i2c_address, I2C_XFR_TIMEOUT_MS);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "device does not exist at address 0x%02x, as3935 device handle initialization failed", as3935_config->i2c_address);

    /* validate memory availability for handle */
    as3935_device_t* dev = (as3935_device_t*)calloc(1, sizeof(as3935_device_t));
    ESP_GOTO_ON_FALSE(dev, ESP_ERR_NO_MEM, err, TAG, "no memory for i2c as3935 device");

    /* copy configuration */
    dev->config = *as3935_config;

    /* set i2c device configuration */
    const i2c_device_config_t i2c_dev_conf = {
        .dev_addr_length    = I2C_ADDR_BIT_LEN_7,
        .device_address     = dev->config.i2c_address,
        .scl_speed_hz       = dev->config.i2c_clock_speed,
    };

    /* validate device handle */
    if (dev->i2c_handle == NULL) {
        ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(master_handle, &i2c_dev_conf, &dev->i2c_handle), err_handle, TAG, "i2c new bus failed");
    }

    /* set up */
    ESP_ERROR_CHECK( as3935_setup((as3935_handle_t)dev) );

    /* set device handle */
    *as3935_handle = (as3935_handle_t)dev;

    return ESP_OK;

    err_handle:
        /* clean up handle instance */
        if (dev && dev->i2c_handle) {
            i2c_master_bus_rm_device(dev->i2c_handle);
        }
        free(dev);
    err:
        return ret;
}

esp_err_t as3935_register_isr(as3935_handle_t handle, const as3935_isr_t isr) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    /* validate arguments */
    ESP_ARG_CHECK( dev );

    ESP_RETURN_ON_ERROR( gpio_isr_handler_add(dev->config.irq_io_num, ((gpio_isr_t) * (isr)), ((void *) handle)), TAG, "isr handler add failed" );
    ESP_RETURN_ON_ERROR( gpio_intr_enable(dev->config.irq_io_num), TAG, "interrupt enable failed" );

    return ESP_OK;
}

esp_err_t as3935_reset_to_defaults(as3935_handle_t handle) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_CMD_PRESET_DEFAULT, AS3935_REG_RST) );

    return ESP_OK;
}

esp_err_t as3935_calibrate_rco(as3935_handle_t handle) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    ESP_ERROR_CHECK( as3935_disable_power(handle) );
    ESP_ERROR_CHECK( as3935_i2c_write_byte_to(dev, AS3935_CMD_CALIB_RCO, AS3935_REG_RST) );

    ESP_ERROR_CHECK( as3935_set_display_oscillator_on_irq(handle, AS3935_OSCILLATOR_SYSTEM_RC, true));
    vTaskDelay(pdMS_TO_TICKS(AS3935_CALIBRATION_DELAY_MS));
    ESP_ERROR_CHECK( as3935_set_display_oscillator_on_irq(handle, AS3935_OSCILLATOR_SYSTEM_RC, false));

    return ESP_OK;
}

esp_err_t as3935_clear_lightning_statistics(as3935_handle_t handle) {
    ESP_ARG_CHECK( handle );

    //ESP_ERROR_CHECK( i2c_master_bus_write_uint8(as3935_handle->i2c_dev_handle, I2C_AS3935_REG_02, I2C_AS3935_REG_96) );

    return ESP_ERR_NOT_FINISHED;
}

esp_err_t as3935_enable_power(as3935_handle_t handle) {
    as3935_0x00_register_t reg_0x00;

    ESP_ARG_CHECK( handle );

    /* get device register 0x00 */
    ESP_ERROR_CHECK( as3935_get_0x00_register(handle, &reg_0x00) );

    /* set register 0x00[0] bit */
    reg_0x00.bits.power_state = AS3935_POWER_ON;

    /* set device register 0x00 */
    ESP_ERROR_CHECK( as3935_set_0x00_register(handle, reg_0x00) );

    return ESP_OK;
}

esp_err_t as3935_get_analog_frontend(as3935_handle_t handle, as3935_analog_frontends_t *const analog_frontend) {
    as3935_0x00_register_t reg_0x00;

    ESP_ARG_CHECK( handle );

    // reg 0x00[5:1]
    ESP_ERROR_CHECK( as3935_get_0x00_register(handle, &reg_0x00) );

    *analog_frontend = reg_0x00.bits.analog_frontend;

    return ESP_OK;
}

esp_err_t as3935_get_watchdog_threshold(as3935_handle_t handle, as3935_watchdog_thresholds_t *const watchdog_threshold) {
    as3935_0x01_register_t reg_0x01;

    ESP_ARG_CHECK( handle );

    // reg 0x01[3:0]
    ESP_ERROR_CHECK( as3935_get_0x01_register(handle, &reg_0x01) );

    *watchdog_threshold = reg_0x01.bits.watchdog_threshold;

    return ESP_OK;
}

esp_err_t as3935_get_noise_floor_threshold(as3935_handle_t handle, as3935_noise_levels_t *const noise_level) {
    as3935_0x01_register_t reg_0x01;

    ESP_ARG_CHECK( handle );

    // reg 0x01[6:4]
    ESP_ERROR_CHECK( as3935_get_0x01_register(handle, &reg_0x01) );

    *noise_level = reg_0x01.bits.noise_floor_level;

    return ESP_OK;
}

esp_err_t as3935_get_spike_rejection(as3935_handle_t handle, uint8_t *const spike_rejection) {
    as3935_0x02_register_t reg_0x02;

    ESP_ARG_CHECK( handle );

    // reg 0x02[3:0]
    ESP_ERROR_CHECK( as3935_get_0x02_register(handle, &reg_0x02) );

    *spike_rejection = reg_0x02.bits.spike_rejection & 0b1111;

    return ESP_OK;
}

esp_err_t as3935_get_minimum_lightnings(as3935_handle_t handle, as3935_minimum_lightnings_t *const min_lightnings) {
    as3935_0x02_register_t reg_0x02;

    ESP_ARG_CHECK( handle );

    // reg 0x02[5:4]
    ESP_ERROR_CHECK( as3935_get_0x02_register(handle, &reg_0x02) );

    *min_lightnings = reg_0x02.bits.min_num_lightning;

    return ESP_OK;
}

esp_err_t as3935_enable_disturber_detection(as3935_handle_t handle) {
    as3935_0x03_register_t reg_0x03;

    ESP_ARG_CHECK( handle );

    // reg 0x03[7:6]
    ESP_ERROR_CHECK( as3935_get_0x03_register(handle, &reg_0x03) );

    /* set register 0x03[5] bit */
    reg_0x03.bits.disturber_detection_state = AS3935_DISTURBER_DETECTION_ENABLED;

    /* set device register 0x03 */
    ESP_ERROR_CHECK( as3935_set_0x03_register(handle, reg_0x03) );

    return ESP_OK;

}

esp_err_t as3935_get_frequency_division_ratio(as3935_handle_t handle, as3935_frequency_division_ratios_t *const ratio) {
    as3935_0x03_register_t reg_0x03;

    ESP_ARG_CHECK( handle );

    // reg 0x03[7:6]
    ESP_ERROR_CHECK( as3935_get_0x03_register(handle, &reg_0x03) );

    *ratio = reg_0x03.bits.freq_div_ratio;

    return ESP_OK;
}

esp_err_t as3935_get_display_oscillator_on_irq(as3935_handle_t handle, const as3935_oscillator_modes_t oscillator_mode, bool *const enabled) {
    as3935_0x08_register_t reg_0x08;

    ESP_ARG_CHECK( handle );

    // reg 0x08[5]|[6]|[7]
    ESP_ERROR_CHECK( as3935_get_0x08_register(handle, &reg_0x08) );

    *enabled = false;

    switch (oscillator_mode) {
        case AS3935_OSCILLATOR_ANTENNA_LC:
            if(reg_0x08.bits.display_lco_state == AS3935_CO_IRQ_PIN_ENABLED) *enabled = true;
            break;
        case AS3935_OSCILLATOR_SYSTEM_RC:
            if(reg_0x08.bits.display_srco_state == AS3935_CO_IRQ_PIN_ENABLED) *enabled = true;
            break;
        case AS3935_OSCILLATOR_TIMER_RC:
            if(reg_0x08.bits.display_trco_state == AS3935_CO_IRQ_PIN_ENABLED) *enabled = true;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t as3935_get_internal_capacitors(as3935_handle_t handle, uint8_t *const value) {
    as3935_0x08_register_t reg_0x08;

    ESP_ARG_CHECK( handle );

    // reg 0x08[3:0]
    ESP_ERROR_CHECK( as3935_get_0x08_register(handle, &reg_0x08) );

    *value = reg_0x08.bits.tuning_capacitors & 0b1111;

    return ESP_OK;
}

esp_err_t as3935_disable_power(as3935_handle_t handle) {
    as3935_0x00_register_t reg_0x00;

    ESP_ARG_CHECK( handle );

    /* get device register 0x0- */
    ESP_ERROR_CHECK( as3935_get_0x00_register(handle, &reg_0x00) );

    /* set register 0x00[0] bit */
    reg_0x00.bits.power_state = AS3935_POWER_OFF;

    /* set device register 0x00 */
    ESP_ERROR_CHECK( as3935_set_0x00_register(handle, reg_0x00) );

    return ESP_OK;
}

esp_err_t as3935_set_analog_frontend(as3935_handle_t handle, const as3935_analog_frontends_t analog_frontend) {
    as3935_0x00_register_t reg_0x00;

    ESP_ARG_CHECK( handle );

    /* get device register 0x0- */
    ESP_ERROR_CHECK( as3935_get_0x00_register(handle, &reg_0x00) );

    /* set register 0x00[5:1] bits */
    reg_0x00.bits.analog_frontend = analog_frontend;

    /* set device register 0x00 */
    ESP_ERROR_CHECK( as3935_set_0x00_register(handle, reg_0x00) );

    return ESP_OK;
}

esp_err_t as3935_set_watchdog_threshold(as3935_handle_t handle, const as3935_watchdog_thresholds_t watchdog_threshold) {
    as3935_0x01_register_t reg_0x01;

    ESP_ARG_CHECK( handle );

    /* get device register 0x01 */
    ESP_ERROR_CHECK( as3935_get_0x01_register(handle, &reg_0x01) );

    /* set register 0x01[3:0] bits */
    reg_0x01.bits.watchdog_threshold = watchdog_threshold;

    /* set device register 0x01 */
    ESP_ERROR_CHECK( as3935_set_0x01_register(handle, reg_0x01) );

    return ESP_OK;
}

esp_err_t as3935_set_noise_floor_threshold(as3935_handle_t handle, const as3935_noise_levels_t noise_level) {
    as3935_0x01_register_t reg_0x01;

    ESP_ARG_CHECK( handle );

    /* get device register 0x01 */
    ESP_ERROR_CHECK( as3935_get_0x01_register(handle, &reg_0x01) );

    /* set register 0x01[6:4] bits */
    reg_0x01.bits.noise_floor_level = noise_level;

    /* set device register 0x01 */
    ESP_ERROR_CHECK( as3935_set_0x01_register(handle, reg_0x01) );

    return ESP_OK;
}

esp_err_t as3935_set_spike_rejection(as3935_handle_t handle, const uint8_t spike_rejection) {
    as3935_0x02_register_t reg_0x02;

    ESP_ARG_CHECK( handle );

    if (spike_rejection > 15)
        return ESP_ERR_INVALID_ARG;

    /* get device register 0x02 */
    ESP_ERROR_CHECK( as3935_get_0x02_register(handle, &reg_0x02) );

    /* set register 0x02[3:0] bits */
    reg_0x02.bits.spike_rejection &= ~0b1111;
    reg_0x02.bits.spike_rejection |= spike_rejection;

    /* set device register 0x02 */
    ESP_ERROR_CHECK( as3935_set_0x02_register(handle, reg_0x02) );

    return ESP_OK;
}

esp_err_t as3935_set_minimum_lightnings(as3935_handle_t handle, const as3935_minimum_lightnings_t min_lightnings) {
    as3935_0x02_register_t reg_0x02;

    ESP_ARG_CHECK( handle );

    /* get device register 0x02 */
    ESP_ERROR_CHECK( as3935_get_0x02_register(handle, &reg_0x02) );

    /* set register 0x02[5:4] bits */
    reg_0x02.bits.min_num_lightning = min_lightnings;

    /* set device register 0x02 */
    ESP_ERROR_CHECK( as3935_set_0x02_register(handle, reg_0x02) );

    return ESP_OK;
}

esp_err_t as3935_disable_disturber_detection(as3935_handle_t handle) {
    as3935_0x03_register_t reg_0x03;

    ESP_ARG_CHECK( handle );

    /* get device register 0x03 */
    ESP_ERROR_CHECK( as3935_get_0x03_register(handle, &reg_0x03) );

    /* set register 0x03[5] bit */
    reg_0x03.bits.disturber_detection_state = AS3935_DISTURBER_DETECTION_DISABLED;

    /* set device register 0x03 */
    ESP_ERROR_CHECK( as3935_set_0x03_register(handle, reg_0x03) );

    return ESP_OK;
}

esp_err_t as3935_set_frequency_division_ratio(as3935_handle_t handle, const as3935_frequency_division_ratios_t ratio) {
    as3935_0x03_register_t reg_0x03;

    ESP_ARG_CHECK( handle );

    /* get device register 0x03 */
    ESP_ERROR_CHECK( as3935_get_0x03_register(handle, &reg_0x03) );

    /* set register 0x03[7:6] bits */
    reg_0x03.bits.freq_div_ratio = ratio;

    /* set device register 0x03 */
    ESP_ERROR_CHECK( as3935_set_0x03_register(handle, reg_0x03) );

    return ESP_OK;
}

esp_err_t as3935_set_display_oscillator_on_irq(as3935_handle_t handle, const as3935_oscillator_modes_t oscillator_mode, const bool enabled) {
    as3935_0x08_register_t reg_0x08;

    ESP_ARG_CHECK( handle );

    /* get device register 0x08 */
    ESP_ERROR_CHECK( as3935_get_0x08_register(handle, &reg_0x08) );

    /* set register 0x08[5]|[6]|[7] bits */
    switch (oscillator_mode) {
        case AS3935_OSCILLATOR_ANTENNA_LC:
            if(enabled == true) reg_0x08.bits.display_lco_state = AS3935_CO_IRQ_PIN_ENABLED;
            break;
        case AS3935_OSCILLATOR_SYSTEM_RC:
            if(enabled == true) reg_0x08.bits.display_srco_state = AS3935_CO_IRQ_PIN_ENABLED;
            break;
        case AS3935_OSCILLATOR_TIMER_RC:
            if(enabled == true) reg_0x08.bits.display_trco_state = AS3935_CO_IRQ_PIN_ENABLED;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }

    /* set device register 0x08 */
    ESP_ERROR_CHECK( as3935_set_0x08_register(handle, reg_0x08) );

    return ESP_OK;
}

esp_err_t as3935_set_internal_capacitors(as3935_handle_t handle, const uint8_t value) {
    as3935_0x08_register_t reg_0x08;

    ESP_ARG_CHECK( handle );

    if (value > 15)
        return ESP_ERR_INVALID_ARG;

    /* get device register 0x08 */
    ESP_ERROR_CHECK( as3935_get_0x08_register(handle, &reg_0x08) );

    /* set register 0x08[3:0] bits */
    reg_0x08.bits.tuning_capacitors &= ~0b1111;
    reg_0x08.bits.tuning_capacitors |= value;

    /* set device register 0x08 */
    ESP_ERROR_CHECK( as3935_set_0x08_register(handle, reg_0x08) );

    return ESP_OK;
}

esp_err_t as3935_get_interrupt_state(as3935_handle_t handle, as3935_interrupt_states_t *const state) {
    as3935_0x03_register_t reg_0x03;

    ESP_ARG_CHECK( handle );

    ESP_ERROR_CHECK( as3935_get_0x03_register(handle, &reg_0x03) );

    *state = reg_0x03.bits.irq_state;

    return ESP_OK;
}

esp_err_t as3935_get_lightning_distance(as3935_handle_t handle, as3935_lightning_distances_t *const distance) {
    as3935_0x07_register_t reg_0x07;

    ESP_ARG_CHECK( handle );

    ESP_ERROR_CHECK( as3935_i2c_read_byte_from(handle, AS3935_REG_07, &reg_0x07.reg) );

    *distance = reg_0x07.bits.lightning_distance;

    return ESP_OK;
}

esp_err_t as3935_get_lightning_distance_km(as3935_handle_t handle, uint8_t *const distance) {
    as3935_lightning_distances_t val;

    ESP_ERROR_CHECK( as3935_get_lightning_distance(handle, &val) );

    *distance = as3935_convert_distance_km(val);

    return ESP_OK;
}

esp_err_t as3935_get_lightning_energy(as3935_handle_t handle, uint32_t *const energy) {
    bit24_uint8_buffer_t data;

    ESP_ARG_CHECK( handle );

    ESP_ERROR_CHECK( as3935_i2c_read_from(handle, AS3935_REG_04, data, BIT24_UINT8_BUFFER_SIZE) );

    data[2] &= 0b11111;

    *energy = 0;
    
    for (size_t i = 0; i < 3; ++i)
        *energy |= data[i] << (i * 8);

    return ESP_OK;
}

esp_err_t as3935_get_lightning_event(as3935_handle_t handle, as3935_lightning_distances_t *const distance, uint32_t *const energy) {
    ESP_ARG_CHECK( handle );

    ESP_ERROR_CHECK( as3935_get_lightning_distance(handle, distance) );

    ESP_ERROR_CHECK( as3935_get_lightning_energy(handle, energy) );

    return ESP_OK;
}

esp_err_t as3935_remove(as3935_handle_t handle) {
    as3935_device_t* dev = (as3935_device_t*)handle;

    ESP_ARG_CHECK( dev );

    return i2c_master_bus_rm_device(dev->i2c_handle);
}

esp_err_t as3935_delete(as3935_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* remove device from master bus */
    ESP_RETURN_ON_ERROR( as3935_remove(handle), TAG, "unable to remove device from i2c master bus, delete handle failed" );

    /* validate handle instance and free handles */
    if(handle) {
        free(handle);
    }

    return ESP_OK;
}

const char* as3935_get_fw_version(void) {
    return (const char*)AS3935_FW_VERSION_STR;
}

int32_t as3935_get_fw_version_number(void) {
    return (int32_t)AS3935_FW_VERSION_INT32;
}