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

/*
    █ ▄▀ █▀█ ▀█▀ █▀█ █▀▀
    █▀▄  █ █  █  █ █ ▀▀█
    ▀  ▀ ▀▀▀ ▀▀▀ ▀▀▀ ▀▀▀
*/

/*
    █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▄ █▀▀ █▀▀ ▀█▀ █▀█
    █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▄ █▀▀ █ █  █  █ █
    ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀  ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀
*/

/*
    █ █ █▀▀ █▀▀ █▀▄   █▀▀ █▀█ █▀▄ █▀▀   █▀▀ █▀█ █▀▄
    █ █ ▀▀█ █▀▀ █▀▄   █   █ █ █ █ █▀▀   █▀▀ █ █ █ █
    ▀▀▀ ▀▀▀ ▀▀▀ ▀ ▀   ▀▀▀ ▀▀▀ ▀▀  ▀▀▀   ▀▀▀ ▀ ▀ ▀▀ 
*/

/**
 * @file main.c
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE

 * @brief ESP-IDF consolidated component repository with basic examples for each component.  Components were developed
 * with an ESP32-S3 development board under Visual Studio Code and Platform IO and may not work with older development
 * board releases.
 * 
 * See `app_config.h` within the include folder to change default I2C master bus and I2C GPIO ping configuration settings.
 * 
 * See `[component-name]_task.h` within the include folder to change default configuration settings by component.
 *
 * https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
 * 
 * 
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * C:\Users\lavco\.platformio\penv\Scripts\platformio.exe system prune
 * 
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <string.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* components */
//#include <i2c_master_ext.h>
#include <nvs_ext.h>

/* i2c component tasks */
#include <ahtxx_task.h>
#include <ak8975_task.h>
#include <as7341_task.h>
#include <at24cxxx_task.h>
#include <as3935_task.h>
#include <bh1750_task.h>
#include <bme680_task.h>
#include <bmp280_task.h>
#include <bmp390_task.h>
#include <ccs811_task.h>
#include <ens160_task.h>
#include <hdc1080_task.h>
#include <hmc5883l_task.h>
#include <ina226_task.h>
#include <ina228_task.h>
#include <ltr390uv_task.h>
#include <max30105_task.h>
#include <mlx90614_task.h>
#include <mmc56x3_task.h>
#include <mpu6050_task.h>
#include <pct2075_task.h>
#include <sgp4x_task.h>
#include <sht4x_task.h>
#include <ssd1306_task.h>
#include <tcs3472_task.h>
#include <tlv493d_task.h>
#include <veml6040_task.h>
#include <veml7700_task.h>

/* owb component tasks */
#include <ds18b20_task.h>

/* spi component tasks */

/* schedule tasks */
#include <time_into_interval_task.h>

/* utilities tasks */
#include <uuid_task.h>


/**
 * @brief I2C components examples enumerator.
 */
typedef enum i2c_components_tag {
    I2C_COMPONENT_AHTXX,
    I2C_COMPONENT_AS7341,
    I2C_COMPONENT_AS3935,
    I2C_COMPONENT_AT24CXXX,
    I2C_COMPONENT_AK8975,
    I2C_COMPONENT_BH1750,
    I2C_COMPONENT_BME680,
    I2C_COMPONENT_BMP280,
    I2C_COMPONENT_BMP390,
    I2C_COMPONENT_CCS811,
    I2C_COMPONENT_ENS160,
    I2C_COMPONENT_HDC1080,
    I2C_COMPONENT_HMC5883L,
    I2C_COMPONENT_INA226,
    I2C_COMPONENT_INA228,
    I2C_COMPONENT_LTR390UV,
    I2C_COMPONENT_MAX30105,
    I2C_COMPONENT_MLX90614,
    I2C_COMPONENT_MMC56X3,
    I2C_COMPONENT_MPU6050,
    I2C_COMPONENT_PCT2075,
    I2C_COMPONENT_SGP4X,
    I2C_COMPONENT_SHT4X,
    I2C_COMPONENT_SSD1306,
    I2C_COMPONENT_TCS3472,
    I2C_COMPONENT_TLV493D,
    I2C_COMPONENT_VEML6040,
    I2C_COMPONENT_VEML7700,
} i2c_components_t;

/**
 * @brief OWB components examples enumerator.
 */
typedef enum owb_components_tag {
    OWB_COMPONENT_DS18B20,
} owb_components_t;

/**
 * @brief SPI components examples enumerator.
 */
typedef enum spi_components_tag {
    SPI_COMPONENT_MAX31865,
} spi_components_t;

/**
 * @brief Utilities components examples enumerator.
 */
typedef enum utils_components_tag {
    UTILS_COMPONENT_UUID,
} utils_components_t;

/**
 * @brief Schedule components examples enumerator.
 */
typedef enum sch_components_tag {
    SCH_COMPONENT_TIME_INTO_INTERVAL,
} sch_components_t;

// initialize master i2c 0 bus configuration
i2c_master_bus_config_t  i2c0_bus_cfg = I2C0_MASTER_CONFIG_DEFAULT;
i2c_master_bus_handle_t  i2c0_bus_hdl;
bool                     i2c0_component_tasked = false;

// initialize master owb 0 bus configuration
onewire_bus_rmt_config_t owb0_rmt_cfg = OW0_RMT_CONFIG_DEFAULT;
onewire_bus_config_t     owb0_bus_cfg = OW0_MASTER_CONFIG_DEFAULT;
onewire_bus_handle_t     owb0_bus_hdl;
bool                     owb0_component_tasked = false;

// initialize master spi 1 bus configuration
spi_bus_config_t         spi1_bus_cfg = SPI1_MASTER_CONFIG_DEFAULT;
spi_device_handle_t      spi1_dev_hdl;
bool                     spi1_component_tasked = false;

bool                     sch_component_tasked = false;
bool                     utils_component_tasked = false;

esp_err_t i2c_master_bus_detect_devices(i2c_master_bus_handle_t handle) {
    const uint16_t probe_timeout_ms = 50; // timeout in milliseconds
    uint8_t address;

    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");

    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);

        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            
            address = i + j;

            esp_err_t ret = i2c_master_probe(handle, address, probe_timeout_ms);

            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    return ESP_OK; 
}

/**
 * @brief Creates a task pinned to the application core (1) by task function
 * and task name to run an schedule component example.
 * 
 * @param task Task function reference.
 * @param name Task reference name.
 */
static inline void sch_task_create(TaskFunction_t task, const char* name) {
    /*  
        note: only one task can run at a time
    */

    /* validate utilities component flag */
    if(sch_component_tasked == true) {
        ESP_LOGE(APP_TAG, "A schedule component sample is already running, failed to create schedule task");
        return;
    }
    /* create task pinned to the app core */
    xTaskCreatePinnedToCore( 
        task,
        name, 
        SCH_TASK_STACK_SIZE, 
        NULL, 
        SCH_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    /* set schedule component flag */
    sch_component_tasked = true;
}

/**
 * @brief Creates a task pinned to the application core (1) by task function
 * and task name to run an utilities component example.
 * 
 * @param task Task function reference.
 * @param name Task reference name.
 */
static inline void utils_task_create(TaskFunction_t task, const char* name) {
    /*  
        note: only one task can run at a time
    */

    /* validate utilities component flag */
    if(utils_component_tasked == true) {
        ESP_LOGE(APP_TAG, "A utilities component sample is already running, failed to create utilities task");
        return;
    }
    /* create task pinned to the app core */
    xTaskCreatePinnedToCore( 
        task,
        name, 
        UTILS_TASK_STACK_SIZE, 
        NULL, 
        UTILS_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    /* set utilities component flag */
    utils_component_tasked = true;
}

/**
 * @brief Creates a task pinned to the application core (1) by task function
 * and task name to run an OWB component example on one wire master bus (0).
 * 
 * @note Only one OWB component example can run on one wire master bus 0.
 * 
 * @param task Task function reference.
 * @param name Task reference name.
 */
static inline void owb0_task_create(TaskFunction_t task, const char* name) {
    /*  
        note: only one task on the one wire master bus 0 can run at a time
    */

    /* validate owb0 component flag */
    if(owb0_component_tasked == true) {
        ESP_LOGE(APP_TAG, "An OWB component sample is already running on one wire master bus 0, failed to create owb0 task");
        return;
    }
    /* create task pinned to the app core */
    xTaskCreatePinnedToCore( 
        task,
        name, 
        OWB0_TASK_STACK_SIZE, 
        NULL, 
        OWB0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    /* set owb0 component flag */
    owb0_component_tasked = true;
}

/**
 * @brief Creates a task pinned to the application core (1) by task function
 * and task name to run an I2C component example on I2C master bus (0).
 * 
 * @note Only one I2C component example can run on I2C master bus 0.
 * 
 * @param task Task function reference.
 * @param name Task reference name.
 */
static inline void i2c0_task_create(TaskFunction_t task, const char* name) {
    /*  
        note: only one task on the i2c master bus 0 can run at a time
    */

    /* validate i2c0 component flag */
    if(i2c0_component_tasked == true) {
        ESP_LOGE(APP_TAG, "An I2C component sample is already running on I2C master bus 0, failed to create i2c0 task");
        return;
    }
    /* create task pinned to the app core */
    xTaskCreatePinnedToCore( 
        task,
        name, 
        I2C0_TASK_STACK_SIZE, 
        NULL, 
        I2C0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
    /* set i2c0 component flag */
    i2c0_component_tasked = true;
}

/**
 * @brief SCH example to run in the application by component name.
 * 
 * @param component SCH component example to run in the application
 */
static inline void sch_component_example_start(const sch_components_t component) {
    /*  
        device task functions use a common naming nomenclature
        for owb, i2c and adc peripherals, as an example: utils_[device]_task

        note: only one utilities 0 task can run at a time
     */
    switch(component) {
        case SCH_COMPONENT_TIME_INTO_INTERVAL:
            sch_task_create(sch_time_into_interval_task, SCH_TIME_INTO_INTERVAL_TASK_NAME);
            break;
    }
}

/**
 * @brief UTILS example to run in the application by component name.
 * 
 * @param component UTILS component example to run in the application
 */
static inline void utils_component_example_start(const utils_components_t component) {
    /*  
        device task functions use a common naming nomenclature
        for owb, i2c and adc peripherals, as an example: utils_[device]_task

        note: only one utilities 0 task can run at a time
     */
    switch(component) {
        case UTILS_COMPONENT_UUID:
            utils_task_create(utils_uuid_task, UTILS_UUID_TASK_NAME);
            break;
    }
}

/**
 * @brief OWB example to run in the application by component name.
 * 
 * @note Only one OWB component example can run on one wire master bus 0.
 * 
 * @param component OWB component example to run in the application
 */
static inline void owb0_component_example_start(const owb_components_t component) {
    /*  
        device task functions use a common naming nomenclature
        for owb, i2c and adc peripherals, as an example: owb0_[device]_task

        note: only one owb 0 task can run at a time
     */
    switch(component) {
        case OWB_COMPONENT_DS18B20:
            owb0_task_create(owb0_ds18b20_task, OWB0_DS18B20_TASK_NAME);
            break;
    }
}

/**
 * @brief I2C example to run in the application by component name.
 * 
 * @note Only one I2C component example can run on I2C master bus 0.
 * 
 * @param component I2C component example to run in the application
 */
static inline void i2c0_component_example_start(const i2c_components_t component) {
    /*  
        device task functions use a common naming nomenclature
        for i2c and adc peripherals, as an example: i2c_0_[device]_task

        note: only one i2c 0 task can run at a time
     */
    switch(component) {
        case I2C_COMPONENT_AHTXX:
            /* starts ahtxx task */
            i2c0_task_create(i2c0_ahtxx_task, I2C0_AHTXX_TASK_NAME);
            break;
        case I2C_COMPONENT_AK8975:
            /* starts ak8975 task */
            i2c0_task_create(i2c0_ak8975_task, I2C0_AK8975_TASK_NAME);
            break;
        case I2C_COMPONENT_AS7341:
            /* starts as7341 task */
            i2c0_task_create(i2c0_as7341_task, I2C0_AS7341_TASK_NAME);
            break;
        case I2C_COMPONENT_AT24CXXX:
            /* starts at24cxxx task */
            i2c0_task_create(i2c0_at24cxxx_task, I2C0_AT24CXXX_TASK_NAME);
            break;
        case I2C_COMPONENT_AS3935:
            /* starts as3935 task */
            i2c0_task_create(i2c0_as3935_task, I2C0_AS3935_TASK_NAME);
            break;
        case I2C_COMPONENT_BH1750:
            /* starts bh1750 task */
            i2c0_task_create(i2c0_bh1750_task, I2C0_BH1750_TASK_NAME);
            break;
        case I2C_COMPONENT_BME680:
            /* starts bme680 task */
            i2c0_task_create(i2c0_bme680_task, I2C0_BME680_TASK_NAME);
            break;
        case I2C_COMPONENT_BMP280:
            /* starts bmp280 task */
            i2c0_task_create(i2c0_bmp280_task, I2C0_BMP280_TASK_NAME);
            break;
        case I2C_COMPONENT_BMP390:
            /* starts bmp390 task */
            i2c0_task_create(i2c0_bmp390_task, I2C0_BMP390_TASK_NAME);
            break;
        case I2C_COMPONENT_CCS811:
            /* starts ccs811 task */
            i2c0_task_create(i2c0_ccs811_task, I2C0_CCS811_TASK_NAME);
            break;
        case I2C_COMPONENT_ENS160:
            /* starts ens160 task */
            i2c0_task_create(i2c0_ens160_task, I2C0_ENS160_TASK_NAME);
            break;
        case I2C_COMPONENT_HDC1080:
            /* starts hdc1080 task */
            i2c0_task_create(i2c0_hdc1080_task, I2C0_HDC1080_TASK_NAME);
            break;
        case I2C_COMPONENT_HMC5883L:
            /* starts hmc5883l task */
            i2c0_task_create(i2c0_hmc5883l_task, I2C0_HMC5883L_TASK_NAME);
            break;
        case I2C_COMPONENT_INA226:
            /* starts ina226 task */
            i2c0_task_create(i2c0_ina226_task, I2C0_INA226_TASK_NAME);
            break;
        case I2C_COMPONENT_INA228:
            /* starts ina228 task */
            i2c0_task_create(i2c0_ina228_task, I2C0_INA228_TASK_NAME);
            break;
        case I2C_COMPONENT_LTR390UV:
            /* starts ltr390uv task */
            i2c0_task_create(i2c0_ltr390uv_task, I2C0_LTR390UV_TASK_NAME);
            break;
        case I2C_COMPONENT_MAX30105:
            /* starts max30105 task */
            i2c0_task_create(i2c0_max30105_task, I2C0_MAX30105_TASK_NAME);
            break;
        case I2C_COMPONENT_MLX90614:
            /* starts mlx90614 task */
            i2c0_task_create(i2c0_mlx90614_task, I2C0_MLX90614_TASK_NAME);
            break;
        case I2C_COMPONENT_MMC56X3:
            /* starts mmc56x3 task */
            i2c0_task_create(i2c0_mmc56x3_task, I2C0_MMC56X3_TASK_NAME);
            break;
        case I2C_COMPONENT_MPU6050:
            /* starts mpu6050 task */
            i2c0_task_create(i2c0_mpu6050_task, I2C0_MPU6050_TASK_NAME);
            break;
        case I2C_COMPONENT_PCT2075:
            /* starts pct2075 task */
            i2c0_task_create(i2c0_pct2075_task, I2C0_PCT2075_TASK_NAME);
            break;
        case I2C_COMPONENT_SGP4X:
            /* starts sgp4x task */
            i2c0_task_create(i2c0_sgp4x_task, I2C0_SGP4X_TASK_NAME);
            break;
        case I2C_COMPONENT_SHT4X:
            /* starts sht4x task */
            i2c0_task_create(i2c0_sht4x_task, I2C0_SHT4X_TASK_NAME);
            break;
        case I2C_COMPONENT_SSD1306:
            /* starts ssd1306 task */
            i2c0_task_create(i2c0_ssd1306_task, I2C0_SSD1306_TASK_NAME);
            break;
        case I2C_COMPONENT_TCS3472:
            /* starts tcs3472 task */
            i2c0_task_create(i2c0_tcs3472_task, I2C0_TCS3472_TASK_NAME);
            break;
        case I2C_COMPONENT_TLV493D:
            /* starts tlv493d task */
            i2c0_task_create(i2c0_tlv493d_task, I2C0_TLV493D_TASK_NAME);
            break;
        case I2C_COMPONENT_VEML6040:
            /* starts veml6040 task */
            i2c0_task_create(i2c0_veml6040_task, I2C0_VEML6040_TASK_NAME);
            break;
        case I2C_COMPONENT_VEML7700:
            /* starts veml7700 task */
            i2c0_task_create(i2c0_veml7700_task, I2C0_VEML7700_TASK_NAME);
            break;
    }
}


/**
 * @brief Scans I2C master bus 0 for i2c devices and prints each device address when detected.
 */
static inline esp_err_t i2c0_device_scan(void) {
    ESP_LOGI(APP_TAG, "Scanning I2C master bus 0 for I2C devices..");
    return i2c_master_bus_detect_devices(i2c0_bus_hdl);
}

/**
 * @brief Main application entry point.
 */
void app_main( void ) {
    ESP_LOGI(APP_TAG, "Startup..");
    ESP_LOGI(APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(APP_TAG, "IDF version: %s", esp_get_idf_version());

    /* set log levels */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(APP_TAG, ESP_LOG_VERBOSE);

    /* instantiate nvs storage */
    ESP_ERROR_CHECK( nvs_init() );

    /* instantiate one wire master bus 0 */
    ESP_ERROR_CHECK( onewire_new_bus_rmt(&owb0_bus_cfg, &owb0_rmt_cfg, &owb0_bus_hdl) );

    /* instantiate i2c master bus 0 */
    ESP_ERROR_CHECK( i2c_new_master_bus(&i2c0_bus_cfg, &i2c0_bus_hdl) );

    /* scan i2c devices on i2c master bus 0 and print results */
    //ESP_ERROR_CHECK( i2c0_device_scan() );


    /* delay task before starting component example */
    vTaskDelay( pdMS_TO_TICKS(500) );

    /* start a component example */
    /* dotnet-gitversion */
    /* note: only one component example can run at a time */
    
    //i2c0_component_example_start(I2C_COMPONENT_AHTXX);
    //i2c0_component_example_start(I2C_COMPONENT_AK8975);
    //i2c0_component_example_start(I2C_COMPONENT_AS7341);
    //i2c0_component_example_start(I2C_COMPONENT_AT24CXXX);
    //i2c0_component_example_start(I2C_COMPONENT_AS3935);
    //i2c0_component_example_start(I2C_COMPONENT_BH1750);
    //i2c0_component_example_start(I2C_COMPONENT_BME680);
    //i2c0_component_example_start(I2C_COMPONENT_BMP280);
    //i2c0_component_example_start(I2C_COMPONENT_BMP390);
    //i2c0_component_example_start(I2C_COMPONENT_CCS811);
    //i2c0_component_example_start(I2C_COMPONENT_ENS160);
    //i2c0_component_example_start(I2C_COMPONENT_HDC1080);
    //i2c0_component_example_start(I2C_COMPONENT_HMC5883L);
    //i2c0_component_example_start(I2C_COMPONENT_INA226);
    //i2c0_component_example_start(I2C_COMPONENT_INA228);
    //i2c0_component_example_start(I2C_COMPONENT_LTR390UV);
    i2c0_component_example_start(I2C_COMPONENT_MAX30105);
    //i2c0_component_example_start(I2C_COMPONENT_MLX90614);
    //i2c0_component_example_start(I2C_COMPONENT_MMC56X3);
    //i2c0_component_example_start(I2C_COMPONENT_MPU6050);
    //i2c0_component_example_start(I2C_COMPONENT_PCT2075);
    //i2c0_component_example_start(I2C_COMPONENT_SGP4X);
    //i2c0_component_example_start(I2C_COMPONENT_SHT4X);
    //i2c0_component_example_start(I2C_COMPONENT_SSD1306);
    //i2c0_component_example_start(I2C_COMPONENT_TCS3472);
    //i2c0_component_example_start(I2C_COMPONENT_TLV493D);
    //i2c0_component_example_start(I2C_COMPONENT_VEML6040);
    //i2c0_component_example_start(I2C_COMPONENT_VEML7700);

    //owb0_component_example_start(OWB_COMPONENT_DS18B20);

    //sch_component_example_start(SCH_COMPONENT_TIME_INTO_INTERVAL);

    //utils_component_example_start(UTILS_COMPONENT_UUID);
}