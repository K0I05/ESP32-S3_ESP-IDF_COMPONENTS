<a href="https://components.espressif.com/components/k0i05/esp_bmp280">
<img src="https://components.espressif.com/components/k0i05/esp_bmp280/badge.svg" />
</a>

# Bosch BMP280 Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Bosch BMP280 sensor.  Information on features and functionality are documented and can be found in the `bmp280.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_bmp280

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `bmp280.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_bmp280
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── bmp280_version.h
    │   └── bmp280.h
    └── bmp280.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <bmp280.h>

void i2c0_bmp280_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time  = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    bmp280_config_t dev_cfg         = I2C_BMP280_CONFIG_DEFAULT;
    bmp280_handle_t dev_hdl;
    //
    // init device
    bmp280_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "bmp280 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## BMP280 - START #########################");
        //
        // handle sensor
        float temperature, pressure;
        esp_err_t result = bmp280_get_measurements(dev_hdl, &temperature, &pressure);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "bmp280 device read failed (%s)", esp_err_to_name(result));
        } else {
            pressure = pressure / 100;
            ESP_LOGI(APP_TAG, "air temperature:     %.2f °C", temperature);
            ESP_LOGI(APP_TAG, "barometric pressure: %.2f hPa", pressure);
        }
        //
        ESP_LOGI(APP_TAG, "######################## BMP280 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    bmp280_delete( dev_hdl );
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)