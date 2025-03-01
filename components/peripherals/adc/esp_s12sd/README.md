<a href="https://components.espressif.com/components/k0i05/esp_s12d">
<img src="https://components.espressif.com/components/k0i05/esp_s12d/badge.svg" />
</a>

# Roithner LaserTechnik GUVA-S12SD Sensor
This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the Roithner LaserTechnik GUVA-S12SD ultraviolet analog sensor.  Information on features and functionality are documented and can be found in the `s12sd.h` header file and in the `documentation` folder.

## Repository
The component is hosted on github and is located here: https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/adc/esp_s12sd

## General Usage
To get started, simply copy the component to your project's `components` folder and reference the `s12sd.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```
components
└── esp_s12sd
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── s12sd_version.h
    │   └── s12sd.h
    └── s12sd.c
```

## Basic Example
Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```
#include <s12sd.h>

static void adc_0_task( void *pvParameters ) {
    TickType_t                         xLastWakeTime;
    //
    // initialize the xLastWakeTime variable with the current time.
    xLastWakeTime                      = xTaskGetTickCount ();
    //
    //
    // initialize s12sd device configuration
    s12sd_config_t                s12sd_dev_cfg = ADC_S12SD_CONFIG_DEFAULT;
    s12sd_handle_t                s12sd_dev_hdl;
    //
    // s12sd init device
    s12sd_init(&s12sd_dev_cfg, &s12sd_dev_hdl);
    if (s12sd_dev_hdl == NULL) ESP_LOGE(CONFIG_APP_TAG, "[APP] adc0 s12sd handle init failed");
    //
    //
    // task loop entry point
    for ( ;; ) {
        // read s12sd uv sensor
        uint8_t uv_index;
        s12sd_measure(s12sd_dev_hdl, &uv_index);
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }
    //
    // free up task resources and remove task from stack
    s12sd_delete(s12sd_dev_hdl);
    vTaskDelete( NULL );
}
```



Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)