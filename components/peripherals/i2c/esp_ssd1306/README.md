# Generic SSD1306 (128x32 | 128x64 | 128x128) OLED Display

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-007ACC?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAxMDAgMTAwIj48ZGVmcz48bGluZWFyR3JhZGllbnQgaWQ9ImEiIHgxPSI1MCIgeTE9IjAiIHgyPSI1MCIgeTI9IjEwMCIgZ3JhZGllbnRVbml0cz0idXNlclNwYWNlT25Vc2UiPjxzdG9wIHN0b3AtY29sb3I9IiNmZmYiLz48c3RvcCBvZmZzZXQ9IjEiIHN0b3AtY29sb3I9IiNmZmYiIHN0b3Atb3BhY2l0eT0iMCIvPjwvbGluZWFyR3JhZGllbnQ+PC9kZWZzPjxwYXRoIGZpbGw9IiMwMDY1QTkiIGQ9Ik05Ni40NiAxMC44IDc1Ljg2Ljg4QzczLjQ3LS4yNyA3MC42Mi4yMSA2OC43NSAyLjA4TDEuMyA2My41OGMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZsNS41MSA1LjAxYzEuNDkgMS4zNSAzLjcyIDEuNDUgNS4zMi4yNGw4MS4yMy02MS42MmMyLjcyLTIuMDcgNi42NC0uMTMgNi42NCAzLjI5di0uMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMDA3QUNDIiBkPSJNOTYuNDYgODkuMiA3NS44NiA5OS4xMmMtMi4zOSAxLjE1LTUuMjQuNjYtNy4xMS0xLjJMMS4zIDM2LjQyYy0xLjgyLTEuNjUtMS44Mi00LjUxIDAtNi4xN2w1LjUxLTUuMDFjMS40OS0xLjM1IDMuNzItMS40NSA1LjMyLS4yM2w4MS4yMyA2MS42MmMyLjcyIDIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMCAyLjQtMS4zOCA0LjU5LTMuNTQgNS42M1oiLz48cGF0aCBmaWxsPSIjMUY5Q0YwIiBkPSJNNzUuODYgOTkuMTNjLTIuMzkgMS4xNS01LjI0LjY2LTcuMTEtMS4yMSAyLjMxIDIuMzEgNi4yNS42NiA2LjI1LTIuNlY0LjY3YzAtMy4yNi0zLjk0LTQuODktNi4yNS0yLjU4IDEuODctMS44NyA0LjcyLTIuMzYgNy4xMS0xLjIxbDIwLjYgOS45MWMyLjE2IDEuMDQgMy41NCAzLjIzIDMuNTQgNS42M3Y2Ny4xN2MwIDIuNC0xLjM4IDQuNTktMy41NCA1LjYzbC0yMC42IDkuOTFaIi8+PHBhdGggZmlsbD0idXJsKCNhKSIgZmlsbC1ydWxlPSJldmVub2RkIiBkPSJNNzAuODUgOTkuMzJjMS41OC42MSAzLjM3LjU3IDQuOTYtLjE5bDIwLjU5LTkuOTFjMi4xNi0xLjA0IDMuNTQtMy4yMyAzLjU0LTUuNjNWMTYuNDFjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M0w3NS44MS44N2MtMi4wOS0xLTQuNTMtLjc2LTYuMzYuNTgtLjI2LjE5LS41MS40LS43NC42NEwyOS4yOSAzOC4wNGwtMTcuMTctMTMuMDNjLTEuNi0xLjIxLTMuODMtMS4xMS01LjMyLjI0bC01LjUgNS4wMWMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZMMTYuMTkgNTAgMS4zIDYzLjU4Yy0xLjgyIDEuNjUtMS44MiA0LjUxIDAgNi4xN2w1LjUxIDUuMDFjMS40OSAxLjM1IDMuNzIgMS40NSA1LjMyLjIzbDE3LjE3LTEzLjAzIDM5LjQxIDM1Ljk2Yy42Mi42MiAxLjM2IDEuMDkgMi4xNCAxLjRaTTc0Ljk1IDI3LjMgNDUuMDUgNTBsMjkuOSAyMi43VjI3LjNaIiBvcGFjaXR5PSIuMjUiIHN0eWxlPSJtaXgtYmxlbmQtbW9kZTpvdmVybGF5Ii8+PC9zdmc+&logoColor=white)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_ssd1306.svg)](https://registry.platformio.org/libraries/k0i05/esp_ssd1306)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_ssd1306/badge.svg)](https://components.espressif.com/components/k0i05/esp_ssd1306)

This esp-idf driver was developed for generic SSD1306 OLED displays.  Information on features and functionality are documented and can be found in the `ssd1306.h` header file.  The SSD1306 component is a compact and simplified driver compatible with 128x64, 128x32, 128x128 OLED displays.  There are three font sizes supported, hardware and software scrolling capabilities, bitmap visualization, and more.  This component has one font implemented now (i.e. 8x8 basic Latin + control + extended Latin) but is ideal for most use cases.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_ssd1306>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `ssd1306.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_ssd1306
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ssd1306_version.h
    │   └── ssd1306.h
    └── ssd1306.c
```

## Basic Example

Once a driver instance is instantiated the display panel is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and displays a sequence of text messages and bitmaps at user defined interval and prints the results.

The example initializes a 128x64 SSD1306 OLED display and demonstrates the following features:

- Display large text (x3)
- Display file receive and transmit bitmap icons
- Display medium text (x2)
- Display text
- Display countdown timer
- Display text scrolling up
- Display text scrolling down
- Display text paging down and up
- Display text scrolling horizontally from right and left
- Display text scrolling vertically downwards and upwards
- Display bitmap images
- Display inverted text and fadeout

```c
#include <ssd1306.h>


void i2c0_ssd1306_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t          last_wake_time   = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    ssd1306_config_t dev_cfg         = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    ssd1306_handle_t dev_hdl;
    //
    // init device
    ssd1306_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "ssd1306 handle init failed");
        assert(dev_hdl);
    }
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SSD1306 - START #########################");
        //
        int center = 1, top = 1, bottom = 4;
        char lineChar[16];
        uint8_t image[24];

        ESP_LOGI(APP_TAG, "Panel is 128x64");

        // Display x3 text
        ESP_LOGI(APP_TAG, "Display x3 Text");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text_x3(dev_hdl, 0, "Hello", false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Display bitmap icons
        ESP_LOGI(APP_TAG, "Display bitmap icons");
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_display_bitmap(dev_hdl, 31, 0, data_rx_img_32x32, 32, 32, false);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        ssd1306_display_bitmap(dev_hdl, 31, 0, data_tx_img_32x32, 32, 32, false);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Display x2 text
        ESP_LOGI(APP_TAG, "Display x2 Text");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text_x2(dev_hdl, 0, "{xTEXTx}", false);
        ssd1306_display_text_x2(dev_hdl, 2, " X2-X2", false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Display text
        ESP_LOGI(APP_TAG, "Display Text");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, 0, "SSD1306 128x64", false);
        ssd1306_display_text(dev_hdl, 1, "Hello World!!", false);
        ssd1306_display_text(dev_hdl, 2, "SSD1306 128x64", true);
        ssd1306_display_text(dev_hdl, 3, "Hello World!!", true);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        
        // Display Count Down
        ESP_LOGI(APP_TAG, "Display Count Down");
        memset(image, 0, sizeof(image));
        ssd1306_display_image(dev_hdl, top, (6*8-1), image, sizeof(image));
        ssd1306_display_image(dev_hdl, top+1, (6*8-1), image, sizeof(image));
        ssd1306_display_image(dev_hdl, top+2, (6*8-1), image, sizeof(image));
        for(int font = 0x39; font > 0x30; font--) {
            memset(image, 0, sizeof(image));
            ssd1306_display_image(dev_hdl, top+1, (7*8-1), image, 8);
            memcpy(image, font8x8_latin_tr[font], 8);
            if (dev_hdl->dev_config.flip_enabled) ssd1306_flip_buffer(image, 8);
            ssd1306_display_image(dev_hdl, top+1, (7*8-1), image, 8);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        
        // Scroll Up
        ESP_LOGI(APP_TAG, "Scroll Up");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, 0, "---Scroll  UP---", true);
        ssd1306_set_software_scroll(dev_hdl, (dev_hdl->pages - 1), 1);
        for (int line = 0; line < bottom+10; line++) {
            lineChar[0] = 0x01;
            sprintf(&lineChar[1], " Line %02d", line);
            ssd1306_display_scroll_text(dev_hdl, lineChar, false);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        
        // Scroll Down
        ESP_LOGI(APP_TAG, "Scroll Down");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, 0, "--Scroll  DOWN--", true);
        ssd1306_set_software_scroll(dev_hdl, 1, (dev_hdl->pages - 1) );
        for (int page = 0; page < bottom+10; page++) {
            lineChar[0] = 0x02;
            sprintf(&lineChar[1], " Line %02d", page);
            ssd1306_display_scroll_text(dev_hdl, lineChar, false);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Page Down
        ESP_LOGI(APP_TAG, "Page Down");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, 0, "---Page   DOWN---", true);
        ssd1306_set_software_scroll(dev_hdl, 1, (dev_hdl->pages-1) );
        for (int page = 0; page < bottom+10; page++) {
            if ( (page % (dev_hdl->pages-1)) == 0) ssd1306_clear_scroll_display(dev_hdl);
            lineChar[0] = 0x02;
            sprintf(&lineChar[1], " Line %02d", page);
            ssd1306_display_scroll_text(dev_hdl, lineChar, false);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        // Horizontal Scroll
        ESP_LOGI(APP_TAG, "Horizontal Scroll");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, center, "Horizontal", false);
        ssd1306_set_hardware_scroll(dev_hdl, SSD1306_SCROLL_RIGHT, SSD1306_SCROLL_2_FRAMES);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_set_hardware_scroll(dev_hdl, SSD1306_SCROLL_LEFT, SSD1306_SCROLL_2_FRAMES);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_set_hardware_scroll(dev_hdl, SSD1306_SCROLL_STOP, SSD1306_SCROLL_2_FRAMES);
        
        // Vertical Scroll
        ESP_LOGI(APP_TAG, "Vertical Scroll");
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, center, "Vertical", false);
        ssd1306_set_hardware_scroll(dev_hdl, SSD1306_SCROLL_DOWN, SSD1306_SCROLL_2_FRAMES);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_set_hardware_scroll(dev_hdl, SSD1306_SCROLL_UP, SSD1306_SCROLL_2_FRAMES);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ssd1306_set_hardware_scroll(dev_hdl, SSD1306_SCROLL_STOP, SSD1306_SCROLL_2_FRAMES);

        // Bitmaps
        ESP_LOGI(APP_TAG, "Bitmaps");
        ssd1306_display_text(dev_hdl, 1, "BATMAN", false);
        int bitmapWidth = 4*8;
        int width = dev_hdl->width;
        int xpos = width / 2; // center of width
        xpos = xpos - bitmapWidth/2; 
        int ypos = 16;
        ESP_LOGD(APP_TAG, "width=%d xpos=%d", width, xpos);
        ssd1306_display_bitmap(dev_hdl, xpos, ypos, batman, 32, 13, false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        for(int i=0;i<128;i++) {
            ssd1306_set_display_wrap_around(dev_hdl, SSD1306_SCROLL_RIGHT, 2, 3, 0);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        ssd1306_clear_display(dev_hdl, false);
        ssd1306_display_bitmap(dev_hdl, 0, 0, logo_mischianti, 128, 64, false);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        for(int i=0;i<64;i++) {
            ssd1306_set_display_wrap_around(dev_hdl, SSD1306_SCROLL_UP, 0, 127, 0);
        }
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        ssd1306_clear_display(dev_hdl, false);
        ssd1306_display_bitmap(dev_hdl, 0, 0, fleischer, 128, 64, false);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Invert
        ESP_LOGI(APP_TAG, "Invert");
        ssd1306_clear_display(dev_hdl, true);
        ssd1306_set_display_contrast(dev_hdl, 0xff);
        ssd1306_display_text(dev_hdl, center, "  Good Bye!!", true);
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        // Fade Out
        ESP_LOGI(APP_TAG, "Fade Out");
        ssd1306_fadeout_display(dev_hdl);
        //
        ESP_LOGI(APP_TAG, "######################## SSD1306 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE + 10 );
    }
    //
    // free resources
    ssd1306_delete( dev_hdl );
    vTaskDelete( NULL );
}
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
