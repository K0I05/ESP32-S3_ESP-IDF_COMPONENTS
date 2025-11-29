# Technical Documentation: ESP-IDF Driver for Texas Instruments HDC1080 Sensor

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-007ACC?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAxMDAgMTAwIj48ZGVmcz48bGluZWFyR3JhZGllbnQgaWQ9ImEiIHgxPSI1MCIgeTE9IjAiIHgyPSI1MCIgeTI9IjEwMCIgZ3JhZGllbnRVbml0cz0idXNlclNwYWNlT25Vc2UiPjxzdG9wIHN0b3AtY29sb3I9IiNmZmYiLz48c3RvcCBvZmZzZXQ9IjEiIHN0b3AtY29sb3I9IiNmZmYiIHN0b3Atb3BhY2l0eT0iMCIvPjwvbGluZWFyR3JhZGllbnQ+PC9kZWZzPjxwYXRoIGZpbGw9IiMwMDY1QTkiIGQ9Ik05Ni40NiAxMC44IDc1Ljg2Ljg4QzczLjQ3LS4yNyA3MC42Mi4yMSA2OC43NSAyLjA4TDEuMyA2My41OGMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZsNS41MSA1LjAxYzEuNDkgMS4zNSAzLjcyIDEuNDUgNS4zMi4yNGw4MS4yMy02MS42MmMyLjcyLTIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMDA3QUNDIiBkPSJNOTYuNDYgODkuMiA3NS44NiA5OS4xMmMtMi4zOSAxLjE1LTUuMjQuNjYtNy4xMS0xLjJMMS4zIDM2LjQyYy0xLjgyLTEuNjUtMS44Mi00LjUxIDAtNi4xN2w1LjUxLTUuMDFjMS40OS0xLjM1IDMuNzItMS40NSA1LjMyLS4yM2w4MS4yMyA2MS42MmMyLjcyIDIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMUY5Q0YwIiBkPSJNNzUuODYgOTkuMTNjLTIuMzkgMS4xNS01LjI0LjY2LTcuMTEtMS4yMSAyLjMxIDIuMzEgNi4yNS42NiA2LjI1LTIuNlY0LjY3YzAtMy4yNi0zLjk0LTQuODktNi4yNS0yLjU4IDEuODctMS44NyA0LjcyLTIuMzYgNy4xMS0xLjIxbDIwLjYgOS45MWMyLjE2IDEuMDQgMy41NCAzLjIzIDMuNTQgNS42M3Y2Ny4xN2MwIDIuNC0xLjM4IDQuNTktMy41NCA1LjYzbC0yMC42IDkuOTFaIi8+PHBhdGggZmlsbD0idXJsKCNhKSIgZmlsbC1ydWxlPSJldmVub2RkIiBkPSJNNzAuODUgOTkuMzJjMS41OC42MSAzLjM3LjU3IDQuOTYtLjE5bDIwLjU5LTkuOTFjMi4xNi0xLjA0IDMuNTQtMy4yMyAzLjU0LTUuNjNWMTYuNDFjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M0w3NS44MS44N2MtMi4wOS0xLTQuNTMtLjc2LTYuMzYuNTgtLjI2LjE5LS41MS40LS43NC42NEwyOS4yOSAzOC4wNGwtMTcuMTctMTMuMDNjLTEuNi0xLjIxLTMuODMtMS4xMS01LjMyLjI0bC01LjUgNS4wMWMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZMMTYuMTkgNTAgMS4zIDYzLjU4Yy0xLjgyIDEuNjUtMS44MiA0LjUxIDAgNi4xN2w1LjUxIDUuMDFjMS40OSAxLjM1IDMuNzIgMS40NSA1LjMyLjIzbDE3LjE3LTEzLjAzIDM5LjQxIDM1Ljk2Yy42Mi42MiAxLjM2IDEuMDkgMi4xNCAxLjRaTTc0Ljk1IDI3LjMgNDUuMDUgNTBsMjkuOSAyMi43VjI3LjNaIiBvcGFjaXR5PSIuMjUiIHN0eWxlPSJtaXgtYmxlbmQtbW9kZTpvdmVybGF5Ii8+PC9zdmc+&logoColor=white)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_hdc1080.svg)](https://registry.platformio.org/libraries/k0i05/esp_hdc1080)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_hdc1080/badge.svg)](https://components.espressif.com/components/k0i05/esp_hdc1080)

## Overview

The `esp_hdc1080` component is an espressif IoT development framework (ESP-IDF) compatible driver for the Texas Instruments HDC1080 digital humidity and temperature sensor. It utilizes the I2C bus for communication and provides a high-level API for sensor configuration, measurement, and data conversion.  It is designed for portability, maintainability, and hardware abstraction, supporting I2C and SPI communication via a generic Hardware Abstraction Layer (HAL).

```text
components
└── esp_hdc1080
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── hdc1080_version.h
    │   └── hdc1080.h
    └── hdc1080.c
```

## Architecture & Dependencies

This driver is built on a Hardware Abstraction Layer (HAL) architecture. All low-level bus operations (such as I2C or SPI transactions) are performed through a generic `hal_master` interface, which abstracts the underlying hardware implementation. This allows the driver to be hardware-agnostic and portable across different platforms or bus implementations.

**Dependencies:**

- `hal_master.h`: For all bus communication (I2C and SPI buses are supported).
- `esp_err.h`: For standard error handling.
- `esp_log.h`: For logging.
- `freertos/FreeRTOS.h`, `freertos/task.h`: For delays and task management.
- `math.h`: For dew-point and wet-bulb calculations.

## Data Structures

### Configuration: `hdc1080_config_t`

This structure defines the configuration parameters required to initialize the HDC1080 device and its communication interface via the HAL. It encapsulates both sensor-specific and bus/HAL-specific settings.

| Field                    | Type                                 | Description                                                                                 |
|--------------------------|--------------------------------------|---------------------------------------------------------------------------------------------|
| `hal_config`             | `void*`                              | Pointer to the HAL master bus configuration instance (required for all communication).      |
| `hal_bif`                | `hal_master_interfaces_t`            | HAL master bus interface type.                                                              |
| `temperature_resolution` | `hdc1080_temperature_resolutions_t`  | Temperature measurement resolution (see enum below).                                        |
| `humidity_resolution`    | `hdc1080_humidity_resolutions_t`     | Humidity measurement resolution (see enum below).                                           |
| `heater_enabled`         | `bool`                               | Enables or disables the on-chip heater (default: `false`).                                  |

**Default Configuration Macro:** `HDC1080_CONFIG_DEFAULT`

The default macro initializes all fields to recommended values for typical operation. The `hal_master` pointer must be set by the user before initialization.

```c
#define HDC1080_CONFIG_DEFAULT {          \
    .hal_bif        = HAL_MASTER_BIF_I2C, \
    .hal_config     = (void*)&(i2c_device_config_t){\
        .device_address = I2C_HDC1080_DEV_ADDR_0,   \
        .scl_speed_hz   = I2C_HDC1080_DEV_CLK_SPD   \
    },                                              \
    .temperature_resolution     = HDC1080_TEMPERATURE_RESOLUTION_14BIT, \
    .humidity_resolution        = HDC1080_HUMIDITY_RESOLUTION_14BIT,    \
    .heater_enabled             = false                                 \
}
```

### Enumerations

#### `hdc1080_temperature_resolutions_t`

Controls the resolution of temperature measurements.

- `HDC1080_TEMPERATURE_RESOLUTION_14BIT`: 14-bit resolution.
- `HDC1080_TEMPERATURE_RESOLUTION_11BIT`: 11-bit resolution.

#### `hdc1080_humidity_resolutions_t`

Controls the resolution of humidity measurements.

- `HDC1080_HUMIDITY_RESOLUTION_14BIT`: 14-bit resolution.
- `HDC1080_HUMIDITY_RESOLUTION_11BIT`: 11-bit resolution.
- `HDC1080_HUMIDITY_RESOLUTION_8BIT`: 8-bit resolution.

### Data Record: `hdc1080_data_record_t`

A container for all measurement outputs.

- `drybulb` (float): Dry-Bulb Temperature in °C.
- `humidity` (float): Relative Humidity in %.
- `dewpoint` (float): Calculated Dew-Point in °C.
- `wetbulb` (float): Calculated Wet-Bulb temperature in °C.

## API Reference

### Initialization

- **`hdc1080_init`**: Allocates resources, probes the I2C bus, resets the sensor, configures the device, and initializes the device handle. Accepts a `void*` master handle.

### Measurement

- **`hdc1080_get_measurement`**: Performs a blocking measurement based on current settings and returns Temperature and Humidity.
- **`hdc1080_get_measurements`**: Same as above but also calculates and returns Dew-Point and Wet-Bulb.
- **`hdc1080_get_measurement_record`**: Returns all data in a `hdc1080_data_record_t` struct.

### Configuration

- **`hdc1080_set_temperature_resolution` / `hdc1080_get_temperature_resolution`**: Set/Get the temperature resolution.
- **`hdc1080_set_humidity_resolution` / `hdc1080_get_humidity_resolution`**: Set/Get the humidity resolution.
- **`hdc1080_enable_heater` / `hdc1080_disable_heater`**: Enable/Disable the on-chip heater.

### System & Maintenance

- **`hdc1080_reset`**: Sends a soft-reset command to the sensor and re-configures it.
- **`hdc1080_remove`**: Removes the device from the I2C bus (does not free memory).
- **`hdc1080_delete`**: Removes the device and frees the handle memory.
- **`hdc1080_get_fw_version`**: Returns the driver version string.
- **`hdc1080_get_fw_version_number`**: Returns the driver version as an integer.

## Implementation Details

### HAL-Based Communication

All communication with the HDC1080 sensor is performed through the HAL interface. The driver does not make any direct calls to ESP-IDF I2C or SPI functions. Instead, it uses the `hal_master` API to perform device probing, configuration, reading, and writing. This ensures that the driver logic is decoupled from the hardware layer and can be adapted to other platforms by providing a compatible HAL implementation.

- **Configuration**: The HAL is used to write/read the 16-bit configuration register.
- **Measurement**: The HAL is used to trigger measurements and read results, with appropriate delays handled at the driver level.

### Signal Conversion

Raw ADC values are converted to physical units using the formulas specified in the datasheet:

- **Temperature**: $T = \frac{S_T}{2^{16}} \cdot 165 - 40$
- **Humidity**: $RH = \frac{S_{RH}}{2^{16}} \cdot 100$

### Derived Calculations

- **Dew Point**: Calculated using the Magnus formula approximation.
- **Wet Bulb**: Calculated using an empirical formula based on dry-bulb temperature and relative humidity (Stull, 2011).

### Timing

The driver handles necessary delays:

- **Power-up**: Waits 30ms.
- **Soft Reset**: Waits 20ms.
- **Measurement**: Waits for a duration corresponding to the selected resolution (e.g., ~7ms for 14-bit).

## Hardware Abstraction Layer (HAL)

The driver is fully decoupled from any specific hardware or SDK by using a Hardware Abstraction Layer (HAL). All bus operations are performed through the `hal_master` interface, which is injected into the driver at initialization. The HAL provides a set of function pointers or API calls for device creation, configuration, read/write, and removal, allowing the driver to be reused with different hardware or bus implementations by simply swapping out the HAL implementation.

### HAL Integration and Device Handle

- The driver expects a valid HAL master bus handle (e.g., I2C) and a device configuration structure (such as `i2c_device_config_t`) to be provided at initialization.
- The driver creates a device handle (`hal_handle`) using the HAL's device creation function (e.g., `hal_master_new_i2c_device`).
- All subsequent communication (read, write, probe, remove) is performed using this device handle.
- The `hdc1080_device_t` structure contains both the sensor configuration and the HAL device handle.

### HAL Function Usage in Driver

The following HAL functions are used by the driver (actual signatures may vary by HAL implementation):

- `hal_master_new_i2c_device(i2c_master_handle, i2c_device_config_t*, hal_master_dev_handle_t*)`: Creates a new device on the bus.
- `hal_master_probe(hal_master_dev_handle_t, uint16_t device_address)`: Probes for device presence.
- `hal_master_read(hal_master_dev_handle_t, uint8_t* data, size_t len)`: Reads data from the device.
- `hal_master_write_command(hal_master_dev_handle_t, uint8_t command)`: Writes a command byte to the device.
- `hal_master_write_word_to(hal_master_dev_handle_t, uint8_t reg, uint16_t value)`: Writes a 16-bit word to a register.
- `hal_master_read_word_from(hal_master_dev_handle_t, uint8_t reg, uint16_t* value)`: Reads a 16-bit word from a register.
- `hal_master_remove(hal_master_dev_handle_t)`: Removes the device from the bus.
- `hal_master_delete(hal_master_dev_handle_t)`: Deletes and frees the device handle.

### HAL-Related Driver Functions

The driver implements several static inline functions that wrap HAL calls for specific sensor operations, such as:

- `hal_get_config_register`, `hal_set_config_register`: Read/write the sensor's configuration register via HAL.
- `hal_get_adc_signals`: Read raw ADC values for temperature and humidity via HAL.
- `hal_set_reset_register`: Issue a soft reset via HAL.
- `hal_setup_registers`: Configure the sensor using HAL calls.

All error handling and device state management is performed at the driver level, with HAL errors propagated up to the public API.

## Internal Helper Functions

These `static inline` functions perform utility tasks, calculations, and logic mapping to support the public API and HAL.

### Timing & Resolution Mapping

- **`get_humidity_duration`**: Returns the required measurement duration (in ms) based on the selected humidity resolution.
- **`get_temperature_duration`**: Returns the required measurement duration (in ms) based on the selected temperature resolution.

### Range Validation

- **`is_value_in_range`**: Generic helper to check if a float value is within a min/max range.
- **`is_drybulb_in_range`**: Validates if a temperature value is within the sensor's operating range (-40 to 125 °C).
- **`is_humidity_in_range`**: Validates if a humidity value is within the sensor's operating range (0 to 100 %).

### Helper: Derived Calculations

- **`get_dewpoint_temperature`**: Calculates dew point temperature using the Magnus formula. Includes input validation.
- **`get_wetbulb_temperature`**: Calculates wet bulb temperature using the Stull formula. Includes input validation.

### Helper: Signal Conversion

- **`convert_adc_signal_to_temperature`**: Converts the raw 16-bit ADC temperature value to degrees Celsius.
- **`convert_adc_signal_to_humidity`**: Converts the raw 16-bit ADC humidity value to relative humidity percentage.

## Example Usage

```c
#include "hdc1080.h"

#define I2C0_MASTER_CONFIG_DEFAULT {                                \
        .clk_source                     = I2C_CLK_SRC_DEFAULT,      \
        .i2c_port                       = I2C0_MASTER_PORT,         \
        .scl_io_num                     = I2C0_MASTER_SCL_IO,       \
        .sda_io_num                     = I2C0_MASTER_SDA_IO,       \
        .glitch_ignore_cnt              = 7,                        \
        .flags.enable_internal_pullup   = true, }

esp_err_t err = ESP_OK;

i2c_master_bus_config_t  i2c0_bus_config = I2C0_MASTER_CONFIG_DEFAULT;
i2c_master_bus_handle_t  i2c0_bus_handle = NULL;

err = i2c_new_master_bus(&i2c0_bus_config, &i2c0_bus_handle);
if (err != ESP_OK) assert(i2c0_bus_handle);

hdc1080_config_t config = HDC1080_CONFIG_DEFAULT;
hdc1080_handle_t handle = NULL;

err = hdc1080_init(i2c0_bus_handle, &config, &handle);
if (err == ESP_OK) {
    float temp, rh;
    hdc1080_get_measurement(handle, &temp, &rh);
}
```

## Notes

- The driver is designed for extensibility and portability.
- The HAL interface allows for future support of other bus types.
- All configuration and state are encapsulated in the device handle.
- The driver is C99-compliant and safe for use in C projects.

## References

- [HDC1080 Repository](https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_hdc1080)
- [HDC1080 Datasheet](https://www.ti.com/lit/ds/symlink/hdc1080.pdf?ts=1764315106676&ref_url=https%253A%252F%252Fwww.google.com%252F)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
