# Technical Documentation: ESP-IDF Driver for Aosong AHTxx (AHT10/AHT20/AHT21/AHT25/AHT30)

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-007ACC?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAxMDAgMTAwIj48ZGVmcz48bGluZWFyR3JhZGllbnQgaWQ9ImEiIHgxPSI1MCIgeTE9IjAiIHgyPSI1MCIgeTI9IjEwMCIgZ3JhZGllbnRVbml0cz0idXNlclNwYWNlT25Vc2UiPjxzdG9wIHN0b3AtY29sb3I9IiNmZmYiLz48c3RvcCBvZmZzZXQ9IjEiIHN0b3AtY29sb3I9IiNmZmYiIHN0b3Atb3BhY2l0eT0iMCIvPjwvbGluZWFyR3JhZGllbnQ+PC9kZWZzPjxwYXRoIGZpbGw9IiMwMDY1QTkiIGQ9Ik05Ni40NiAxMC44IDc1Ljg2Ljg4QzczLjQ3LS4yNyA3MC42Mi4yMSA2OC43NSAyLjA4TDEuMyA2My41OGMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZsNS41MSA1LjAxYzEuNDkgMS4zNSAzLjcyIDEuNDUgNS4zMi4yNGw4MS4yMy02MS42MmMyLjcyLTIuMDcgNi42NC0uMTMgNi42NCAzLjI5di0uMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMDA3QUNDIiBkPSJNOTYuNDYgODkuMiA3NS44NiA5OS4xMmMtMi4zOSAxLjE1LTUuMjQuNjYtNy4xMS0xLjJMMS4zIDM2LjQyYy0xLjgyLTEuNjUtMS44Mi00LjUxIDAtNi4xN2w1LjUxLTUuMDFjMS40OS0xLjM1IDMuNzItMS40NSA1LjMyLS4yM2w4MS4yMyA2MS42MmMyLjcyIDIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMCAyLjQtMS4zOCA0LjU5LTMuNTQgNS42M1oiLz48cGF0aCBmaWxsPSIjMUY5Q0YwIiBkPSJNNzUuODYgOTkuMTNjLTIuMzkgMS4xNS01LjI0LjY2LTcuMTEtMS4yMSAyLjMxIDIuMzEgNi4yNS42NiA2LjI1LTIuNlY0LjY3YzAtMy4yNi0zLjk0LTQuODktNi4yNS0yLjU4IDEuODctMS44NyA0LjcyLTIuMzYgNy4xMS0xLjIxbDIwLjYgOS45MWMyLjE2IDEuMDQgMy41NCAzLjIzIDMuNTQgNS42M3Y2Ny4xN2MwIDIuNC0xLjM4IDQuNTktMy41NCA1LjYzbC0yMC42IDkuOTFaIi8+PHBhdGggZmlsbD0idXJsKCNhKSIgZmlsbC1ydWxlPSJldmVub2RkIiBkPSJNNzAuODUgOTkuMzJjMS41OC42MSAzLjM3LjU3IDQuOTYtLjE5bDIwLjU5LTkuOTFjMi4xNi0xLjA0IDMuNTQtMy4yMyAzLjU0LTUuNjNWMTYuNDFjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M0w3NS44MS44N2MtMi4wOS0xLTQuNTMtLjc2LTYuMzYuNTgtLjI2LjE5LS41MS40LS43NC42NEwyOS4yOSAzOC4wNGwtMTcuMTctMTMuMDNjLTEuNi0xLjIxLTMuODMtMS4xMS01LjMyLjI0bC01LjUgNS4wMWMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZMMTYuMTkgNTAgMS4zIDYzLjU4Yy0xLjgyIDEuNjUtMS44MiA0LjUxIDAgNi4xN2w1LjUxIDUuMDFjMS40OSAxLjM1IDMuNzIgMS40NSA1LjMyLjIzbDE3LjE3LTEzLjAzIDM5LjQxIDM1Ljk2Yy42Mi42MiAxLjM2IDEuMDkgMi4xNCAxLjRaTTc0Ljk1IDI3LjMgNDUuMDUgNTBsMjkuOSAyMi43VjI3LjNaIiBvcGFjaXR5PSIuMjUiIHN0eWxlPSJtaXgtYmxlbmQtbW9kZTpvdmVybGF5Ii8+PC9zdmc+&logoColor=white)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_ahtxx.svg)](https://registry.platformio.org/libraries/k0i05/esp_ahtxx)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_ahtxx/badge.svg)](https://components.espressif.com/components/k0i05/esp_ahtxx)

## Overview

The `esp_ahtxx` component is an espressif IoT development framework (ESP-IDF) compatible driver for the Aosong AHTxx family of digital humidity and temperature sensors (AHT10, AHT20, AHT21, AHT25, AHT30). It supports I2C communication and provides a high-level API for sensor configuration, measurement, and data conversion, with support for dew-point and wet-bulb calculations.

```text
components
└── esp_ahtxx
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── ahtxx_version.h
    │   └── ahtxx.h
    └── ahtxx.c
```

## Architecture & Design

This driver is built on a Hardware Abstraction Layer (HAL) using the `hal_master` interface, which decouples the driver from any specific hardware or SDK. All bus operations (I2C) are performed through the HAL, allowing for portability and testability.

**Dependencies:**

- `hal_master.h`: For all bus communication (I2C bus supported).
- `esp_err.h`: For standard error handling.
- `esp_log.h`: For logging.
- `freertos/FreeRTOS.h`, `freertos/task.h`: For delays and task management.
- `math.h`: For dew-point and wet-bulb calculations.

## Hardware Abstraction Layer (HAL)

The driver uses the `hal_master_interfaces_t` type and a `void *hal_config` pointer for bus operations, supporting I2C. The user must provide a valid HAL master device handle and configuration struct during initialization. All bus operations (read/write) are performed via the HAL interface, and errors are propagated as `esp_err_t`.

### HAL Integration and Device Handle

- The device handle is an opaque pointer (`ahtxx_handle_t`), internally referencing a structure containing the configuration and HAL device handle.
- The configuration struct includes the bus interface type and configuration pointer, as well as the sensor type.

### HAL-Related Driver Functions

- All low-level I2C operations (read, write, device creation/removal) are performed via the HAL.
- HAL functions are used for device initialization, register access, measurement triggering, and device removal.

## Configuration Structure

### Configuration: `ahtxx_config_t`

| Field         | Type                      | Description                                      |
|--------------|---------------------------|--------------------------------------------------|
| `hal_bif`    | `hal_master_interfaces_t`  | HAL master bus interface type (I2C)              |
| `hal_config` | `void*`                   | HAL bus configuration (e.g., `i2c_device_config_t`) |
| `sensor_type`| `ahtxx_sensor_types_t`     | Sensor type (AHT10, AHT20, AHT21, AHT25, AHT30)  |

### Default Configuration Macros

- `AHT10_CONFIG_DEFAULT`, `AHT20_CONFIG_DEFAULT`, `AHT21_CONFIG_DEFAULT`, `AHT25_CONFIG_DEFAULT`, `AHT30_CONFIG_DEFAULT`: Macros for initializing the configuration struct for each supported sensor type.

## Data Structures

### Enumerations

- `ahtxx_sensor_types_t`: Enumerates supported sensor types (AHT10, AHT20, AHT21, AHT25, AHT30).

## API Reference

### Initialization & Device Management

- `ahtxx_init`: Initializes the sensor on the I2C bus using the provided HAL handle and configuration.
- `ahtxx_remove`: Removes the device from the bus (does not free memory).
- `ahtxx_delete`: Removes the device and frees the handle memory.

### Measurement

- `ahtxx_get_measurement`: Reads temperature and humidity.
- `ahtxx_get_measurements`: Reads temperature, humidity, and calculates dew-point and wet-bulb temperatures.

### Configuration

- `ahtxx_get_busy_status`: Reads the busy status flag.
- `ahtxx_get_calibration_status`: Reads the calibration status flag.
- `ahtxx_get_status`: Reads both busy and calibration status flags.

### System & Maintenance

- `ahtxx_reset`: Issues a soft-reset and re-initializes the sensor.
- `ahtxx_get_fw_version`: Returns the driver version string.
- `ahtxx_get_fw_version_number`: Returns the driver version as an integer.

## Error Handling & Validation

- All public API functions return `esp_err_t`.
- Arguments are validated with macros; invalid arguments return `ESP_ERR_INVALID_ARG`.
- All bus and HAL errors are propagated up to the API.

## Implementation Details

### HAL-Based Communication

- All I2C transactions are performed via the HAL interface, using the configuration provided at initialization.
- The driver supports only I2C for AHTxx sensors.

### Device Structure

- The internal device structure contains the configuration and the HAL device handle.

### Measurement Timing & Command Mapping

- The driver handles all required delays for power-up, reset, initialization, and measurement as specified in the datasheet.
- Measurement is triggered by sending a command, followed by a delay and a read operation.

### Signal Conversion

- Raw ADC values are converted to engineering units:
    - **Temperature:** `T = ((signal / 2^20) * 200) - 50` (°C)
    - **Humidity:** `RH = (signal / 2^20) * 100` (%)

### Derived Calculations

- **Dew Point:** Calculated using the Magnus formula.
- **Wet Bulb:** Calculated using the Stull formula.

### Timing

- **Power-up:** Waits 120ms.
- **Reset:** Waits 25ms.
- **Initialization:** Waits 15ms.
- **Measurement:** Waits ~80ms (typical).

### Register Access

- Register access is performed via HAL functions for reading/writing device registers and status.
- CRC8 validation is performed for data integrity (AHT20/21/25/30).

### Thread Safety & Blocking Behavior

- All API functions are blocking and not thread-safe by default.
- Delays are implemented using FreeRTOS `vTaskDelay`.

## Example Usage

```c
#include "ahtxx.h"

// Configure for AHT20
static ahtxx_config_t config = AHT20_CONFIG_DEFAULT;
ahtxx_handle_t handle;

// Initialize (assuming i2c_master_bus_handle_t i2c_bus is already created)
ahtxx_init(i2c_bus, &config, &handle);

float temperature, humidity;
ahtxx_get_measurement(handle, &temperature, &humidity);

// Clean up
ahtxx_delete(handle);
```

## Notes

- The driver is designed for extensibility and portability.
- The HAL interface allows for future support of other bus types.
- All configuration and state are encapsulated in the device handle.
- The driver is C99-compliant and safe for use in C projects.

## References

- [AHTxx Repository](https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_ahtxx)
- [AHTxx Datasheet](https://aosong.com/en/products/details/AHT20.html)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
