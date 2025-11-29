# Technical Documentation: ESP-IDF Driver for Sensirion SHT4X Series of Sensors

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-007ACC?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAxMDAgMTAwIj48ZGVmcz48bGluZWFyR3JhZGllbnQgaWQ9ImEiIHgxPSI1MCIgeTE9IjAiIHgyPSI1MCIgeTI9IjEwMCIgZ3JhZGllbnRVbml0cz0idXNlclNwYWNlT25Vc2UiPjxzdG9wIHN0b3AtY29sb3I9IiNmZmYiLz48c3RvcCBvZmZzZXQ9IjEiIHN0b3AtY29sb3I9IiNmZmYiIHN0b3Atb3BhY2l0eT0iMCIvPjwvbGluZWFyR3JhZGllbnQ+PC9kZWZzPjxwYXRoIGZpbGw9IiMwMDY1QTkiIGQ9Ik05Ni40NiAxMC44IDc1Ljg2Ljg4QzczLjQ3LS4yNyA3MC42Mi4yMSA2OC43NSAyLjA4TDEuMyA2My41OGMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZsNS41MSA1LjAxYzEuNDkgMS4zNSAzLjcyIDEuNDUgNS4zMi4yNGw4MS4yMy02MS42MmMyLjcyLTIuMDcgNi42NC0uMTMgNi42NCAzLjI5di0uMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMDA3QUNDIiBkPSJNOTYuNDYgODkuMiA3NS44NiA5OS4xMmMtMi4zOSAxLjE1LTUuMjQuNjYtNy4xMS0xLjJMMS4zIDM2LjQyYy0xLjgyLTEuNjUtMS44Mi00LjUxIDAtNi4xN2w1LjUxLTUuMDFjMS40OS0xLjM1IDMuNzItMS40NSA1LjMyLS4yM2w4MS4yMyA2MS42MmMyLjcyIDIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMCAyLjQtMS4zOCA0LjU5LTMuNTQgNS42M1oiLz48cGF0aCBmaWxsPSIjMUY5Q0YwIiBkPSJNNzUuODYgOTkuMTNjLTIuMzkgMS4xNS01LjI0LjY2LTcuMTEtMS4yMSAyLjMxIDIuMzEgNi4yNS42NiA2LjI1LTIuNlY0LjY3YzAtMy4yNi0zLjk0LTQuODktNi4yNS0yLjU4IDEuODctMS44NyA0LjcyLTIuMzYgNy4xMS0xLjIxbDIwLjYgOS45MWMyLjE2IDEuMDQgMy41NCAzLjIzIDMuNTQgNS42M3Y2Ny4xN2MwIDIuNC0xLjM4IDQuNTktMy41NCA1LjYzbC0yMC42IDkuOTFaIi8+PHBhdGggZmlsbD0idXJsKCNhKSIgZmlsbC1ydWxlPSJldmVub2RkIiBkPSJNNzAuODUgOTkuMzJjMS41OC42MSAzLjM3LjU3IDQuOTYtLjE5bDIwLjU5LTkuOTFjMi4xNi0xLjA0IDMuNTQtMy4yMyAzLjU0LTUuNjNWMTYuNDFjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M0w3NS44MS44N2MtMi4wOS0xLTQuNTMtLjc2LTYuMzYuNTgtLjI2LjE5LS41MS40LS43NC42NEwyOS4yOSAzOC4wNGwtMTcuMTctMTMuMDNjLTEuNi0xLjIxLTMuODMtMS4xMS01LjMyLjI0bC01LjUgNS4wMWMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZMMTYuMTkgNTAgMS4zIDYzLjU4Yy0xLjgyIDEuNjUtMS44MiA0LjUxIDAgNi4xN2w1LjUxIDUuMDFjMS40OSAxLjM1IDMuNzIgMS40NSA1LjMyLjIzbDE3LjE3LTEzLjAzIDM5LjQxIDM1Ljk2Yy42Mi42MiAxLjM2IDEuMDkgMi4xNCAxLjRaTTc0Ljk1IDI3LjMgNDUuMDUgNTBsMjkuOSAyMi43VjI3LjNaIiBvcGFjaXR5PSIuMjUiIHN0eWxlPSJtaXgtYmxlbmQtbW9kZTpvdmVybGF5Ii8+PC9zdmc+&logoColor=white)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_sht4x.svg)](https://registry.platformio.org/libraries/k0i05/esp_sht4x)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_sht4x/badge.svg)](https://components.espressif.com/components/k0i05/esp_sht4x)

## Overview

The `esp_sht4x` component is an espressif IoT development framework (ESP-IDF) compatible driver for the Sensirion SHT4x series of digital humidity and temperature sensors (SHT40, SHT41, SHT43, and SHT45). It utilizes the I2C bus for communication and provides a high-level API for sensor configuration, measurement, and data conversion.  It is designed for portability, maintainability, and hardware abstraction, supporting I2C and SPI communication via a generic Hardware Abstraction Layer (HAL).

```text
components
└── esp_sht4x
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── sht4x_version.h
    │   └── sht4x.h
    └── sht4x.c
```

## Architecture & Design

- Written in C, C99-compliant, and safe for use in C projects.
- All bus communication is abstracted via a HAL interface (`hal_master.h`), decoupling the driver from ESP-IDF-specific APIs.
- The main API is exposed via `sht4x.h`.
- Device configuration and state are encapsulated in an opaque handle (`sht4x_handle_t`).
- Thread-safe for typical use; measurement functions are blocking.
- Internal device structure (`sht4x_device_t`) holds configuration, HAL handle, and serial number.

## HAL (Hardware Abstraction Layer)

- The driver uses `hal_master_interfaces_t` and a `void *hal_config` for bus operations, supporting I2C (and potentially other buses in the future).
- The user must provide a valid HAL master device handle and a configuration struct during initialization.
- All bus operations (read/write) are performed via the HAL interface, enabling portability and testability.
- HAL functions wrap low-level bus operations and propagate errors as `esp_err_t`.

## Configuration Structure

The configuration is provided via a `sht4x_config_t` struct:

```c
typedef struct sht4x_config_s {
    hal_master_interfaces_t hal_bif;           // HAL master bus interface type
    void *hal_config;                          // HAL master bus interface configuration (e.g., i2c_device_config_t)
    sht4x_repeat_modes_t repeat_mode;          // Measurement repeatability mode
    sht4x_heater_modes_t heater_mode;          // Heater mode
} sht4x_config_t;
```

### Default Configuration Macro

Use the macro `SHT4X_CONFIG_DEFAULT` to initialize a config struct with recommended defaults:

```c
#define SHT4X_CONFIG_DEFAULT { \
    .hal_bif = HAL_MASTER_BIF_I2C, \
    .hal_config = (void*)&(i2c_device_config_t){ \
        .device_address = I2C_SHT4X_DEV_ADDR_LO, \
        .scl_speed_hz = I2C_SHT4X_DEV_CLK_SPD \
    }, \
    .repeat_mode = SHT4X_REPEAT_HIGH, \
    .heater_mode = SHT4X_HEATER_OFF \
}
```

## Data Structures

### Enumerations

#### `sht4x_repeat_modes_t`

- `SHT4X_REPEAT_HIGH`: High repeatability (Longest duration)
- `SHT4X_REPEAT_MEDIUM`: Medium repeatability
- `SHT4X_REPEAT_LOW`: Low repeatability (Shortest duration)

#### `sht4x_heater_modes_t`

- `SHT4X_HEATER_OFF`: Heater disabled
- `SHT4X_HEATER_HIGH_LONG`: High power (~200mW), 1s
- `SHT4X_HEATER_HIGH_SHORT`: High power (~200mW), 0.1s
- `SHT4X_HEATER_MEDIUM_LONG`: Medium power (~110mW), 1s
- `SHT4X_HEATER_MEDIUM_SHORT`: Medium power (~110mW), 0.1s
- `SHT4X_HEATER_LOW_LONG`: Low power (~20mW), 1s
- `SHT4X_HEATER_LOW_SHORT`: Low power (~20mW), 0.1s

### Data Record: `sht4x_data_record_t`

A container for all measurement outputs:

- `drybulb` (float): Dry-Bulb Temperature in °C
- `humidity` (float): Relative Humidity in %
- `dewpoint` (float): Calculated Dew-Point in °C
- `wetbulb` (float): Calculated Wet-Bulb temperature in °C

## API Reference

### Initialization & Device Management

- `sht4x_init()` - Allocates resources, configures the device, probes the bus, resets the sensor, and returns a handle.
- `sht4x_remove()` - Removes the device from the HAL master bus.
- `sht4x_delete()` - Removes the device and frees the handle memory.

### Measurement

- `sht4x_get_measurement()` - Blocking read of temperature and humidity.
- `sht4x_get_measurements()` - Blocking read of temperature, humidity, dew-point, and wet-bulb.
- `sht4x_get_measurement_record()` - Returns all data in a `sht4x_data_record_t` struct.

### Configuration

- `sht4x_get_repeat_mode()` / `sht4x_set_repeat_mode()`
- `sht4x_get_heater_mode()` / `sht4x_set_heater_mode()`

### System & Maintenance

- `sht4x_reset()` - Issues a soft-reset command.
- `sht4x_get_fw_version()` / `sht4x_get_fw_version_number()`

## Error Handling & Validation

- All public API functions return `esp_err_t`.
- Arguments are validated with macros; invalid arguments return `ESP_ERR_INVALID_ARG`.
- All bus and HAL errors are propagated up to the API.
- CRC8 is checked on all received data; CRC errors return `ESP_ERR_INVALID_CRC`.

## Internal Implementation Details

### Device Structure

- Internal `sht4x_device_t` holds configuration, HAL handle, and serial number.
- Memory is dynamically allocated and freed by the driver.

### CRC8 Verification

- All 16-bit data words from the sensor are followed by an 8-bit CRC.
- The driver implements the CRC-8 algorithm (polynomial 0x31, init 0xFF).
- CRC is checked for all reads; failure returns `ESP_ERR_INVALID_CRC`.

### Measurement Timing & Command Mapping

- Measurement duration is determined by repeatability and heater mode.
- Helper functions map configuration to I2C command bytes and required delays.
- Blocking measurement functions delay the calling task for the required time (up to ~1.1s for long heater pulses).
- Retry logic is used for read operations if the sensor is busy.

### Signal Conversion

- Raw ADC values are converted to physical units using datasheet formulas:
    - Temperature: $T = -45 + 175 \cdot \frac{S_T}{2^{16}-1}$
    - Humidity: $RH = -6 + 125 \cdot \frac{S_{RH}}{2^{16}-1}$

### Derived Calculations

- Dew point is calculated using the Magnus formula.
- Wet bulb is calculated using the Stull formula.
- Helper functions validate input ranges for all derived calculations.

### Register Access

- Serial number and reset are accessed via specific command sequences.
- All register access is performed via HAL functions, which wrap the bus operations.

### Thread Safety & Blocking Behavior

- Measurement functions are blocking and delay the calling task.
- The driver is safe for use in typical FreeRTOS tasks but should not be called from timer callbacks.

## Example Usage

```c
#include "sht4x.h"

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

sht4x_config_t config = SHT4X_CONFIG_DEFAULT;
sht4x_handle_t handle = NULL;

err = sht4x_init(i2c0_bus_handle, &config, &handle);
if (err == ESP_OK) {
    float temp, rh;
    sht4x_get_measurement(handle, &temp, &rh);
}
```

## Notes

- The driver is designed for extensibility and portability.
- The HAL interface allows for future support of other bus types.
- All configuration and state are encapsulated in the device handle.
- The driver is C99-compliant and safe for use in C projects.

## References

- [SHT4X Repository](https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_sht4x)
- [SHT4X Datasheet](https://sensirion.com/media/documents/8EC8B2B7/64B1B2B7/Sensirion_Humidity_Sensors_SHT4x_Datasheet.pdf)
- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
