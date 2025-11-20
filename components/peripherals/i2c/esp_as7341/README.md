# OSRAM AS7341 Sensor

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red?logo=espressif)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://img.shields.io/badge/Edited_with-VS_Code-007ACC?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAxMDAgMTAwIj48ZGVmcz48bGluZWFyR3JhZGllbnQgaWQ9ImEiIHgxPSI1MCIgeTE9IjAiIHgyPSI1MCIgeTI9IjEwMCIgZ3JhZGllbnRVbml0cz0idXNlclNwYWNlT25Vc2UiPjxzdG9wIHN0b3AtY29sb3I9IiNmZmYiLz48c3RvcCBvZmZzZXQ9IjEiIHN0b3AtY29sb3I9IiNmZmYiIHN0b3Atb3BhY2l0eT0iMCIvPjwvbGluZWFyR3JhZGllbnQ+PC9kZWZzPjxwYXRoIGZpbGw9IiMwMDY1QTkiIGQ9Ik05Ni40NiAxMC44IDc1Ljg2Ljg4QzczLjQ3LS4yNyA3MC42Mi4yMSA2OC43NSAyLjA4TDEuMyA2My41OGMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZsNS41MSA1LjAxYzEuNDkgMS4zNSAzLjcyIDEuNDUgNS4zMi4yNGw4MS4yMy02MS42MmMyLjcyLTIuMDcgNi42NC0uMTMgNi42NCAzLjI5di0uMjRjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M1oiLz48cGF0aCBmaWxsPSIjMDA3QUNDIiBkPSJNOTYuNDYgODkuMiA3NS44NiA5OS4xMmMtMi4zOSAxLjE1LTUuMjQuNjYtNy4xMS0xLjJMMS4zIDM2LjQyYy0xLjgyLTEuNjUtMS44Mi00LjUxIDAtNi4xN2w1LjUxLTUuMDFjMS40OS0xLjM1IDMuNzItMS40NSA1LjMyLS4yM2w4MS4yMyA2MS42MmMyLjcyIDIuMDcgNi42NC4xMyA2LjY0LTMuM3YuMjRjMCAyLjQtMS4zOCA0LjU5LTMuNTQgNS42M1oiLz48cGF0aCBmaWxsPSIjMUY5Q0YwIiBkPSJNNzUuODYgOTkuMTNjLTIuMzkgMS4xNS01LjI0LjY2LTcuMTEtMS4yMSAyLjMxIDIuMzEgNi4yNS42NiA2LjI1LTIuNlY0LjY3YzAtMy4yNi0zLjk0LTQuODktNi4yNS0yLjU4IDEuODctMS44NyA0LjcyLTIuMzYgNy4xMS0xLjIxbDIwLjYgOS45MWMyLjE2IDEuMDQgMy41NCAzLjIzIDMuNTQgNS42M3Y2Ny4xN2MwIDIuNC0xLjM4IDQuNTktMy41NCA1LjYzbC0yMC42IDkuOTFaIi8+PHBhdGggZmlsbD0idXJsKCNhKSIgZmlsbC1ydWxlPSJldmVub2RkIiBkPSJNNzAuODUgOTkuMzJjMS41OC42MSAzLjM3LjU3IDQuOTYtLjE5bDIwLjU5LTkuOTFjMi4xNi0xLjA0IDMuNTQtMy4yMyAzLjU0LTUuNjNWMTYuNDFjMC0yLjQtMS4zOC00LjU5LTMuNTQtNS42M0w3NS44MS44N2MtMi4wOS0xLTQuNTMtLjc2LTYuMzYuNTgtLjI2LjE5LS41MS40LS43NC42NEwyOS4yOSAzOC4wNGwtMTcuMTctMTMuMDNjLTEuNi0xLjIxLTMuODMtMS4xMS01LjMyLjI0bC01LjUgNS4wMWMtMS44MiAxLjY1LTEuODIgNC41MSAwIDYuMTZMMTYuMTkgNTAgMS4zIDYzLjU4Yy0xLjgyIDEuNjUtMS44MiA0LjUxIDAgNi4xN2w1LjUxIDUuMDFjMS40OSAxLjM1IDMuNzIgMS40NSA1LjMyLjIzbDE3LjE3LTEzLjAzIDM5LjQxIDM1Ljk2Yy42Mi42MiAxLjM2IDEuMDkgMi4xNCAxLjRaTTc0Ljk1IDI3LjMgNDUuMDUgNTBsMjkuOSAyMi43VjI3LjNaIiBvcGFjaXR5PSIuMjUiIHN0eWxlPSJtaXgtYmxlbmQtbW9kZTpvdmVybGF5Ii8+PC9zdmc+&logoColor=white)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_as7341.svg)](https://registry.platformio.org/libraries/k0i05/esp_as7341)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_as7341/badge.svg)](https://components.espressif.com/components/k0i05/esp_as7341)

This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the OSRAM AS7341 11-channel multi-spectral digital sensor.  Information on features and functionality are documented and can be found in the `as7341.h` header file and in the `documentation` folder.

This document describes the ESP-IDF component that provides a driver for the AMS/TAOS AS7341 multi-channel spectral sensor. It documents the purpose of the component, wiring and hardware notes, basic usage and examples, and references the component's internal readme.md and header/source files (as7341.h / as7341.c) for details.

For component-level installation and general repository layout please also see the component's readme.md:

- ./readme.md (component-level) — refer to the component folder readme.md for wiring pictures, license and any project-specific notes.

> Note: This file documents the API and high-level usage. Always consult as7341.h for exact function prototypes, types, return codes and additional helper functions.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/i2c/esp_as7341>

## Contents

- Overview
- Features
- Hardware / Wiring
- Requirements
- Configuration (as7341_config_t)
- Integration with ESP-IDF
- Component Package
- Basic Usage Examples
- API Summary (high level — see as7341.h)
- Troubleshooting & Tips
- License & Attribution

## Overview

The AS7341 is an RGB + multi-spectral light sensor that measures multiple narrow-band visible channels and some broadband channels. This component implements an I2C driver to configure the device, control integration/gain, and read spectral channel data.  The driver communicates over an I2C master bus that uses the `i2c_master` API (types like `i2c_master_bus_handle_t` and `i2c_device_config_t` are used by the driver).

This component contains at least:

- as7341.h — public API, data structures and constants
- as7341.c — implementation of the driver (I2C transactions, initialization, read/write helpers)

The driver:

- Initializes the device on an I2C master bus.
- Configures ATIME/ASTEP (integration time) and the spectral gain.
- Manages SMUX configurations (low channels / high channels / flicker detection).
- Reads spectral ADC channel counts (F1..F8, Clear, NIR).
- Converts spectral counts to "basic counts" (float) using integration time and gain.
- Performs flicker-detection measurement and returns a detection state.
- Provides power/SMUX/LED/flicker control helpers.

## Features

- Initialize and configure the AS7341 over I2C
- Control integration time and gain
- Enable/disable measurement channels
- Read spectral channel data (per-channel 16-bit/24-bit values depending on implementation)
- Interrupt (if supported by hardware wiring) or polling based read

## Hardware / Wiring

Typical wiring between ESP32 (ESP32-S3) and AS7341 breakout:

- VCC -> 3.3V (do NOT use 5V unless the breakout explicitly supports it)
- GND -> GND
- SDA -> SDA (I2C data pin on chosen I2C peripheral)
- SCL -> SCL (I2C clock pin on chosen I2C peripheral)
- INT (optional) -> GPIO interrupt pin (if you want to use INT pin for data-ready)

Notes:

- Use proper pull-ups on SDA and SCL if not provided by the breakout (typical values 2.2k–10k).
- Keep I2C wires short to improve reliability.
- Verify voltage levels: ESP32 I/O is 3.3V.

## Requirements

- ESP-IDF environment

- An I2C master bus implementation exposing:
  - i2c_master_bus_handle_t
  - i2c_master_bus_add_device(...)
  - i2c_master_bus_rm_device(...)
  - i2c_master_probe(...)
  - i2c_master_transmit / i2c_master_transmit_receive (the driver calls `i2c_master_transmit` and `i2c_master_transmit_receive` through the `i2c_master` wrapper)

Default I2C settings used by the driver:

- Default I2C address: `I2C_AS7341_DEV_ADDR` (0x39)
- Default clock speed macro: `I2C_AS7341_DEV_CLK_SPD` (100000 Hz)

## Configuration (as7341_config_t)

as7341_config_t (exact fields)

- uint16_t i2c_address;      // I2C device address
- uint32_t i2c_clock_speed;  // I2C SCL clock speed (Hz)
- uint8_t  atime;            // ATIME (integration steps LSB)
- uint16_t astep;            // ASTEP (integration step size)
- as7341_spectral_gains_t spectral_gain; // gain enumerator
- bool     power_enabled;    // if true, power bit is set during setup

Default initializer macro:

- AS7341_CONFIG_DEFAULT expands to:
  - i2c_address = I2C_AS7341_DEV_ADDR
  - i2c_clock_speed = I2C_AS7341_DEV_CLK_SPD
  - spectral_gain = AS7341_SPECTRAL_GAIN_32X
  - power_enabled = true
  - atime = 29
  - astep = 599

## Integration with ESP-IDF

There are two common ways to add the component to an ESP-IDF project:

1. Add the component folder to the `components/` directory of your project (recommended for local development).
   - Project layout:
     - components/esp_as7341/{as7341.c, as7341.h, CMakeLists.txt, readme.md}

2. Use the component as a Git submodule or via the project’s component registry (if applicable).

In your app code add:

```c
#include "as7341.h"
```

Ensure component's CMakeLists.txt or idf_component_register properly exposes the header to the application.

## Component Package

To get started, simply copy the component to your project's `components` folder and reference the `as7341.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_as7341
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── as7341_version.h
    │   └── as7341.h
    └── as7341.c
```

## Basic Usage Examples

Once a driver instance is instantiated the sensor is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings and makes a measurement request from the sensor at user defined interval and prints the results.

```c
#include <as7341.h>

void i2c0_as7341_task( void *pvParameters ) {
    // initialize the xLastWakeTime variable with the current time.
    TickType_t        last_wake_time = xTaskGetTickCount ();
    //
    // initialize i2c device configuration
    as7341_config_t dev_cfg          = I2C_AS7341_CONFIG_DEFAULT;
    as7341_handle_t dev_hdl          = NULL;;
    bool           flicker_completed = false;
    uint8_t           flicker_cycles = 0;
    //
    // init device
    as7341_init(i2c0_bus_hdl, &dev_cfg, &dev_hdl);
    if (dev_hdl == NULL) {
        ESP_LOGE(APP_TAG, "as7341 handle init failed");
        assert(dev_hdl);
    }
    //
    //as7341_disable_led(dev_hdl);
    //
    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## AS7341 - START #########################");
        //
        if(flicker_completed == true) {
            // handle sensor
            as7341_channels_spectral_data_t adc_data;
            esp_err_t result = as7341_get_spectral_measurements(dev_hdl, &adc_data);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "spectral measurement failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGW(APP_TAG, "F1    %d", adc_data.f1);
                ESP_LOGW(APP_TAG, "F2    %d", adc_data.f2);
                ESP_LOGW(APP_TAG, "F3    %d", adc_data.f3);
                ESP_LOGW(APP_TAG, "F4    %d", adc_data.f4);
                ESP_LOGW(APP_TAG, "F5    %d", adc_data.f5);
                ESP_LOGW(APP_TAG, "F6    %d", adc_data.f6);
                ESP_LOGW(APP_TAG, "F7    %d", adc_data.f7);
                ESP_LOGW(APP_TAG, "F8    %d", adc_data.f8);
                ESP_LOGW(APP_TAG, "NIR   %d", adc_data.nir);
                ESP_LOGW(APP_TAG, "CLEAR %d", adc_data.clear);
            }

            as7341_channels_basic_counts_data_t basic_counts_data;
            result = as7341_get_basic_counts(dev_hdl, adc_data, &basic_counts_data);
            if(result != ESP_OK) {
                ESP_LOGE(APP_TAG, "basic counts conversion failed (%s)", esp_err_to_name(result));
            } else {
                ESP_LOGW(APP_TAG, "F1    %f", basic_counts_data.f1);
                ESP_LOGW(APP_TAG, "F2    %f", basic_counts_data.f2);
                ESP_LOGW(APP_TAG, "F3    %f", basic_counts_data.f3);
                ESP_LOGW(APP_TAG, "F4    %f", basic_counts_data.f4);
                ESP_LOGW(APP_TAG, "F5    %f", basic_counts_data.f5);
                ESP_LOGW(APP_TAG, "F6    %f", basic_counts_data.f6);
                ESP_LOGW(APP_TAG, "F7    %f", basic_counts_data.f7);
                ESP_LOGW(APP_TAG, "F8    %f", basic_counts_data.f8);
                ESP_LOGW(APP_TAG, "NIR   %f", basic_counts_data.nir);
                ESP_LOGW(APP_TAG, "CLEAR %f", basic_counts_data.clear);
            }
        } else {
            if(flicker_cycles < 5) {
                as7341_flicker_detection_states_t flicker_state;
                esp_err_t result = as7341_get_flicker_detection_status(dev_hdl, &flicker_state);
                if(result != ESP_OK) {
                    ESP_LOGE(APP_TAG, "flicker detection failed (%s)", esp_err_to_name(result));
                } else {
                    switch(flicker_state) {
                        case AS7341_FLICKER_DETECTION_INVALID:
                            ESP_LOGW(APP_TAG, "Flicker Detection: Invalid");
                            break;
                        case AS7341_FLICKER_DETECTION_UNKNOWN:
                            ESP_LOGW(APP_TAG, "Flicker Detection: Unknown");
                            break;
                        case AS7341_FLICKER_DETECTION_SATURATED:
                            ESP_LOGW(APP_TAG, "Flicker Detection: Saturated");
                            break;
                        case AS7341_FLICKER_DETECTION_100HZ:
                            ESP_LOGW(APP_TAG, "Flicker Detection: 100 Hz");
                            break;
                        case AS7341_FLICKER_DETECTION_120HZ:
                            ESP_LOGW(APP_TAG, "Flicker Detection: 120 Hz");
                            break;
                    }
                }
                as7341_clear_flicker_detection_status_register(dev_hdl);
                ++flicker_cycles;
            } else {
                flicker_completed = true;
            }
        }
        //
        ESP_LOGI(APP_TAG, "######################## AS7341 - END ###########################");
        //
        //
        // pause the task per defined wait period
        vTaskDelaySecUntil( &last_wake_time, I2C0_TASK_SAMPLING_RATE );
    }
    //
    // free resources
    as7341_delete( dev_hdl );
    vTaskDelete( NULL );
}
```

## API Summary (high level)

Refer to as7341.h for the definitive API. Typical functions you will find or expect in the header:

Primary initialization function (exact signature):

- esp_err_t as7341_init(i2c_master_bus_handle_t master_handle,
                        const as7341_config_t *as7341_config,
                        as7341_handle_t *as7341_handle);

Behavior:

- Probes the I2C address on the provided `master_handle` (`i2c_master_probe`).
- Allocates `as7341_device_t` and adds an i2c device to the master bus via `i2c_master_bus_add_device`.
- Configures device registers (ATIME, ASTEP, spectral gain) according to `as7341_config`.
- Returns `ESP_OK` and a handle (opaque `as7341_handle_t`) on success.
- On errors returns the appropriate `esp_err_t` (e.g. `ESP_ERR_NO_MEM`, probe failures, I2C errors).

Cleanup:

- esp_err_t as7341_remove(as7341_handle_t handle);
  - removes the device from its I2C master bus (calls `i2c_master_bus_rm_device`).
- esp_err_t as7341_delete(as7341_handle_t handle);
  - calls `as7341_remove()` and frees the handle memory.

Always rely on the header file for exact prototypes and any return codes (esp_err_t values).

## Power Management & Performance

- Integration time and gain settings control sensitivity and dynamic range. Longer integration and higher gain increase sensitivity but also increase measurement time and risk of saturation.
- The driver may provide blocking read functions and/or interrupt-driven data-ready callbacks. Choose according to your application’s real-time needs.

## Troubleshooting

- No response from device:
  - Confirm wiring and I2C pull-ups.
  - Use an I2C scanner to confirm device address.
  - Check power rails (3.3V) and ground connection.
- Reading zeros or saturated values:
  - Reduce gain or shorten integration time.
- Intermittent communication:
  - Check cable length, use shielded wiring if needed, and confirm I2C clock frequency (try lowering it).

## Testing

- Use a simple application that prints channel values periodically to validate operation.
- Compare results against a reference sensor or known light source if available.

## Reference & Attribution

- Component source files: as7341.c and as7341.h (see these files in the component directory for implementation details and full API).
- Component-level readme: ./readme.md — consult this file for repository/component-specific notes, license, and contribution guidelines.

## License

- See the component readme.md and the header files for license information. Respect the license when reusing or redistributing the code.

## Contributing / Changes

- This file is an additional documentation file for the component and does not change any source files. To improve the documentation, update readme_AS7341.MD or as7341.h/readme.md in the component folder and submit a pull request following the repository contribution guidelines.

Copyright (c) 2025 Eric Gionet (<gionet.c.eric@gmail.com>)
