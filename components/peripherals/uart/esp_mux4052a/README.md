# MIKROE UART MUX Click Multiplexer

[![K0I05](https://img.shields.io/badge/K0I05-a9a9a9?logo=data:image/svg%2bxml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODgiIGhlaWdodD0iMTg3Ij48cGF0aCBmaWxsPSIjNDU0QjU0IiBkPSJNMTU1LjU1NSAyMS45M2MxOS4yNzMgMTUuOTggMjkuNDcyIDM5LjM0NSAzMi4xNjggNjMuNzg5IDEuOTM3IDIyLjkxOC00LjU1MyA0Ni42Ni0xOC44NDggNjQuNzgxQTUwOS40NzggNTA5LjQ3OCAwIDAgMSAxNjUgMTU1bC0xLjQ4NCAxLjg4M2MtMTMuMTk2IDE2LjUzMS0zNS41NTUgMjcuMjE1LTU2LjMzOSAyOS45MDItMjguMzEyIDIuOC01Mi4yNTUtNC43MzctNzQuNzMyLTIxLjcxNUMxMy4xNzIgMTQ5LjA5IDIuOTczIDEyNS43MjUuMjc3IDEwMS4yODEtMS42NiA3OC4zNjMgNC44MyA1NC42MjEgMTkuMTI1IDM2LjVBNTA5LjQ3OCA1MDkuNDc4IDAgMCAxIDIzIDMybDEuNDg0LTEuODgzQzM3LjY4IDEzLjU4NiA2MC4wNCAyLjkwMiA4MC44MjMuMjE1YzI4LjMxMi0yLjggNTIuMjU1IDQuNzM3IDc0LjczMiAyMS43MTVaIi8+PHBhdGggZmlsbD0iI0ZERkRGRCIgZD0iTTExOS44NjcgNDUuMjdDMTI4LjkzMiA1Mi4yNiAxMzMuODIgNjMgMTM2IDc0Yy42MyA0Ljk3Mi44NDIgOS45NTMuOTUzIDE0Ljk2LjA0NCAxLjkxMS4xMjIgMy44MjIuMjAzIDUuNzMxLjM0IDEyLjIxLjM0IDEyLjIxLTMuMTU2IDE3LjMwOWE5NS42MDQgOTUuNjA0IDAgMCAxLTQuMTg4IDMuNjI1Yy00LjUgMy43MTctNi45NzQgNy42ODgtOS43MTcgMTIuODAzQzEwNi45NCAxNTIuNzkyIDEwNi45NCAxNTIuNzkyIDk3IDE1N2MtMy40MjMuNTkyLTUuODAxLjY4NS04Ljg3OS0xLjA3NC05LjgyNi03Ljg4LTE2LjAzNi0xOS41OS0yMS44NTgtMzAuNTEyLTIuNTM0LTQuNTc1LTUuMDA2LTcuMjEtOS40NjYtMTAuMDItMy43MTQtMi44ODItNS40NS02Ljk4Ni02Ljc5Ny0xMS4zOTQtLjU1LTQuODg5LS41NjEtOS4zMTYgMS0xNCAuMDkzLTEuNzYzLjE4Mi0zLjUyNy4yMzktNS4yOTIuNDkxLTEzLjg4NCAzLjg2Ni0yNy4wNTcgMTQuMTU2LTM3LjAyOCAxNy4yMTgtMTQuMzM2IDM1Ljg1OC0xNS4wNjYgNTQuNDcyLTIuNDFaIi8+PHBhdGggZmlsbD0iI0M2RDVFMCIgZD0iTTEwOSAzOWMxMS43MDMgNS4yNTUgMTkuMjA2IDEzLjE4NiAyNC4yOTMgMjUuMDA0IDIuODU3IDguMjQgMy40NyAxNi4zMTYgMy42NiAyNC45NTYuMDQ0IDEuOTExLjEyMiAzLjgyMi4yMDMgNS43MzEuMzQgMTIuMjEuMzQgMTIuMjEtMy4xNTYgMTcuMzA5YTk1LjYwNCA5NS42MDQgMCAwIDEtNC4xODggMy42MjVjLTQuNSAzLjcxNy02Ljk3NCA3LjY4OC05LjcxNyAxMi44MDNDMTA2LjgwNCAxNTMuMDQxIDEwNi44MDQgMTUzLjA0MSA5NyAxNTdjLTIuMzMyLjA3OC00LjY2OC4wOS03IDBsMi4xMjUtMS44NzVjNS40My01LjQ0NSA4Ljc0NC0xMi41NzcgMTEuNzU0LTE5LjU1OWEzNDkuNzc1IDM0OS43NzUgMCAwIDEgNC40OTYtOS44NzlsMS42NDgtMy41NWMyLjI0LTMuNTU1IDQuNDEtNC45OTYgNy45NzctNy4xMzcgMi4zMjMtMi42MSAyLjMyMy0yLjYxIDQtNWwtMyAxYy0yLjY4LjE0OC01LjMxOS4yMy04IC4yNWwtMi4xOTUuMDYzYy01LjI4Ny4wMzktNS4yODcuMDM5LTcuNzc4LTEuNjUzLTEuNjY2LTIuNjkyLTEuNDUzLTQuNTYtMS4wMjctNy42NiAyLjM5NS00LjM2MiA0LjkyNC04LjA0IDkuODI4LTkuNTcgMi4zNjQtLjQ2OCA0LjUxNC0uNTI4IDYuOTIyLS40OTNsMi40MjIuMDI4TDEyMSA5MmwtMS0yYTkyLjc1OCA5Mi43NTggMCAwIDEtLjM2LTQuNTg2QzExOC42IDY5LjYzMiAxMTYuNTE3IDU2LjA5NCAxMDQgNDVjLTUuOTA0LTQuNjY0LTExLjYtNi4wODgtMTktNyA3LjU5NC00LjI2NCAxNi4yMjMtMS44MSAyNCAxWiIvPjxwYXRoIGZpbGw9IiM0OTUwNTgiIGQ9Ik03NyA5MmM0LjYxMyAxLjY3MSA3LjI2IDMuOTQ1IDEwLjA2MyA3LjkzOCAxLjA3OCAzLjUyMy45NzYgNS41NDYtLjA2MyA5LjA2Mi0yLjk4NCAyLjk4NC02LjI1NiAyLjM2OC0xMC4yNSAyLjM3NWwtMi4yNzcuMDc0Yy01LjI5OC4wMjgtOC4yNTQtLjk4My0xMi40NzMtNC40NDktMi44MjYtMy41OTctMi40MTYtNy42MzQtMi0xMiA0LjUwMi00LjcyOCAxMC45OS0zLjc2IDE3LTNaIi8+PHBhdGggZmlsbD0iIzQ4NEY1NyIgZD0ibTExOCA5MS43NSAzLjEyNS0uMDc4YzMuMjU0LjM3MSA0LjU5NyAxLjAwMiA2Ljg3NSAzLjMyOC42MzkgNC4yMzEuMjkgNi40NDItMS42ODggMTAuMjUtMy40MjggNC4wNzgtNS44MjcgNS41OTgtMTEuMTk1IDYuMTQ4LTEuNDE0LjAwOC0yLjgyOCAwLTQuMjQyLS4wMjNsLTIuMTY4LjAzNWMtMi45OTgtLjAxNy01LjE1Ny0uMDMzLTcuNjcyLTEuNzU4LTEuNjgxLTIuNjg0LTEuNDYtNC41NTItMS4wMzUtNy42NTIgMi4zNzUtNC4zMjUgNC44OTQtOC4wMDkgOS43NS05LjU1OSAyLjc3Ny0uNTQ0IDUuNDItLjY0OSA4LjI1LS42OTFaIi8+PHBhdGggZmlsbD0iIzUyNTg2MCIgZD0iTTg2IDEzNGgxNmwxIDRjLTIgMi0yIDItNS4xODggMi4yNjZMOTQgMTQwLjI1bC0zLjgxMy4wMTZDODcgMTQwIDg3IDE0MCA4NSAxMzhsMS00WiIvPjwvc3ZnPg==)](https://github.com/K0I05)
[![License: MIT](https://cdn.prod.website-files.com/5e0f1144930a8bc8aace526c/65dd9eb5aaca434fac4f1c34_License-MIT-blue.svg)](/LICENSE)
[![Language](https://img.shields.io/badge/Language-C-navy.svg)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Framework](https://img.shields.io/badge/Framework-ESP_IDF-red.svg)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html)
[![Edited with VS Code](https://badgen.net/badge/icon/VS%20Code?icon=visualstudio&label=edited%20with)](https://code.visualstudio.com/)
[![Build with PlatformIO](https://img.shields.io/badge/build%20with-PlatformIO-orange?logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMjUwMCIgaGVpZ2h0PSIyNTAwIiB2aWV3Qm94PSIwIDAgMjU2IDI1NiIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIiBwcmVzZXJ2ZUFzcGVjdFJhdGlvPSJ4TWlkWU1pZCI+PHBhdGggZD0iTTEyOCAwQzkzLjgxIDAgNjEuNjY2IDEzLjMxNCAzNy40OSAzNy40OSAxMy4zMTQgNjEuNjY2IDAgOTMuODEgMCAxMjhjMCAzNC4xOSAxMy4zMTQgNjYuMzM0IDM3LjQ5IDkwLjUxQzYxLjY2NiAyNDIuNjg2IDkzLjgxIDI1NiAxMjggMjU2YzM0LjE5IDAgNjYuMzM0LTEzLjMxNCA5MC41MS0zNy40OUMyNDIuNjg2IDE5NC4zMzQgMjU2IDE2Mi4xOSAyNTYgMTI4YzAtMzQuMTktMTMuMzE0LTY2LjMzNC0zNy40OS05MC41MUMxOTQuMzM0IDEzLjMxNCAxNjIuMTkgMCAxMjggMCIgZmlsbD0iI0ZGN0YwMCIvPjxwYXRoIGQ9Ik0yNDkuMzg2IDEyOGMwIDY3LjA0LTU0LjM0NyAxMjEuMzg2LTEyMS4zODYgMTIxLjM4NkM2MC45NiAyNDkuMzg2IDYuNjEzIDE5NS4wNCA2LjYxMyAxMjggNi42MTMgNjAuOTYgNjAuOTYgNi42MTQgMTI4IDYuNjE0YzY3LjA0IDAgMTIxLjM4NiA1NC4zNDYgMTIxLjM4NiAxMjEuMzg2IiBmaWxsPSIjRkZGIi8+PHBhdGggZD0iTTE2MC44NjkgNzQuMDYybDUuMTQ1LTE4LjUzN2M1LjI2NC0uNDcgOS4zOTItNC44ODYgOS4zOTItMTAuMjczIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzJzLTEwLjMyIDQuNjItMTAuMzIgMTAuMzJjMCAzLjc1NSAyLjAxMyA3LjAzIDUuMDEgOC44MzdsLTUuMDUgMTguMTk1Yy0xNC40MzctMy42Ny0yNi42MjUtMy4zOS0yNi42MjUtMy4zOWwtMi4yNTggMS4wMXYxNDAuODcybDIuMjU4Ljc1M2MxMy42MTQgMCA3My4xNzctNDEuMTMzIDczLjMyMy04NS4yNyAwLTMxLjYyNC0yMS4wMjMtNDUuODI1LTQwLjU1NS01Mi4xOTd6TTE0Ni41MyAxNjQuOGMtMTEuNjE3LTE4LjU1Ny02LjcwNi02MS43NTEgMjMuNjQzLTY3LjkyNSA4LjMyLTEuMzMzIDE4LjUwOSA0LjEzNCAyMS41MSAxNi4yNzkgNy41ODIgMjUuNzY2LTM3LjAxNSA2MS44NDUtNDUuMTUzIDUxLjY0NnptMTguMjE2LTM5Ljc1MmE5LjM5OSA5LjM5OSAwIDAgMC05LjM5OSA5LjM5OSA5LjM5OSA5LjM5OSAwIDAgMCA5LjQgOS4zOTkgOS4zOTkgOS4zOTkgMCAwIDAgOS4zOTgtOS40IDkuMzk5IDkuMzk5IDAgMCAwLTkuMzk5LTkuMzk4em0yLjgxIDguNjcyYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDkgMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OXoiIGZpbGw9IiNFNTcyMDAiLz48cGF0aCBkPSJNMTAxLjM3MSA3Mi43MDlsLTUuMDIzLTE4LjkwMWMyLjg3NC0xLjgzMiA0Ljc4Ni01LjA0IDQuNzg2LTguNzAxIDAtNS43LTQuNjItMTAuMzItMTAuMzItMTAuMzItNS42OTkgMC0xMC4zMTkgNC42Mi0xMC4zMTkgMTAuMzIgMCA1LjY4MiA0LjU5MiAxMC4yODkgMTAuMjY3IDEwLjMxN0w5NS44IDc0LjM3OGMtMTkuNjA5IDYuNTEtNDAuODg1IDIwLjc0Mi00MC44ODUgNTEuODguNDM2IDQ1LjAxIDU5LjU3MiA4NS4yNjcgNzMuMTg2IDg1LjI2N1Y2OC44OTJzLTEyLjI1Mi0uMDYyLTI2LjcyOSAzLjgxN3ptMTAuMzk1IDkyLjA5Yy04LjEzOCAxMC4yLTUyLjczNS0yNS44OC00NS4xNTQtNTEuNjQ1IDMuMDAyLTEyLjE0NSAxMy4xOS0xNy42MTIgMjEuNTExLTE2LjI4IDMwLjM1IDYuMTc1IDM1LjI2IDQ5LjM2OSAyMy42NDMgNjcuOTI2em0tMTguODItMzkuNDZhOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTkgOS4zOTggOS4zOTkgOS4zOTkgMCAwIDAgOS40IDkuNCA5LjM5OSA5LjM5OSAwIDAgMCA5LjM5OC05LjQgOS4zOTkgOS4zOTkgMCAwIDAtOS4zOTktOS4zOTl6bS0yLjgxIDguNjcxYTIuMzc0IDIuMzc0IDAgMSAxIDAtNC43NDggMi4zNzQgMi4zNzQgMCAwIDEgMCA0Ljc0OHoiIGZpbGw9IiNGRjdGMDAiLz48L3N2Zz4=)](https://platformio.org/)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/k0i05/library/esp_mux4052a.svg)](https://registry.platformio.org/libraries/k0i05/esp_mux4052a)
[![ESP Component Registry](https://components.espressif.com/components/k0i05/esp_mux4052a/badge.svg)](https://components.espressif.com/components/k0i05/esp_mux4052a)

This ESP32 espressif IoT development framework (esp-idf) i2c peripheral driver was developed for the MUX4052A uart port multiplexer.  The 4052A denotes the multiplexer type (SN74LV4052A) and number of channels supported when utilized for serial applications.  This component was developed using the MIKROE UAR MUX Click breakout board which utlizes the SN74LV4052A chip by Texas Instruments.  Information on features and functionality are documented and can be found in the `mux4052a.h` header file and in the `documentation` folder.

## Repository

The component is hosted on github and is located here: <https://github.com/K0I05/ESP32-S3_ESP-IDF_COMPONENTS/tree/main/components/peripherals/uart/esp_mux4052a>

## General Usage

To get started, simply copy the component to your project's `components` folder and reference the `mux4052a.h` header file as an include.  The component includes documentation for the peripheral such as the datasheet, application notes, and/or user manual where applicable.

```text
components
└── esp_mux4052a
    ├── CMakeLists.txt
    ├── README.md
    ├── LICENSE
    ├── idf_component.yml
    ├── library.json
    ├── documentation
    │   └── datasheets, etc.
    ├── include
    │   └── mux4052a_version.h
    │   └── mux4052a.h
    └── mux4052a.c
```

## Basic Example

Once a driver instance is instantiated the multiplexer is ready for usage as shown in the below example.   This basic implementation of the driver utilizes default configuration settings, configures the MUX4052A's channel, transmits a message from UART_NUM_2 at a user defined interval, and prints the message received from UART_NUM_0.  The MUX4052A is interfaced to the MCU on UART_NUM_2 and MUX4052A channels 0 and 2 are interfaced to the MCU on UART_NUM_0.  The uart transmit lines for channels 0 and 2 on the MUX4052A are connected to the MCU's receive lines on UART_NUM_0.  There is a 5 second delay between channel configuration and message transmissions.

```c
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <esp_log.h>

#include <nvs.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <mux4052a.h>

#include <driver/uart.h>
#include <driver/gpio.h>

#define UART_0_TASK_NAME            "uart_0_rx_tsk"
#define UART_0_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE * 4)
#define UART_0_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)

#define UART_2_TASK_NAME            "uart_2_tx_tsk"
#define UART_2_TASK_STACK_SIZE      (configMINIMAL_STACK_SIZE * 4)
#define UART_2_TASK_PRIORITY        (tskIDLE_PRIORITY + 2)

#define UART_0_PORT_NUM             (UART_NUM_0)
#define UART_0_TXD_IO_NUM           (GPIO_NUM_1)
#define UART_0_RXD_IO_NUM           (GPIO_NUM_3)

#define UART_RX_BUF_SIZE            (1024)

#define APP_TAG                     "MUX4052A [APP]"


static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

/**
 * @brief Initializes UART 0.
 */
static inline void init_uart( void ) {
    // configuration for UART 1 and 2
    const uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // UART0 - tx
    uart_driver_install(UART_0_PORT_NUM, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_0_PORT_NUM, &uart_config);
    uart_set_pin(UART_0_PORT_NUM, UART_0_TXD_IO_NUM, UART_0_RXD_IO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief Transmits data from UART 2.
 * 
 * @param log_name Log name for ESP logging.
 * @param data Data to transmit from UART 2.
 * @return int Numbers of bytes transmitted from UART 2.
 */
static inline int uart_2_tx_data(const char* log_name, const char* data) {
    const int len = strlen(data);
    const int tx_bytes = uart_write_bytes(MUX4052A_PORT_NUM, data, len);
    ESP_LOGI(log_name, "Wrote %d bytes", tx_bytes);
    return tx_bytes;
}

static void uart_2_tx_task( void *pvParameters ) {
    static const char *TX_TASK_TAG = "UART_1_TX_TASK";
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    
    mux4052a_config_t mux_cfg = MUX4052A_CONFIG_DEFAULT;
    mux4052a_handle_t mux_hdl = NULL;

    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    
    /* attempt to instantiate mux handle */
    mux4052a_init(&mux_cfg, &mux_hdl);
    if(mux_hdl == NULL) {
        ESP_LOGE(APP_TAG, "mux4052a_init failed");
        esp_restart();
    }

    // task loop entry point
    for ( ;; ) {

        mux4052a_set_channel(mux_hdl, MUX4052A_CHANNEL_0);
        ESP_LOGI(TX_TASK_TAG, "Set MUX4052A to channel 0");
        uart_2_tx_data(TX_TASK_TAG, "Hello World via MUX4052A channel 0!");

        vTaskDelay(5000 / portTICK_PERIOD_MS);

        mux4052a_set_channel(mux_hdl, MUX4052A_CHANNEL_2);
        ESP_LOGI(TX_TASK_TAG, "Set MUX4052A to channel 2");
        uart_2_tx_data(TX_TASK_TAG, "Hello World via MUX4052A channel 2!");

        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 10 );
    }

    // free up task resources and remove task from stack
    mux4052a_delete( mux_hdl );
    vTaskDelete( NULL );
}

static void uart_0_rx_task(void *pv_parameters) {
    static const char *RX_TASK_TAG = "UART_0_RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*)malloc(UART_RX_BUF_SIZE + 1);
    while (1) {
        const int rx_bytes = uart_read_bytes(UART_0_PORT_NUM, data, UART_RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rx_bytes > 0) {
            data[rx_bytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rx_bytes, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rx_bytes, ESP_LOG_INFO);
            uart_flush(UART_0_PORT_NUM);
        }
    }
    free(data);
    vTaskDelete(NULL);
}

void app_main( void ) {
    ESP_LOGI(APP_TAG, "Startup..");
    ESP_LOGI(APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(APP_TAG, "IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(APP_TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    init_uart();
    
    xTaskCreatePinnedToCore( 
        uart_2_tx_task, 
        UART_2_TASK_NAME, 
        UART_2_TASK_STACK_SIZE, 
        NULL, 
        UART_2_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );

    xTaskCreatePinnedToCore( 
        uart_0_rx_task, 
        UART_0_TASK_NAME, 
        UART_0_TASK_STACK_SIZE, 
        NULL, 
        UART_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
}
```

```text
I (21100) UART_1_TX_TASK: Set MUX4052A to channel 0
I (21100) UART_1_TX_TASK: Wrote 35 bytes
I (22100) UART_0_RX_TASK: Read 35 bytes: 'Hello World via MUX4052A channel 0!'
I (22100) UART_0_RX_TASK: 0x3fca70a4   48 65 6c 6c 6f 20 57 6f  72 6c 64 20 76 69 61 20  |Hello World via |
I (22110) UART_0_RX_TASK: 0x3fca70b4   4d 55 58 34 30 35 32 41  20 63 68 61 6e 6e 65 6c  |MUX4052A channel|
I (22110) UART_0_RX_TASK: 0x3fca70c4   20 30 21                                          | 0!|
I (26100) UART_1_TX_TASK: Set MUX4052A to channel 2
I (26100) UART_1_TX_TASK: Wrote 35 bytes
I (27100) UART_0_RX_TASK: Read 35 bytes: 'Hello World via MUX4052A channel 2!'
I (27100) UART_0_RX_TASK: 0x3fca70a4   48 65 6c 6c 6f 20 57 6f  72 6c 64 20 76 69 61 20  |Hello World via |
I (27110) UART_0_RX_TASK: 0x3fca70b4   4d 55 58 34 30 35 32 41  20 63 68 61 6e 6e 65 6c  |MUX4052A channel|
I (27110) UART_0_RX_TASK: 0x3fca70c4   20 32 21                                          | 2!|
```

Copyright (c) 2024 Eric Gionet (<gionet.c.eric@gmail.com>)
