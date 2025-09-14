cmake_minimum_required(VERSION 3.16.0)

list(APPEND EXTRA_COMPONENT_DIRS

    "components/utilities/sensirion_gas_index_algorithm"
    "components/utilities/esp_kalman_motion" 
    "components/utilities/esp_pressure_tendency" 
    "components/utilities/esp_scalar_trend" 
    "components/utilities/esp_type_utils"
    "components/utilities/esp_uuid" 

    "components/peripherals/adc/esp_s12sd"

    "components/peripherals/i2c/esp_ahtxx"
    "components/peripherals/i2c/esp_ak8975"
    "components/peripherals/i2c/esp_as3935"
    "components/peripherals/i2c/esp_as7341"
    "components/peripherals/i2c/esp_at24cxxx"
    "components/peripherals/i2c/esp_bh1750"
    "components/peripherals/i2c/esp_bme680"
    "components/peripherals/i2c/esp_bmp280"
    "components/peripherals/i2c/esp_bmp390"
    "components/peripherals/i2c/esp_ccs811"
    "components/peripherals/i2c/esp_ens160"
    "components/peripherals/i2c/esp_hdc1080"
    "components/peripherals/i2c/esp_hmc5883l"
    "components/peripherals/i2c/esp_ina226"
    "components/peripherals/i2c/esp_ina228"
    "components/peripherals/i2c/esp_ltr390uv"
    #"components/peripherals/i2c/esp_max30105"
    "components/peripherals/i2c/esp_mlx90614"
    "components/peripherals/i2c/esp_mmc56x3"
    "components/peripherals/i2c/esp_mpu6050"
    "components/peripherals/i2c/esp_pct2075"
    "components/peripherals/i2c/esp_sgp4x"
    "components/peripherals/i2c/esp_sht4x"
    "components/peripherals/i2c/esp_ssd1306"
    "components/peripherals/i2c/esp_tcs3472"
    "components/peripherals/i2c/esp_tlv493d"
    "components/peripherals/i2c/esp_veml6040"
    "components/peripherals/i2c/esp_veml7700"

    "components/peripherals/owb/onewire_bus"
    "components/peripherals/owb/esp_ds18b20"

    "components/peripherals/spi/esp_max31865"

    "components/peripherals/uart/esp_mux4052a"

    "components/schedule/esp_time_into_interval"

    "components/storage/esp_datalogger"
    "components/storage/esp_nvs_ext"

)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Enable if components should be synchronized with platformio and esp-idf registries
idf_build_set_property(COMPONENT_REGISTRY_SYNC "ENABLED") # ENABLED OR DISABLED
idf_build_set_property(COMPONENT_REGISTRY_OWNER "k0i05")  # PlatformIO owner name

project(ESP32-S3_ESP-IDF_COMPONENTS)
