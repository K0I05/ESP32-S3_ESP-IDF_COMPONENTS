#!/usr/bin/env python
# Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
# Released under the MIT License (MIT) - see LICENSE file

from component import Component

### Local Configuration ###
component_working_path = "C:\\Users\\lavco\\OneDrive\\Documents\\PlatformIO\\Projects\\ESP32-S3_ESP-IDF_COMPONENTS\\components"
component_archive_path = "C:\\Espressif\\component_archive"


### ESP Components Registry ###
### PlatformIO Components Registry ###
pio_owner = "k0i05"

### Components ###
# peripheral/adc/[component_name]
# peripheral/i2c/[component_name]
# peripheral/owb/[component_name]
# peripheral/spi/[component_name]
# peripheral/uart/[component_name]
# schedule/[component_name]
# storage/[component_name]
# utilities/[component_name]

# instantiate component list to configure which components will be processed.
component_list = [ 
                  ### Utilities ###
                  # pressure tendency
                  Component(name="esp_pressure_tendency", header_name="pressure_tendency", relative_path="utilities\\esp_pressure_tendency"),
                  # type utilities
                  Component(name="esp_type_utils", header_name="type_utils", relative_path="utilities\\esp_type_utils"),
                  # uuid
                  Component(name="esp_uuid", header_name="uuid", relative_path="utilities\\esp_uuid"),
                  # cla
                  Component(name="esp_cla", header_name="cla", relative_path="utilities\\esp_cla"),
                  
                  ### Peripherals ###
                  # adc peripherals 
                  Component(name="esp_s12sd", header_name="s12sd", relative_path="peripherals\\adc\\esp_s12sd"),
                  # i2c peripherals
                  Component(name="esp_ahtxx", header_name="ahtxx", relative_path="peripherals\\i2c\\esp_ahtxx"),
                  Component(name="esp_ak8975", header_name="ak8975", relative_path="peripherals\\i2c\\esp_ak8975"),
                  Component(name="esp_as3935", header_name="as3935", relative_path="peripherals\\i2c\\esp_as3935"),
                  Component(name="esp_as7341", header_name="as7341", relative_path="peripherals\\i2c\\esp_as7341"),
                  Component(name="esp_bh1750", header_name="bh1750", relative_path="peripherals\\i2c\\esp_bh1750"),
                  Component(name="esp_bme680", header_name="bme680", relative_path="peripherals\\i2c\\esp_bme680"),
                  Component(name="esp_bmp280", header_name="bmp280", relative_path="peripherals\\i2c\\esp_bmp280"),
                  Component(name="esp_bmp390", header_name="bmp390", relative_path="peripherals\\i2c\\esp_bmp390"),
                  Component(name="esp_ccs811", header_name="ccs811", relative_path="peripherals\\i2c\\esp_ccs811"),
                  Component(name="esp_ens160", header_name="ens160", relative_path="peripherals\\i2c\\esp_ens160"),
                  Component(name="esp_hdc1080", header_name="hdc1080", relative_path="peripherals\\i2c\\esp_hdc1080"),
                  Component(name="esp_ina226", header_name="ina226", relative_path="peripherals\\i2c\\esp_ina226"),
                  Component(name="esp_ltr390uv", header_name="ltr390uv", relative_path="peripherals\\i2c\\esp_ltr390uv"),
                  Component(name="esp_max30105", header_name="max30105", relative_path="peripherals\\i2c\\esp_max30105"),
                  Component(name="esp_mlx90614", header_name="mlx90614", relative_path="peripherals\\i2c\\esp_mlx90614"),
                  Component(name="esp_mmc56x3", header_name="mmc56x3", relative_path="peripherals\\i2c\\esp_mmc56x3"),
                  Component(name="esp_mpu6050", header_name="mpu6050", relative_path="peripherals\\i2c\\esp_mpu6050"),
                  Component(name="esp_pct2075", header_name="pct2075", relative_path="peripherals\\i2c\\esp_pct2075"),
                  Component(name="esp_sgp4x", header_name="sgp4x", relative_path="peripherals\\i2c\\esp_sgp4x"),
                  Component(name="esp_sht4x", header_name="sht4x", relative_path="peripherals\\i2c\\esp_sht4x"),
                  Component(name="esp_tcs3472", header_name="tcs3472", relative_path="peripherals\\i2c\\esp_tcs3472"),
                  Component(name="esp_ssd1306", header_name="ssd1306", relative_path="peripherals\\i2c\\esp_ssd1306"),
                  Component(name="esp_veml6040", header_name="veml6040", relative_path="peripherals\\i2c\\esp_veml6040"),
                  Component(name="esp_veml7700", header_name="veml7700", relative_path="peripherals\\i2c\\esp_veml7700"),
                  # owb peripherals
                  Component(name="esp_ds18b20", header_name="ds18b20", relative_path="peripherals\\owb\\esp_ds18b20"),
                  # uart peripherals
                  Component(name="esp_mux4052a", header_name="mux4052a", relative_path="peripherals\\uart\\esp_mux4052a"),
                  
                  ### Schedule ###
                  # time-into-interval
                  Component(name="esp_time_into_interval", header_name="time_into_interval", relative_path="schedule\\esp_time_into_interval"),
                  
                  ### Storage ###
                  # nvs ext
                  Component(name="esp_nvs_ext", header_name="nvs_ext", relative_path="storage\\esp_nvs_ext"),
                  # datalogger
                  Component(name="esp_datalogger", header_name="datatable", relative_path="storage\\esp_datalogger")
                ]
"""List of components to process"""
