idf_component_register(
    SRCS bmp280.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c_ext esp_common esp_timer
)
