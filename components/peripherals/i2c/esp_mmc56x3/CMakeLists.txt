idf_component_register(
    SRCS mmc56x3.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c_ext esp_common esp_timer
)
