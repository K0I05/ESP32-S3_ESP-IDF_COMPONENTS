idf_component_register(
    SRCS mlx90614.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c_ext esp_common
)
