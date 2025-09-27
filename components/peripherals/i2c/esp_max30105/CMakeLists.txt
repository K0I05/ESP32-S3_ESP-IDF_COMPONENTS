idf_component_register(
    SRCS max30105.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c esp_type_utils esp_timer
)
