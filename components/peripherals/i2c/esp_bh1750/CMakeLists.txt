idf_component_register(
    SRCS bh1750.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c esp_type_utils
)
