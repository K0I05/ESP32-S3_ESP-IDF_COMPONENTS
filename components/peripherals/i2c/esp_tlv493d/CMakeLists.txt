idf_component_register(
    SRCS tlv493d.c
    INCLUDE_DIRS include
    REQUIRES esp_driver_i2c esp_type_utils esp_timer
)
