idf_component_register(
    SRCS
    "main.c"
    INCLUDE_DIRS "."
    REQUIRES ieee802154 openthread nvs_flash esp_adc
)
