idf_component_register(SRCS "hello_world_main.c"
                    INCLUDE_DIRS ""
                    REQUIRES spi_epd nvs_flash esp_wifi shtc3_sensor gindicator gcaptive ap_portal)
