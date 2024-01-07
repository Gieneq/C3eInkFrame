idf_component_register(SRCS "epaper_frame_main.c"
                    INCLUDE_DIRS ""
                    REQUIRES spi_epd_7in5v2 nvs_flash esp_wifi shtc3_sensor gindicator gcaptive ap_portal gframe gstorage)
