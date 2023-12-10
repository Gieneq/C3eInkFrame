idf_component_register(SRCS "gcaptive.c" "dns_server.c"
                    INCLUDE_DIRS "include"
                    EMBED_FILES root.html
                    REQUIRES esp_event esp_netif esp_http_server esp_wifi nvs_flash)
