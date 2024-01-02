idf_component_register(SRCS  "gcaptive.c" "dns_server.c"
                    INCLUDE_DIRS "include"
                    EMBED_FILES esp32c3.png favicon.ico gframe_icon.png index.html scripts.js styles.css 
                    REQUIRES esp_event esp_netif esp_http_server esp_wifi nvs_flash)