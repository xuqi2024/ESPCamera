idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp32-camera nvs_flash esp_wifi esp_timer)