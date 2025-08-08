idf_component_register(SRCS "wifi_acess_point.c"
            PRIV_REQUIRES esp_wifi nvs_flash project_types
            INCLUDE_DIRS "include")