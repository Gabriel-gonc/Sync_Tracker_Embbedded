idf_component_register(SRCS "gpio_control.c"
            PRIV_REQUIRES driver esp_timer project_types
            INCLUDE_DIRS "include")