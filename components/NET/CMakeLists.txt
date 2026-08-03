set(src_dirs
        WIFI
        AI
        ASR
)

set(include_dirs
        WIFI
        AI
        ASR
)

set(requires_dirs
            driver
            esp_wifi
            esp_event
            esp_netif
            nvs_flash
            esp_timer
            Core
            MYHAL
            BSP
            esp_http_client
            esp_websocket_client
            json
            mbedtls
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires_dirs})
