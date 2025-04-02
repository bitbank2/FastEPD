
set(
    app_sources "jpgdec-render.cpp"
    )

idf_component_register(SRCS ${app_sources} 
        REQUIRES
        FastEPD
        esp_wifi driver esp_lcd
        jpegdec
        nvs_flash esp-tls esp_http_client esp_timer mbedtls esp_http_server
        # Directly use the ESP-IDF certificate bundle feature
        REQUIRES mbedtls
        )
