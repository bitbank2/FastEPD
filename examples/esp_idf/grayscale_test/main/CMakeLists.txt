
set(
    app_sources "grayscale_test.cpp"
    )

idf_component_register(SRCS ${app_sources} 
    REQUIRES
    esp_wifi driver esp_lcd
    nvs_flash esp-tls esp_http_client esp_timer
    FastEPD #Must be the same name as the git root folder
    )
