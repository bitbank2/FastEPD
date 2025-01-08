
set(
    app_sources "jpgdec-render.cpp"
    )

idf_component_register(SRCS ${app_sources} 
        REQUIRES
        esp_wifi driver esp_lcd
        jpegdec
        nvs_flash esp-tls esp_http_client esp_timer
        # Embed the server root certificate into the final binary
        EMBED_TXTFILES ${project_dir}/ssl_cert/server_cert.pem
        )
