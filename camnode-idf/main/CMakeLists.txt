idf_component_register(
    SRCS "main.cpp" "settings.cpp" "camera.cpp" "sdcard.cpp" "avi.cpp"
         "recorder.cpp" "motion.cpp" "timelapse.cpp" "http_api.cpp" "http_files.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi esp_http_server nvs_flash fatfs sdmmc esp_driver_sdmmc
             esp_timer app_update esp_psram esp_netif esp_event vfs driver)
