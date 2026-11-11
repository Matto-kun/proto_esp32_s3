# SENSOR_BRINGUP=1: HMC5983+AMG8833 serial test on I2C0 (SDA=39,SCL=38).
# Default: HMC5883L logger (microSD + Wi-Fi SoftAP).
if("$ENV{SENSOR_BRINGUP}" STREQUAL "1")
    idf_component_register(
        SRCS "sensor_bringup_main.cpp"
        INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include"
        REQUIRES
            driver
            freertos
            log
    )
else()
    idf_component_register(
        SRCS "main.cpp" "app_mag.cpp" "app_sd_log.cpp" "app_wifi.cpp"
             "app_cbor.cpp" "mag_fft.cpp"
        INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include"
        REQUIRES
            driver
            freertos
            log
            esp-dsp
            esp_wifi
            esp_event
            esp_netif
            nvs_flash
            esp_timer
            lwip
    )
endif()
