# SENSOR_BRINGUP=1: HMC5983+AMG8833 serial test on I2C1 (SDA=39,SCL=38).
# Default: BLE bring-up in main.cpp.
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
        SRCS "main.cpp" "mag_plot.cpp" "mag_app.cpp" "mag_fft.cpp"
             "ir_heatmap.cpp" "ir_app.cpp"
        INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include"
        REQUIRES
            driver
            freertos
            log
            esp-dsp
    )
endif()
