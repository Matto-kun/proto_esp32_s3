idf_component_register(
    SRCS "main.cpp" "mag_plot.cpp" "mag_app.cpp"
    INCLUDE_DIRS "." "${CMAKE_SOURCE_DIR}/include"
    REQUIRES
        driver
        freertos
        log
)
