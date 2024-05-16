set(SOURCES main.cpp)
idf_component_register(SRCS "esp32_serial_transport.c" "main.cpp"
                    INCLUDE_DIRS "")
