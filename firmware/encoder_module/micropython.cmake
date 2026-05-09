add_library(usermod_encoder INTERFACE)

pico_generate_pio_header(usermod_encoder
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

target_sources(usermod_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/modencoder.c
    ${CMAKE_CURRENT_LIST_DIR}/encoder.c
)

target_include_directories(usermod_encoder INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod_encoder INTERFACE
    pico_stdlib
    hardware_pio
    hardware_gpio
    pico_time
)
target_link_libraries(usermod INTERFACE usermod_encoder)