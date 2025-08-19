add_library(bh1750_light_sensor STATIC
    bh1750_light_sensor.c
)

target_include_directories(bh1750_light_sensor PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(bh1750_light_sensor
    pico_stdlib
    hardware_i2c
)
