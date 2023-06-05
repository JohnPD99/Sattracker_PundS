cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(Tracker C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME}
    main.c
    MPU_6050/mpu_6050.c
    HMC5883L/hmc5883l.c
)

target_include_directories(
    ${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/MPU_6050
    ${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/HMC5883L
)



pico_sdk_init()

target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
)

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${PROJECT_NAME})



