cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(pico_sdk_import.cmake)

project(ProjetoIntegrador C CXX ASM)

pico_sdk_init()


add_executable(ProjetoIntegrador ProjetoIntegrador.c inc/ssd1306.c)


# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ProjetoIntegrador 1)
pico_enable_stdio_usb(ProjetoIntegrador 1)

pico_generate_pio_header(ProjetoIntegrador ${CMAKE_CURRENT_LIST_DIR}/pio_matrix.pio)


# Add the standard library to the build
target_link_libraries(ProjetoIntegrador
    pico_stdlib hardware_adc hardware_pwm hardware_i2c hardware_pio)

# Add the standard include files to the build
target_include_directories(ProjetoIntegrador PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(ProjetoIntegrador)

