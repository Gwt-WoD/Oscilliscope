# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(Overclock-Demo main.c)


pico_set_program_name(Overclock-Demo "Overclock-Demo")
pico_set_program_version(Overclock-Demo "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Overclock-Demo 0)
pico_enable_stdio_usb(Overclock-Demo 1)

# Add the standard library to the build
target_link_libraries(Overclock-Demo pico_stdlib)

# Add the standard include files to the build
target_include_directories(Overclock-Demo PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Overclock-Demo
	Overclock
)

pico_add_extra_outputs(Overclock-Demo)
