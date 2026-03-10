# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(Seesaw-Demo main.c)


pico_set_program_name(Seesaw-Demo "Seesaw-Demo")
pico_set_program_version(Seesaw-Demo "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Seesaw-Demo 0)
pico_enable_stdio_usb(Seesaw-Demo 1)

# Add the standard library to the build
# target_link_libraries(Seesaw-Demo pico_stdlib)
target_link_libraries(Seesaw-Demo pico_stdlib Seesaw)

# Add the standard include files to the build
target_include_directories(Seesaw-Demo PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Seesaw-Demo
	pico_multicore
	hardware_i2c
	Overclock
)

pico_add_extra_outputs(Seesaw-Demo)
