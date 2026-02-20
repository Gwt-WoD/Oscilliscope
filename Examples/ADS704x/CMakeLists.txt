# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
# add_executable(ADS704x-Demo main.c dma_test2.c)
add_executable(ADS704x-Demo main.c)


pico_set_program_name(ADS704x-Demo "ADS704x-Demo")
pico_set_program_version(ADS704x-Demo "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ADS704x-Demo 0)
pico_enable_stdio_usb(ADS704x-Demo 1)

# Add the standard library to the build
# target_link_libraries(ADS704x-Demo pico_stdlib)
# target_link_libraries(ADS704x-Demo pico_stdlib ADS704x)
target_link_libraries(ADS704x-Demo pico_stdlib ADS704x_DMA)

# Add the standard include files to the build
target_include_directories(ADS704x-Demo PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(ADS704x-Demo
	pico_multicore
	hardware_dma
	hardware_irq
	hardware_spi
	# hardware_timer
	Overclock
)

pico_add_extra_outputs(ADS704x-Demo)
