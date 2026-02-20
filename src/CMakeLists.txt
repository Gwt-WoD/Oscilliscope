# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1
add_executable(Oscilloscope
	${CMAKE_CURRENT_LIST_DIR}/main.c
	# Add more c files here...
)

pico_set_program_name(Oscilloscope "Oscilloscope")
pico_set_program_version(Oscilloscope "0.1")

# Generate PIO header
pico_generate_pio_header(Oscilloscope ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Oscilloscope 1)
pico_enable_stdio_usb(Oscilloscope 1)

# Add the standard library to the build
target_link_libraries(Oscilloscope pico_stdlib)

# Add the standard include files to the build
target_include_directories(Oscilloscope PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(Oscilloscope
	# Pico
	pico_multicore
	# Pico Hardware
	hardware_spi
	hardware_pio
	hardware_dma
	# ADC
	# ADS704x
	ADS704x_DMA
	# Display
	ili9341
	gfx
	# User Input
	Seesaw
	# Utility
	Overclock
)

pico_add_extra_outputs(Oscilloscope)
