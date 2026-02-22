
# Define an INTERFACE library (recommended for Pico SDK)
add_library(ws2812 INTERFACE)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(ws2812 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

# Specify the source files for the library
target_sources(ws2812 INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/ws2812.c
)

# Specify the include directories (PUBLIC makes headers available to consumers)
target_include_directories(ws2812 INTERFACE
	${CMAKE_CURRENT_LIST_DIR}
)

# Link any necessary Pico SDK libraries (e.g., pico_stdlib) that your library needs
target_link_libraries(ws2812 INTERFACE
	pico_stdlib
	hardware_pio
)

