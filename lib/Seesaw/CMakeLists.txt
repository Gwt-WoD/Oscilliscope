
# Define an INTERFACE library (recommended for Pico SDK)
add_library(Seesaw INTERFACE)

# Specify the source files for the library
target_sources(Seesaw INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/seesaw.c
)


# Specify the include directories (PUBLIC makes headers available to consumers)
target_include_directories(Seesaw INTERFACE
	${CMAKE_CURRENT_LIST_DIR}
)

# Link any necessary Pico SDK libraries (e.g., pico_stdlib) that your library needs
target_link_libraries(Seesaw INTERFACE
	pico_stdlib
	hardware_i2c
)

