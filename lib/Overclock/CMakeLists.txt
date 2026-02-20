
# Define an INTERFACE library (recommended for Pico SDK)
add_library(Overclock INTERFACE)


# Specify the include directories (PUBLIC makes headers available to consumers)
target_include_directories(Overclock PUBLIC
	${CMAKE_CURRENT_LIST_DIR}
)

# Link any necessary Pico SDK libraries (e.g., pico_stdlib) that your library needs
target_link_libraries(Overclock INTERFACE
	pico_stdlib
)

