
# Define an INTERFACE library (recommended for Pico SDK)
add_library(ADS704x INTERFACE)

# Specify the source files for the library
target_sources(ADS704x INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/ads7043.c
)


# Specify the include directories (PUBLIC makes headers available to consumers)
target_include_directories(ADS704x INTERFACE
	${CMAKE_CURRENT_LIST_DIR}
)

# Link any necessary Pico SDK libraries (e.g., pico_stdlib) that your library needs
target_link_libraries(ADS704x INTERFACE
	pico_stdlib
	hardware_spi
	hardware_dma
	hardware_irq
)

#=====================================================================================

# Define an INTERFACE library (recommended for Pico SDK)
add_library(ADS704x_DMA INTERFACE)

# Specify the source files for the library
target_sources(ADS704x_DMA INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/ads704x_dma.c
	${CMAKE_CURRENT_LIST_DIR}/ads704x_dma_presets.c
)


# Specify the include directories (PUBLIC makes headers available to consumers)
target_include_directories(ADS704x_DMA INTERFACE
	${CMAKE_CURRENT_LIST_DIR}
)

# Link any necessary Pico SDK libraries (e.g., pico_stdlib) that your library needs
target_link_libraries(ADS704x_DMA INTERFACE
	pico_stdlib
	hardware_spi
	hardware_dma
	hardware_irq
)