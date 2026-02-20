/**
 * @file ads7043.c
 * @brief ADS7043 ADC driver implementation.
 * 
 * This file contains functions to initialize and read data from the ADS7043
 * analog-to-digital converter (ADC) using SPI communication.
 * 
 * @author Michael MacDonald
 */

/**
 * TODO:
 * - Implement buffer to hold samples
 * - Setup timer interrupt for each screen refresh acquisition
 * - Setup DMA or interrupt to read samples at precise intervals
 * - Setup a DMA channel triggered by the timer to read samples into buffer
 */


/**
 * I wonder if we can alternate the CS lines of the two ADS7043s
 * such that we can remove the two zero bits between samples and
 * bitpack the data from both ADCs??
 */


#include "ads7043.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/spi.h"



// ADS7043 initialization
void ads7043_init(ads7043_inst_t *inst) {
	// Initialize SPI
	spi_init(inst->spi, inst->spi_baudrate);

	// Set SPI format: 8 bits per transfer, CPOL=0, CPHA=0, MSB first
	// spi_set_format(inst->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
	spi_set_format(inst->spi, 14, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);


	// Initialize GPIO pins for SPI
	gpio_set_function(inst->sck_pin, GPIO_FUNC_SPI);
	gpio_set_function(inst->miso_pin, GPIO_FUNC_SPI);
}

void ads7043_selfcal(ads7043_inst_t *inst) {
	/**
	 * Offset calibration can be done during normal device
	 * operation if at least 32 SCLK falling edges are
	 * provided in one serial transfer frame.
	 */
	// Add one extra byte transfer to ensure at least 32 SCLK falling edges for calibration?
	uint8_t zero_buffer[4] = {0}; // 8 * 4 = 32 SCLK falling edges
	spi_write_blocking(inst->spi, zero_buffer, 4);
}


uint16_t ads7043_read_raw(ads7043_inst_t *inst) {

	uint16_t result = 0;

	// Assert CS (active low)
	gpio_put(inst->cs_pin, false); // Is this necessary if SPI is handled by hardware?
	

	// Read 2 bytes from ADS7043
	uint8_t buffer[2] = {0};
	spi_read_blocking(inst->spi, 0x00, buffer, 2);
	// spi_read

	// Small delay to meet acquisition time requirements
	sleep_us(ADS7043_MIN_ACQUISITION_TIME_NS);

	// Deassert CS
	gpio_put(inst->cs_pin, true);


	// Combine the two bytes into a single 16-bit result
	result = ((uint16_t)buffer[0] << 8) | buffer[1];

	return result & ADS7043_DATA_MASK; // Mask to get valid data bits
}

void _test_raw16_to_float(uint16_t raw) {
	// Raw value * 2x attenuation, then scale to voltage based on 12-bit resolution and 2.8V reference
	float voltage1 = (raw / (float)(1 << (12 - 2))) * 2.8f;
	float voltage2 = (raw / (float)(1 << 12)) * 2.8f; // Assuming 3.3V reference
	printf("Raw: %04X, Voltage: %.3f V\n", raw, voltage1);
}

