/**
 * file: main.cpp
 * author: Michael MacDonald
 * date: 2026-02-03
 */


/**
 * ========================================
 * Includes
 * ========================================
 */

// Standard includes
#include <stdio.h>
#include <stdlib.h> // For malloc and free
#include <stdint.h>
#include <stdbool.h>

// Pico SDK includes
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

#include "ads704x_dma.h"

// Overclocking settings
#define OVERCLOCKING_ENABLED 1 // Define this before including overclock.h
#include "overclock.h"


/**
 * ========================================
 * Configuration Defines
 * ========================================
 */

// #define PIN_ONBOARD_LED 25
// #define PIN_ONBOARD_LED 24
#define PIN_ONBOARD_LED 13


#define SPI_PORT spi1
#define PIN_MISO 8
#define PIN_CS 25
#define PIN_SCK 14
#define PIN_MOSI 15

// #define USBBOOT_BTN 4

// #define PAUSE_AT_STARTUP 0
#define PAUSE_AT_STARTUP 1



extern struct adc_dma_cfg_s adc_dma_cfg;


// Function declarations
void main_core1();

int main() {
	// Initialize serial output
	stdio_init_all();
	// Wait to ensure initialization completes
	sleep_ms(50);

#ifdef USBBOOT_BTN
	gpio_set_dir(USBBOOT_BTN, GPIO_IN);
	gpio_set_pulls(USBBOOT_BTN, true, false); // Enable pull-up
#endif

#if PAUSE_AT_STARTUP
#ifdef USBBOOT_BTN
	// Wait for serial connection or button press
	while (!stdio_usb_connected() && gpio_get(USBBOOT_BTN)) {
		sleep_ms(100);
	}
#else // USBBOOT_BTN
	// Wait for serial connection
	while (!stdio_usb_connected()) {
		sleep_ms(10);
	}
#endif // USBBOOT_BTN
#endif // PAUSE_AT_STARTUP

	printf("Serial connected!\n");

	// Overclock core
	// int ret = overclock_core(OC_PRESETS[OC_FREQ_250], true);
	// // int ret = overclock_core(OC_PRESETS[OC_FREQ_125], true);
	// if (ret < 0) {
	// 	printf("Overclocking failed with error code: %d\n", ret);
	// 	if (ret == OC_ERR_RESUS_OCCURRED) {
	// 		stdio_init_all();
	// 		printf("A resus event occurred during overclocking. System has been reset to safe frequency.\n");
	// 	}
	// } else {
	// 	printf("Overclocking successful! New frequency: %d kHz\n", ret);
	// }


	// Launch core 1
	printf("Starting core 1...\n");
	multicore_launch_core1(main_core1);

	// ads704x_dma_set_spi(spi_inst_t *spi, uint8_t sck_pin, uint8_t miso_pin, uint8_t cs0_pin, uint8_t cs1_pin);
	ads704x_dma_channel_enable(ADS704x_DMA_CH0);
	uint16_t* buff0 = (uint16_t*)malloc(sizeof(uint16_t)*TEST_SIZE);
	if (buff0 == NULL) {
		printf("Error allocating buffer\n");
		return -1;
	}
	ads704x_dma_set_buffers(buff0, NULL, TEST_SIZE);
	// ads704x_dma_set_frame_complete_handler(_frame_complete_handler);
	// ads704x_dma_set_sample_rate(uint32_t sample_rate);

	ads704x_dma_set_sample_rate(200000); // Set sample rate to 200 kHz
	printf("Sample rate set to %d Hz\n", ads704x_dma_get_sample_rate());
	
	int ret = ads704x_dma_init();
	if (ret < 0) {
		printf("Error initializing ADC DMA: %d\n", ret);
		return -1;
	}

	// spi_init(SPI_PORT, 1000 * 1000); // 1 MHz for initial testing, can be increased later
	// spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
	// gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
	// gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
	// gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
	
	// gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
	// gpio_set_dir(PIN_CS, GPIO_OUT);
	// gpio_put(PIN_CS, 1); // Deassert CS

	// spi_write_blocking(SPI_PORT, (const uint8_t*)"Hello, ADC!", 12); // Send some initial data to test SPI


	// adc_dma_cfg.adc0_enabled = true;
	// adc_dma_cfg.adc1_enabled = false;
	// adc_dma_cfg.dma_enabled = false;

	// adc_dma_cfg.loop_dma = true;

	// adc_dma_cfg.rxbuf0 = (uint16_t*)malloc(sizeof(uint16_t) * TEST_SIZE);
	// adc_dma_cfg.rxbuf1 = (uint16_t*)malloc(sizeof(uint16_t) * TEST_SIZE);
	// adc_dma_cfg.buf_len = TEST_SIZE;

	// init_adc_dma();

	// adc_set_sample_rate(200000); // Set sample rate to 200 kHz
	// printf("Sample rate set to %d Hz\n", adc_dma_cfg.sample_rate);


	printf("Initialization complete!\n");


	printf("Starting ADC DMA...\n");
	ads704x_dma_start();

	// adc_dma_cfg.dma_enabled = true;
	// dma_channel_start(adc_dma_cfg.dma_cs0_assert);
	// dma_channel_start(adc_dma_cfg.dma_tx0);

	// Main loop
	while (1) {
		// if (dma_channel_is_busy(cfg.dma.cs0_deassert)||dma_channel_is_busy(cfg.dma.rx0))
		// 	printf("ADC0 DMA is busy\n");
		if (!ads704x_dma_is_running()) {
			printf("DMA is not running, restarting...\n");
			ads704x_dma_start();
		}
	}
}



void main_core1() {
	gpio_set_function(PIN_ONBOARD_LED, GPIO_FUNC_SIO);
	gpio_set_dir(PIN_ONBOARD_LED, GPIO_OUT);
	gpio_set_drive_strength(PIN_ONBOARD_LED, GPIO_DRIVE_STRENGTH_4MA);

	while(1) {
		// Blink onboard LED
		gpio_put(PIN_ONBOARD_LED, false); // LED off
		sleep_ms(1000);
		gpio_put(PIN_ONBOARD_LED, true); // LED on
		// printf("Blink!\n");
		sleep_ms(1000);
	}
}




