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

#include "ads7043.h"

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
#define PIN_ONBOARD_LED 0

#define ADC_SPI       spi1
#define ADC_PIN_SCK   14
#define ADC_PIN_MISO  12
#define ADC_PIN_CS0   9
#define ADC_PIN_CS1   13

// #define USBBOOT_BTN 4

// #define PAUSE_AT_STARTUP 0
#define PAUSE_AT_STARTUP 1


uint16_t test_buff[1000];


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

	// ads7043_inst_t* ads7043 = ads7043_new(ADC_SPI, ADC_PIN_CS0, ADC_PIN_SCK, ADC_PIN_MISO, 16*1000*1000);
	ads7043_inst_t* ads7043 = ads7043_new(ADC_SPI, ADC_PIN_CS0, ADC_PIN_SCK, ADC_PIN_MISO, 1000*1000);
	// ads7043_inst_t* ads7043 = ads7043_new(ADC_SPI, ADC_PIN_CS0, ADC_PIN_SCK, ADC_PIN_MISO, 1000*1000);

	ads7043_init(ads7043);

	ads7043_selfcal(ads7043);

	// test_buff[0] = ads7043_read_raw(ads7043);

	// printf("Test read from ADC: %u\n", test_buff[0]);

	// spi_init(ADC_SPI, 1000 * 1000); // 1 MHz for initial testing, can be increased later
	// spi_set_format(ADC_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
	// gpio_set_function(ADC_PIN_MISO, GPIO_FUNC_SPI);
	// gpio_set_function(ADC_PIN_SCK, GPIO_FUNC_SPI);
	// gpio_set_function(ADC_PIN_MOSI, GPIO_FUNC_SPI);
	
	// gpio_set_function(ADC_PIN_CS0, GPIO_FUNC_SIO);
	// gpio_set_dir(ADC_PIN_CS0, GPIO_OUT);
	// gpio_put(ADC_PIN_CS0, 1); // Deassert CS

	// spi_write_blocking(SPI_PORT, (const uint8_t*)"Hello, ADC!", 12); // Send some initial data to test SPI

	// adc_set_sample_rate(200000); // Set sample rate to 200 kHz
	// printf("Sample rate set to %d Hz\n", adc_dma_cfg.sample_rate);


	printf("Initialization complete!\n");


	// printf("Starting ADC DMA...\n");
	// ads704x_dma_start();

	// Main loop
	while (1) {
		// printf("Reading from ADC...\n");
		// for (int i = 0; i < 1000; i++) {
		// 	test_buff[i] = ads7043_read_raw(ads7043);
		// }
		// printf("ADC Read complete! First sample: %u\n", test_buff[0]);
		// sleep_ms(1000);

		_test_raw16_to_float(ads7043_read_raw(ads7043));
		sleep_ms(500);
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




