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
#include <stdint.h>
#include <stdbool.h>

// Pico SDK includes
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"

// Seesaw Driver
#include "seesaw.h"

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

// I2C defines
#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3

// #define USBBOOT_BTN 4

// #define PAUSE_AT_STARTUP 0
#define PAUSE_AT_STARTUP 1

Seesaw_t ss = {
	.i2c_inst = I2C_PORT,
	.i2c_addr = SEESAW_DEFAULT_ADDR
};

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
	int ret = overclock_core(OC_PRESETS[OC_FREQ_250], true);
	// int ret = overclock_core(OC_PRESETS[OC_FREQ_125], true);
	if (ret < 0) {
		printf("Overclocking failed with error code: %d\n", ret);
		if (ret == OC_ERR_RESUS_OCCURRED) {
			stdio_init_all();
			printf("A resus event occurred during overclocking. System has been reset to safe frequency.\n");
		}
	} else {
		printf("Overclocking successful! New frequency: %d kHz\n", ret);
	}


	// Launch core 1
	printf("Starting core 1...\n");
	multicore_launch_core1(main_core1);


	// I2C Initialisation. Using it at 400Khz.
	printf("Initialising I2C...\n");
	// Returns actual baudrate which may differ from requested baud
	ss.baud = i2c_init(I2C_PORT, 400*1000); // 400kHz
	/**
	 * NOTE: We may need stronger pull-ups for higher baud rates (2.2k for 400kHz)
	 */
	printf("I2C baudrate set to: %d Hz\n", ss.baud);
	gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA);
	gpio_pull_up(I2C_SCL);

	// Configure seesaw
	printf("Configuring Seesaw...\n");
	seesaw_gpio_pin_mode(ss, SEESAW_PIN_LED, SEESAW_OUTPUT); // Set pin 5 (onboard LED) as output
	seesaw_gpio_digital_write_bulk(ss, (1ul << SEESAW_PIN_LED), 1); // Turn off onboard LED
	const uint8_t button_pins[] = {9, 18, 12};
	const uint8_t num_enc = 3;
	for (int i = 0; i < num_enc; i++) {
		printf("Configuring encoder %d and button pin %d...\n", i, button_pins[i]);
		printf("Enabling encoder interrupt...\n");
		seesaw_encoder_enable_interrupt(ss, i); // Enable interrupt for encoder
		printf("\tSetting encoder position to 0...\n");
		seesaw_encoder_set_position(ss, i, 0); // Reset encoder position to 0
		printf("\tSetting button pin mode...\n");
		seesaw_gpio_pin_mode(ss, button_pins[i], SEESAW_INPUT_PULLUP); // Set button pins as input with pull-up
	}
	

	printf("Initialization complete!\n");

	// Main loop
	bool buttons[num_enc];
	int32_t position[num_enc];
	int32_t position_calc[num_enc];
	int32_t delta[num_enc];
	for (int i = 0; i < num_enc; i++) {
		position[i] = 0;
		position_calc[i] = 0;
		delta[i] = 0;
		buttons[i] = false;
	}
	const absolute_time_t delay_us = (50*1000); // 50ms
	absolute_time_t alarm_time = get_absolute_time() + delay_us;
	const absolute_time_t delay1_us = (1000*1000); // 1s
	absolute_time_t alarm1_time = get_absolute_time() + delay_us;
	while (1) {
		if (time_reached(alarm1_time)) {
			alarm1_time = get_absolute_time() + delay1_us;
			bool curr_led_state;
			seesaw_gpio_digital_read(ss, SEESAW_PIN_LED, &curr_led_state);
			// Toggle LED state
			seesaw_gpio_digital_write(ss, SEESAW_PIN_LED, !curr_led_state);
		}

		// Read encoder positions every 100ms
		if (time_reached(alarm_time)) {
			absolute_time_t old = alarm_time;
			alarm_time = get_absolute_time() + delay_us;
		
			bool bad = false;
			for (int i = 0; i < num_enc; i++) {
				// if (seesaw_encoder_get_delta(ss, i, &delta[i]) == 0)
				// 	position_calc[i] += delta[i];
				// else {
				// 	bad = true;
				// 	printf("Error reading encoder delta\n");
				// }
				if (seesaw_encoder_get_position(ss, i, &position[i]) != 0) {
					bad = true;
					// printf("Error reading encoder position\n");
				}
				sleep_us(250); // Small delay to avoid overloading the seesaw
				if (seesaw_gpio_digital_read(ss, button_pins[i], &buttons[i]) != 0) {
					bad = true;
					// printf("Error reading button state\n");
				}
				sleep_us(250); // Small delay to avoid overloading the seesaw
			}
			
			if (bad) {
				// printf("Error reading from Seesaw\n");
				continue; // Skip printing if there was an error reading from Seesaw
			}
			// printf("Time: %ld us    Overshoot: %ld us\t", (uint32_t)get_absolute_time(), (uint32_t)absolute_time_diff_us(old, get_absolute_time()));
			for (int i = 0; i < num_enc; i++) {
				// printf("Encoder %d: Pos Calc = %03d, Pos = %03d, Delta = %03d    ", i, position_calc[i], position[i], delta[i]);
				printf("Encoder %d Pos: %04ld Btn: %d    ", i, position[i], !buttons[i]);
				// printf("Encoder %d Pos: %03d    ", i, position_calc[i]);
			}
			printf("\n");
		}
	}
}



void main_core1() {
	gpio_set_function(PIN_ONBOARD_LED, GPIO_FUNC_SIO);
	gpio_set_dir(PIN_ONBOARD_LED, GPIO_OUT);
	gpio_set_drive_strength(PIN_ONBOARD_LED, GPIO_DRIVE_STRENGTH_4MA);

	while(1) {
		// Blink seesaw onboard LED
		// sleep_ms(1000);
		// seesaw_gpio_digital_write(ss, SEESAW_PIN_LED, false); // LED off
		// sleep_ms(1000);
		// seesaw_gpio_digital_write(ss, SEESAW_PIN_LED, true); // LED on
		// printf("Blink!\n");
		
		// Blink onboard LED
		gpio_put(PIN_ONBOARD_LED, false); // LED off
		// seesaw_gpio_digital_write(ss, SEESAW_PIN_LED, false); // LED on
		sleep_ms(1000);
		gpio_put(PIN_ONBOARD_LED, true); // LED on
		// seesaw_gpio_digital_write(ss, SEESAW_PIN_LED, true); // LED off
		// printf("Blink!\n");
		sleep_ms(1000);
	}
}




