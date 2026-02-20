
#include <stdio.h>
#include "pico/stdlib.h"

// Overclocking settings
#define OVERCLOCKING_ENABLED 1 // Define this before including overclock.h
#include "overclock.h"


#define PIN_ONBOARD_LED 25


int main() {
	// Initialize serial output
	stdio_init_all();
	// Wait to ensure initialization completes
	sleep_ms(50);

	// Wait for serial connection
	while (!stdio_usb_connected()) {
		sleep_ms(10);
	}

	printf("Serial connected!\n");

	gpio_set_dir(PIN_ONBOARD_LED, GPIO_OUT);
	gpio_set_drive_strength(PIN_ONBOARD_LED, GPIO_DRIVE_STRENGTH_2MA);

	// Overclock core
	int ret = overclock_core(OC_PRESETS[OC_FREQ_250], true);
	if (ret < 0) {
		printf("Overclocking failed with error code: %d\n", ret);
		if (ret == OC_ERR_RESUS_OCCURRED) {
			stdio_init_all();
			printf("A resus event occurred during overclocking. System has been reset to safe frequency.\n");
		}
	} else {
		printf("Overclocking successful! New frequency: %d kHz\n", ret);
	}

	// Loop
	while (1) {
		gpio_put(PIN_ONBOARD_LED, !gpio_get(PIN_ONBOARD_LED));
		printf("Running at %d MHz\n", clock_get_hz(clk_sys)/MHZ);
		sleep_ms(1000);
	}

}