#ifndef OVERCLOCK_H
#define OVERCLOCK_H

/**
 * ========================================
 * DO AT YOUR OWN RISK: Overclocking settings
 * ========================================
 */ 


/**
 * Must include
 */

#include "hardware/vreg.h"   // For overclocking
#include "hardware/clocks.h" // For overclocking
#include "hardware/pll.h"    // For overclocking


// #include "hardware/clocks.h" // For overclocking
// #include "hardware/regs/clocks.h" // For overclocking
// #include "hardware/pll.h"    // For overclocking
// #include "hardware/xosc.h"    // For overclocking


#ifndef OVERCLOCKING_ENABLED
#define OVERCLOCKING_ENABLED 0
#endif


typedef enum {
	// OC_ERR_OK = 0,
	OC_ERR_NOT_ENABLED = -1,
	OC_ERR_INVALID_CONFIG = -2,
	OC_ERR_CONFIG_FAILED = -3,
	OC_ERR_RESUS_OCCURRED = -4
} OC_ERR_t;


typedef struct {
	uint32_t frequency_khz;
	enum vreg_voltage voltage;
} OverclockConfig_t;


enum OverclockPresetNum {
	OC_FREQ_125 = 0,
	OC_FREQ_150,
	OC_FREQ_200,
	OC_FREQ_250,
	OC_FREQ_260,
	OC_FREQ_270 // WARNING: Requires 1.20V - Do at your own risk!
};

const OverclockConfig_t OC_PRESETS[] = {
	{125000, VREG_VOLTAGE_0_95},
	{150000, VREG_VOLTAGE_1_00},
	{200000, VREG_VOLTAGE_1_05},
	{250000, VREG_VOLTAGE_1_10},
	{260000, VREG_VOLTAGE_1_15},
	{270000, VREG_VOLTAGE_1_20} // The flash can only reach speeds of up to 260MHz-270MHz
};


// MACRO: Convert voltage enum to float value
#define CORE_VOLTAGE_FLOAT(volt) (0.55 + (volt * 0.05))


static volatile bool seen_resus = false;

void resus_callback(void) {
	// Reconfigure PLL sys back to the default state of 1500 / 6 / 2 = 125MHz
	pll_init(pll_sys, 1, 1500 * MHZ, 6, 2);

	// CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
	clock_configure(clk_sys,
	                CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
	                CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
	                125 * MHZ,
	                125 * MHZ);

	// Reconfigure uart as clocks have changed
	stdio_init_all();
	printf("Resus event fired\n");

	// // Wait for uart output to finish
	// uart_default_tx_wait_blocking();

	seen_resus = true;
}


/**
 * Overclock the core to the specified configuration.
 * On success, returns new frequency in kHz, read back
 * from the clock system. This may differ slightly from the
 * requested frequency due to clock divider rounding.
 * Returns type OC_ERR_t on failure:
 *     -1 = overclocking not enabled,
 *     -2 = invalid configuration,
 *     -3 = configuration failed,
 *     -4 = resus event occurred during overclocking (core reset)
 * The function enables resus handling before attempting to overclock,
 * ensuring that the processor does not halt, but the user must ensure
 * that their system can handle resus events appropriately. This
 * involves reinitializing peripherals and restoring state as needed,
 * and includes, but is not limited to, reinitializing serial output.
 * 
 * Parameters:
 *     config - OverclockConfig_t structure specifying frequency (kHz)
 *              and voltage (vreg_voltage enum).
 *     verbose - If true, prints detailed information to serial output.
 */
// int32_t overclock_core(OverclockConfig_t config, bool verbose = false) {
int32_t overclock_core(OverclockConfig_t config, bool verbose) {
	if (!OVERCLOCKING_ENABLED)
		return OC_ERR_NOT_ENABLED; // Overclocking not enabled


	// if (seen_resus)
	// 	return OC_ERR_RESUS_OCCURRED; // Previous resus event detected

	// Enable resus handling so we can recover if the core halts
	seen_resus = false; // Reset flag
	clocks_enable_resus(&resus_callback);
	
	if (verbose) {
		printf("Overclocking to %d MHz...\n", config.frequency_khz / 1000);
		printf("\tCurrent core voltage: %f mV\n", CORE_VOLTAGE_FLOAT(vreg_get_voltage()));
		printf("\tCurrent frequency: %d MHz\n", clock_get_hz(clk_sys)/MHZ);
		printf("\tSetting core voltage to %fV...\n", CORE_VOLTAGE_FLOAT(config.voltage));
	}

	// Set core voltage
	vreg_set_voltage(config.voltage);

	if (verbose) {
		printf("\tSetting frequency to %d MHz...\n", config.frequency_khz / 1000);
	}

	set_sys_clock_khz(config.frequency_khz, false);

	if (verbose) {
		printf("\tNew core voltage: %f mV\n", CORE_VOLTAGE_FLOAT(vreg_get_voltage()));
		printf("\tNew frequency: %d MHz\n", clock_get_hz(clk_sys)/MHZ);
	}

	return seen_resus ? OC_ERR_RESUS_OCCURRED : (clock_get_hz(clk_sys) / 1000);
}


bool test_resus() {
	seen_resus = false; // Reset flag
	clocks_enable_resus(&resus_callback);

	pll_deinit(pll_sys);

	sleep_ms(5);
	return seen_resus;
}


void find_valid_freq() {
	/**
	 * Find all the valid frequencies possible with
	 * the clock dividers and PLL settings.
	*/
	const int entry_size = sizeof(uint32_t) * 3; // Each entry has 3 uint32_t values

	uint32_t** valid_freqs;
	uint32_t len = 0;

	// uint 
	// for (int i = 12; i <= 400; i++) { // PLL VCO range: 1200MHz to 3000MHz
	// 	check_sys_clock_khz(125000); // Default frequency
	// }

}



#endif /* OVERCLOCK_H */