
#include "pico/stdlib.h"


#include "hardware/clocks.h" // For overclocking
#include "hardware/regs/clocks.h" // For overclocking
#include "hardware/pll.h"    // For overclocking
#include "hardware/xosc.h"    // For overclocking


int main()
{
	// // Disable resus handling during initialization to prevent unwanted resets
	// clocks_hw->resus.ctrl = 0;
	// // Disable external crystal oscillator to for baseline power draw measurements
	// xosc_disable();

	// overclock_core({50000, VREG_VOLTAGE_0_90}, true);
	// while(1) sleep_ms(0xFFFFFFFF); // Placeholder to allow core 1 to run first

}