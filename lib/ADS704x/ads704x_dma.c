#include "ads704x_dma.h"

#include <stdint.h>
#include <stdbool.h>

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

#include "hardware/regs/io_bank0.h"


typedef struct {
	
	ads704x_dma_spi_t spi;
	_ads704x_dma_channel_t dma;
	
	uint32_t sample_rate;

	size_t buf_len; // Buffers must be the same length
	uint16_t* buf_rx0; // Must be 16-bit aligned for 16-bit transfers
	uint16_t* buf_rx1; // Must be 16-bit aligned for 16-bit transfers

	bool ch0_en;
	bool ch1_en;

	bool running;
	// bool finished;

	// bool dma_enabled;
	bool loop; // If true, the DMA will automatically restart after completing a buffer, otherwise it will stop and wait for the handler to restart it
	// bool ping_pong; // If true, the two ADC DMAs will alternate between the two buffers, otherwise they will both write to the same buffer

	void (*frame_complete_handler)(void);

} ads704x_dma_cfg_t;

ads704x_dma_cfg_t cfg = {
	.spi = {
		.inst = spi1,
		.sck = 14,
		.miso = 12,
		.cs0 = 13,
		.cs1 = 25
	},
	.dma = {
		.timer = 0,
		.irq = 0
	},
	.sample_rate = 200*1000,
	.buf_len = 0,
	.buf_rx0 = NULL,
	.buf_rx1 = NULL,
	.ch0_en = false,
	.ch1_en = false,
	.running = false,
	// .dma_enabled = false,
	.loop = true,
	// .ping_pong = false,
	.frame_complete_handler = NULL
};


// const uint8_t gpio_cs0 = 2; // GPIO pin used for CS of first ADS7043
// const uint8_t gpio_cs1 = 3; // GPIO pin used for CS of second
// #define gpio_cs0 2
// #define gpio_cs0 25
// #define gpio_cs0 19
// #define gpio_cs1 3
// #define spi_port spi1

// const uint16_t null_data = 0x0000;
const uint16_t null_data = 0xAAAA;
#define GPIO_MASK(pin) (1ul << (pin & 0x1fu)) // Data to write to GPIO set registers to assert CS lines


// #define TEST_SIZE 256





// dma_cs0_assert
// dma_tx
// dma_adc0_rx
// dma_cs0_deassert
// dma_cs1_assert
// dma_tx
// dma_adc1_rx
// dma_cs1_deassert
// Loop

/**
 * TODO:
 * - Make this function run from SRAM
 */
static void _sample_complete_handler() {
	printf("DMA RX Complete!\n");

	// reset the dma channel counters to restart the DMA chain for the next sample
	// dma_channel_set_read_addr(adc_dma_cfg.dma_tx0, &null_data, false);
	// dma_channel_set_write_addr(adc_dma_cfg.dma_tx0, &spi_get_hw(spi_port)->dr, false);

	cfg.running = true;
	dma_channel_start(cfg.dma.cs1_assert);
	return;
	/**
	 * TODO:
	 * Don't loop the DMA, instead use the handler to trigger the next sample
	 * acquisition by re-enabling the CS assert DMA for the next sample.
	 */

	/**
	 * TODO:
	 * Implement the ring buffer then we just have to check if loop_dma is enabled
	 * and if the end of the buffer has been reached before re-enabling the CS assert DMA.
	 */

	// TODO: Check if we should stop
	// if (!cfg.loop_dma) {
	// 	dma_channel_abort(cfg.dma.cs0_assert);
	// 	dma_channel_abort(cfg.dma.tx0);
	// 	dma_channel_abort(cfg.dma.rx0);
	// 	dma_channel_abort(cfg.dma.cs0_deassert);
	// 	dma_channel_abort(cfg.dma.cs1_assert);
	// 	dma_channel_abort(cfg.dma.tx1);
	// 	dma_channel_abort(cfg.dma.rx1);
	// 	dma_channel_abort(cfg.dma.cs1_deassert);
	// 	return;
	// }

	// Check the DMA channel write address to check if the end of the buffer has been reached
	// Always check the first enabled ADC's DMA as it triggers the interrupt before the second ADC has finished writing its sample
	if (cfg.ch0_en) {
		dma_channel_hw_t *hw = dma_channel_hw_addr(cfg.dma.rx0);
		// if (hw->read_addr >= (uint32_t)(adc_dma_cfg.rxbuf0 + adc_dma_cfg.buf_len)) {
		// 	dma_channel_set_enabled(adc_dma_cfg.dma_cs0_assert, false);
		// }
		if (hw->read_addr < (uint32_t)(cfg.buf_rx0 + cfg.buf_len)) {
			dma_channel_start(cfg.dma.cs0_assert); // Continue
			return;
		} else if (cfg.loop) {
			ads704x_dma_start();
			return;
		} else {
			cfg.running = false;
			return; // Stop
		}
	} else {
		dma_channel_hw_t *hw = dma_channel_hw_addr(cfg.dma.rx1);
		// if (hw->read_addr >= (uint32_t)(adc_dma_cfg.rxbuf1 + adc_dma_cfg.buf_len)) {
		// 	dma_channel_set_enabled(adc_dma_cfg.dma_cs1_assert, false);
		// }
		if (hw->read_addr < (uint32_t)(cfg.buf_rx1 + cfg.buf_len)) {
			dma_channel_start(cfg.dma.cs1_assert); // Continue
			return;
		} else if (cfg.loop) {
			ads704x_dma_start();
			return;
		} else {
			cfg.running = false;
			return; // Stop
		}
	}

}

static void _setup_channels() {
	cfg.dma.cs0_assert_cfg = dma_channel_get_default_config(cfg.dma.cs0_assert);
	cfg.dma.tx0_cfg = dma_channel_get_default_config(cfg.dma.tx0);
	cfg.dma.rx0_cfg = dma_channel_get_default_config(cfg.dma.rx0);
	cfg.dma.cs0_deassert_cfg = dma_channel_get_default_config(cfg.dma.cs0_deassert);
	cfg.dma.cs1_assert_cfg = dma_channel_get_default_config(cfg.dma.cs1_assert);
	cfg.dma.tx1_cfg = dma_channel_get_default_config(cfg.dma.tx1);
	cfg.dma.rx1_cfg = dma_channel_get_default_config(cfg.dma.rx1);
	cfg.dma.cs1_deassert_cfg = dma_channel_get_default_config(cfg.dma.cs1_deassert);

	// Configure each DMA channel with appropriate read/write addresses, transfer counts, and pacing

	/**
	 * CAUTION:
	 * READ_ADDR and WRITE_ADDR must always be aligned to the current transfer size,
	 * as specified in CTRL.DATA_SIZE. It is up to software to ensure the initial
	 * values are correctly aligned.
	 */

	const uint32_t mask0 = GPIO_MASK(cfg.spi.cs0);
	const uint32_t mask1 = GPIO_MASK(cfg.spi.cs1);

	// CS0 assert DMA: Write to GPIO clr register to assert CS0
	channel_config_set_dreq(&cfg.dma.cs0_assert_cfg, (DREQ_DMA_TIMER0 + cfg.dma.timer)); // Pace with timer to achieve desired sampling rate
	// channel_config_set_chain_to(&cfg.dma.cs0_assert_cfg, cfg.dma.tx0_cfg); // Chain to SPI TX DMA
	channel_config_set_transfer_data_size(&cfg.dma.cs0_assert_cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg.dma.cs0_assert_cfg, false);
	channel_config_set_write_increment(&cfg.dma.cs0_assert_cfg, false);
	dma_channel_configure(cfg.dma.cs0_assert, &cfg.dma.cs0_assert_cfg,
	                      &(sio_hw->gpio_clr),             // write address
	                      &mask0,                  // read address
	                      1,                               // element count
	                      false                            // don't start yet
	                     );
	// SPI TX DMA: Write dummy data to SPI to trigger ADC conversion
	// channel_config_set_chain_to(&cfg.dma.tx0_cfg, cfg.dma.rx0_cfg); // Chain to ADC0 RX DMA
	channel_config_set_transfer_data_size(&cfg.dma.tx0_cfg, DMA_SIZE_16);
	channel_config_set_dreq(&cfg.dma.tx0_cfg, spi_get_dreq(spi1, true)); // Pace by SPI TX FIFO DREQ
	channel_config_set_read_increment(&cfg.dma.tx0_cfg, false);
	channel_config_set_write_increment(&cfg.dma.tx0_cfg, false);
	dma_channel_configure(cfg.dma.tx0, &cfg.dma.tx0_cfg,
	                      &spi_get_hw(spi1)->dr,    // write address
	                      &null_data,                      // read address
	                      1,                               // element count
	                      false                            // don't start yet
	                     );
	// ADC0 RX DMA: Read data from SPI RX FIFO to buffer0
	// channel_config_set_chain_to(&cfg.dma.adc0_rx_cfg, cfg.dma.cs0_deassert_cfg); // Chain to CS0 deassert DMA
	channel_config_set_transfer_data_size(&cfg.dma.rx0_cfg, DMA_SIZE_16);
	channel_config_set_dreq(&cfg.dma.rx0_cfg, spi_get_dreq(spi1, false)); // Pace by SPI RX FIFO DREQ
	channel_config_set_read_increment(&cfg.dma.rx0_cfg, false);
	channel_config_set_write_increment(&cfg.dma.rx0_cfg, true);
	dma_channel_configure(cfg.dma.rx0, &cfg.dma.rx0_cfg,
	                      cfg.buf_rx0,            // write address
	                      &spi_get_hw(spi1)->dr,    // read address
	                      1,                               // Copy one sample
	                      false                            // don't start yet
	                     );
	// CS0 deassert DMA: Write to GPIO set register to deassert CS0 (triggered by ADC0 RX completion)
	// channel_config_set_chain_to(&cfg.dma.cs0_deassert_cfg, cfg.dma.cs1_assert_cfg); // Chain to CS1 assert DMA
	channel_config_set_transfer_data_size(&cfg.dma.cs0_deassert_cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg.dma.cs0_deassert_cfg, false);
	channel_config_set_write_increment(&cfg.dma.cs0_deassert_cfg, false);
	dma_channel_configure(cfg.dma.cs0_deassert, &cfg.dma.cs0_deassert_cfg,
	                      &(sio_hw->gpio_set),             // write address
	                      &mask0,                  // read address
	                      1,                               // element count
	                      false                            // don't start yet
	                     );
	// CS1 assert DMA: Write to GPIO clr register to assert CS1 (triggered by CS0 deassert completion)
	// channel_config_set_chain_to(&cfg.dma.cs1_assert_cfg, cfg.dma.tx1); // Chain to SPI TX DMA for second ADC
	channel_config_set_transfer_data_size(&cfg.dma.cs1_assert_cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg.dma.cs1_assert_cfg, false);
	channel_config_set_write_increment(&cfg.dma.cs1_assert_cfg, false);
	dma_channel_configure(cfg.dma.cs1_assert, &cfg.dma.cs1_assert_cfg,
	                      &(sio_hw->gpio_clr),             // write address
	                      &mask1,                  // read address
	                      1,                               // element count
	                      false                            // don't start yet
	                     );
	// SPI TX DMA for second ADC: Write dummy data to SPI to trigger ADC conversion
	// channel_config_set_chain_to(&cfg.dma.tx1_cfg, cfg.dma.adc1_rx); // Chain to ADC1 RX DMA
	channel_config_set_transfer_data_size(&cfg.dma.tx1_cfg, DMA_SIZE_16);
	channel_config_set_dreq(&cfg.dma.tx1_cfg, spi_get_dreq(spi1, true)); // Pace by SPI TX FIFO DREQ
	channel_config_set_read_increment(&cfg.dma.tx1_cfg, false);
	channel_config_set_write_increment(&cfg.dma.tx1_cfg, false);
	dma_channel_configure(cfg.dma.tx1, &cfg.dma.tx1_cfg,
	                      &spi_get_hw(spi1)->dr,    // write address
	                      &null_data,                      // read address
	                      1,                               // element count
	                      false                            // don't start yet
	                     );
	// ADC1 RX DMA: Read data from SPI RX FIFO to buffer1
	// channel_config_set_chain_to(&adc_dma_cfg.dma_adc1_rx_config, adc_dma_cfg.dma_cs1_deassert_config); // Chain to CS1 deassert DMA
	channel_config_set_transfer_data_size(&cfg.dma.rx1_cfg, DMA_SIZE_16);
	channel_config_set_dreq(&cfg.dma.rx1_cfg, spi_get_dreq(spi1, false)); // Pace by SPI RX FIFO DREQ
	channel_config_set_read_increment(&cfg.dma.rx1_cfg, false);
	channel_config_set_write_increment(&cfg.dma.rx1_cfg, true);
	dma_channel_configure(cfg.dma.rx1, &cfg.dma.rx1_cfg,
	                      cfg.buf_rx1, // write address
	                      &spi_get_hw(spi1)->dr,       // read address (SPI data register)
	                      1,                                  // Copy one sample
	                      false                               // don't start yet (will be started by CS1 assert DMA completion)
	                     );
	// CS1 deassert DMA: Write to GPIO set register to deassert CS1 (triggered by ADC1 RX completion)
	// channel_config_set_chain_to(&cfg.dma.cs1_deassert_cfg, cfg.dma.cs0_assert); // Chain back to CS0 assert DMA to create continuous loop
	channel_config_set_transfer_data_size(&cfg.dma.cs1_deassert_cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg.dma.cs1_deassert_cfg, false);
	channel_config_set_write_increment(&cfg.dma.cs1_deassert_cfg, false);
	dma_channel_configure(cfg.dma.cs1_deassert, &cfg.dma.cs1_deassert_cfg,
	                      &(sio_hw->gpio_set),                // write address (CS1 deassert)
	                      &mask1,                   // read address (dummy data)
	                      1,                                  // element count (single 32-bit value)
	                      false                               // don't start yet (will be started by ADC1 RX DMA completion)
	                     );

	// Chain the DMAs and configure IRQ for ADC RX completion to trigger sample processing and stopping the timer DMA to end the acquisition window
	if (cfg.ch0_en && cfg.ch1_en) {
		// Both ADCs enabled: Chain in sequence CS0 assert -> ADC0 RX -> CS0 deassert -> CS1 assert -> ADC1 RX -> CS1 deassert
		channel_config_set_chain_to(&cfg.dma.cs0_assert_cfg, cfg.dma.tx0); // Chain to SPI TX DMA for ADC0
		channel_config_set_chain_to(&cfg.dma.tx0_cfg, cfg.dma.rx0); // Chain to ADC0 RX DMA
		channel_config_set_chain_to(&cfg.dma.rx0_cfg, cfg.dma.cs0_deassert); // Chain to CS0 deassert DMA
		channel_config_set_chain_to(&cfg.dma.cs0_deassert_cfg, cfg.dma.cs1_assert); // Chain to CS1 assert DMA
		channel_config_set_chain_to(&cfg.dma.cs1_assert_cfg, cfg.dma.tx1); // Chain to SPI TX DMA for ADC1
		channel_config_set_chain_to(&cfg.dma.tx1_cfg, cfg.dma.rx1); // Chain to ADC1 RX DMA
		channel_config_set_chain_to(&cfg.dma.rx1_cfg, cfg.dma.cs1_deassert); // Chain to CS1 deassert DMA
		// channel_config_set_chain_to(&cfg.dma.cs1_deassert_cfg, cfg.dma.cs0_assert); // Chain back to CS0 assert DMA to create continuous loop
		// Don't pace cs1
		channel_config_set_dreq(&cfg.dma.cs1_assert_cfg, 0);
		// Configure IRQ on completion of ADC RX DMA (end of full frame)
		dma_channel_set_irq0_enabled(cfg.dma.rx0, true); // Use ADC0 RX completion to allow the most time to check for end of buffer and stop.
		dma_channel_set_irq0_enabled(cfg.dma.rx1, true); // Use ADC1 RX completion to allow the most time to check for end of buffer and stop.
	} else if (cfg.ch0_en) {
		printf("Only ADC0 enabled, configuring DMA chain for single ADC operation\n");
		// Only ADC0 enabled: Chain CS0 assert -> ADC0 RX -> CS0 deassert
		channel_config_set_chain_to(&cfg.dma.cs0_assert_cfg, cfg.dma.tx0); // Chain to SPI TX DMA for ADC0
		channel_config_set_chain_to(&cfg.dma.tx0_cfg, cfg.dma.rx0); // Chain to ADC0 RX DMA
		channel_config_set_chain_to(&cfg.dma.rx0_cfg, cfg.dma.cs0_deassert); // Chain to CS0 deassert DMA
		channel_config_set_chain_to(&cfg.dma.cs0_deassert_cfg, cfg.dma.cs0_assert); // Chain to CS0 assert DMA
		// Don't pace cs1
		// channel_config_set_dreq(&cfg.dma.cs1_assert_cfg, 0); // Don't care
		// channel_config_set_chain_to(&cfg.dma.cs0_deassert_cfg, cfg.dma.cs0_assert); // Chain back to CS0 assert DMA to create continuous loop
		// Configure IRQ on completion of ADC RX DMA (end of full frame)
		dma_channel_set_irq0_enabled(cfg.dma.rx0, true);
	} else if (cfg.ch1_en) {
		// Only ADC1 enabled: Chain CS1 assert -> ADC1 RX -> CS1 deassert
		channel_config_set_chain_to(&cfg.dma.cs1_assert_cfg, cfg.dma.tx1); // Chain to SPI TX DMA for ADC1
		channel_config_set_chain_to(&cfg.dma.tx1_cfg, cfg.dma.rx1); // Chain to ADC1 RX DMA
		channel_config_set_chain_to(&cfg.dma.rx1_cfg, cfg.dma.cs1_deassert); // Chain to CS1 deassert DMA
		// Pace with timer to achieve desired sampling rate
		channel_config_set_dreq(&cfg.dma.cs1_assert_cfg, (DREQ_DMA_TIMER0 + cfg.dma.timer));
		// channel_config_set_chain_to(&cfg.dma.cs1_deassert_cfg, cfg.dma.cs1_assert); // Chain back to CS1 assert DMA to create continuous loop
		// Configure IRQ on completion of ADC RX DMA (end of full frame)
		dma_channel_set_irq0_enabled(cfg.dma.rx1, true);
	} else {
		// No ADCs enabled: Chain CS assert DMAs to create continuous loop
		// channel_config_set_chain_to(&cfg.dma.cs0_assert_cfg, cfg.dma.cs0_deassert); // Chain CS0 assert to CS0 deassert
		// channel_config_set_chain_to(&cfg.dma.cs0_deassert_cfg, cfg.dma.cs1_assert); // Chain CS0 deassert to CS1 assert
		// channel_config_set_chain_to(&cfg.dma.cs1_assert_cfg, cfg.dma.cs1_deassert); // Chain CS1 assert to CS1 deassert
		// channel_config_set_chain_to(&cfg.dma.cs1_deassert_cfg, cfg.dma.cs0_assert); // Chain CS1 deassert back to CS0 assert
	}

	// dma_channel_set_irq0_enabled(cfg.dma_cs0_assert, true);
	// dma_channel_set_irq0_enabled(cfg.dma_tx0, true);
	dma_channel_set_irq0_enabled(cfg.dma.rx0, true);
	dma_channel_set_irq0_enabled(cfg.dma.cs0_deassert, true);

}

void ads704x_dma_spi_init() {
	// Configure SPI pins
	gpio_set_function(cfg.spi.sck, GPIO_FUNC_SPI);
	gpio_set_function(cfg.spi.miso, GPIO_FUNC_SPI);

	// Configure CS pins as GPIO outputs
	gpio_set_function(cfg.spi.cs0, GPIO_FUNC_SIO);
	gpio_set_dir(cfg.spi.cs0, GPIO_OUT);
	gpio_put(cfg.spi.cs0, 1); // Deassert CS

	gpio_set_function(cfg.spi.cs1, GPIO_FUNC_SIO);
	gpio_set_dir(cfg.spi.cs1, GPIO_OUT);
	gpio_put(cfg.spi.cs1, 1); // Deassert CS

	// Configure SPI
	spi_init(cfg.spi.inst, 16*1000*1000);
	// ADS7043 requires CPOL=0 and CPHA=0.
	// Use 14 bits per transfer to read one sample in each transfer and avoid
	// having to do bit manipulation to extract samples from 16-bit transfers.
	spi_set_format(cfg.spi.inst, 14, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
}


void ads704x_dma_set_spi(spi_inst_t *spi, uint8_t sck_pin, uint8_t miso_pin, uint8_t cs0_pin, uint8_t cs1_pin) {
	cfg.spi.inst = spi;
	cfg.spi.sck = sck_pin;
	cfg.spi.miso = miso_pin;
	cfg.spi.cs0 = cs0_pin;
	cfg.spi.cs1 = cs1_pin;

	ads704x_dma_spi_init();
}


int ads704x_dma_init() {

	ads704x_dma_spi_init();

	// Configure DMA channels and handlers
	// adc_dma_cfg.frame_complete_handler = _sample_complete_handler;

	/**
	 * TODO:
	 * Configure ring:
	 * channel_config_set_ring(&c, true, 3); // 1 << 3 byte boundary on write ptr
	 * Ring works by masking the lower address bits of the read or write address.
	 * effective_addr = (base_addr & ∼((1<<n)-1)) | (addr_counter & ((1<<n)-1))
	 * where n = ring_bits
	 */

	/**
	 * TODO: channel_config_set_irq_quiet(&c, true);
	 */

	cfg.dma.cs0_assert = dma_claim_unused_channel(true);
	cfg.dma.tx0 = dma_claim_unused_channel(true);
	cfg.dma.rx0 = dma_claim_unused_channel(true);
	cfg.dma.cs0_deassert = dma_claim_unused_channel(true);
	cfg.dma.cs1_assert = dma_claim_unused_channel(true);
	cfg.dma.tx1 = dma_claim_unused_channel(true);
	cfg.dma.rx1 = dma_claim_unused_channel(true);
	cfg.dma.cs1_deassert = dma_claim_unused_channel(true);


	/**
	 * Configure timer to trigger at 200kHz given 250MHz system clock
	 * 250MHz / 200kHz = 1250 cycles
	 * The timer will run at the system_clock_freq * numerator / denominator, so set numerator to 1 and denominator to calculated divider to achieve desired sample rate
	 * 
	 * The numerator and denominator are both 16-bit values
	 */

	for (int i = 0; i <= 4; i++) {
		if (i==4) {
			printf("ERROR: No more timers available to claim!\n");
			return 1;
		}
		if (!dma_timer_is_claimed(i)) {
			cfg.dma.timer = i;
			dma_timer_claim(i);
			printf("Claimed DMA timer %d\n", i);
			break;
		}
	}

	for (int i = 0; i <= 2; i++) {
		if (i==2) {
			printf("ERROR: No more IRQs available to claim!\n");
			return 1;
		}
		if (!irq_has_handler(i)) {
			cfg.dma.irq = i;
			irq_set_exclusive_handler(DMA_IRQ_NUM(i), _sample_complete_handler);
			irq_set_enabled(DMA_IRQ_NUM(i), true);
			printf("Claimed DMA IRQ %d\n", i);
		}
	}

	// Configure the processor to run _sample_complete_handler() when ADC RX DMA completes a sample
	irq_set_exclusive_handler(DMA_IRQ_NUM(cfg.dma.irq), _sample_complete_handler);
	irq_set_enabled(DMA_IRQ_NUM(cfg.dma.irq), true);

	// Configure DMA channels
	_setup_channels();

	// Set initial sample rate
	ads704x_dma_set_sample_rate(cfg.sample_rate);
	

	return 0;
}

uint32_t ads704x_dma_get_sample_rate() {
	return cfg.sample_rate;
}

uint32_t ads704x_dma_set_sample_rate(uint32_t sample_rate) {
	struct freq_div* dma_timer_settings = NULL;
	size_t len = 0;
	if (clock_get_hz(clk_sys) == 125000000) {
		dma_timer_settings = dma_timer_settings_125MHz;
		len = dma_timer_settings_125MHz_len;
	} else if (clock_get_hz(clk_sys) == 200000000) {
		dma_timer_settings = dma_timer_settings_200MHz;
		len = dma_timer_settings_200MHz_len;
	} else if (clock_get_hz(clk_sys) == 250000000) {
		dma_timer_settings = dma_timer_settings_250MHz;
		len = dma_timer_settings_250MHz_len;
	} else if (clock_get_hz(clk_sys) == 260000000) {
		dma_timer_settings = dma_timer_settings_260MHz;
		len = dma_timer_settings_260MHz_len;
	} else {
		printf("Unsupported system clock frequency for setting sample rate\n");
		return cfg.sample_rate;
	}

	for (size_t i = 0; i < len; i++) {
		// printf("Checking timer setting: divider %d -> %d Hz\n", dma_timer_settings[i].d, dma_timer_settings[i].freq);
		if (dma_timer_settings[i].freq == sample_rate) {
			printf("Setting sample rate to %d Hz with timer divider %d\n",
			       sample_rate, dma_timer_settings[i].d);
			dma_timer_set_fraction(0, 1, dma_timer_settings[i].d);
			cfg.sample_rate = dma_timer_settings[i].freq;
			break;
		}
	}
	return cfg.sample_rate;
}

void ads704x_dma_get_valid_sample_rates() {
// 	for x in range(1,(2**16)-1):
//  *     if sys_clk%x == 0:
//  *         print("{", x, ", ", (sys_clk/x), "},")

	uint32_t* sample_rates;
	uint16_t len = 0;

	uint32_t clk = clock_get_hz(clk_sys);
	for (int i = 1; i < 65536; i++) {
		if (clk % i == 0) {
			// sample_rates = realloc(&sample_rates, (len+1)*sizeof(uint32_t));
			if (sample_rates == NULL)
				return; // TODO: ERROR
			sample_rates[len] = (uint32_t)(clk / i);
			len++;
		}
	}
	return;
}

// void ads704x_dma_get_valid_sample_rates(uint32_t** rates, size_t** len) {
// 	struct freq_div* dma_timer_settings = NULL;
// 	size_t len_tmp = 0;
// 	if (clock_get_hz(clk_sys) == 125000000) {
// 		dma_timer_settings = dma_timer_settings_125MHz;
// 		len_tmp = dma_timer_settings_125MHz_len;
// 	} else if (clock_get_hz(clk_sys) == 200000000) {
// 		dma_timer_settings = dma_timer_settings_200MHz;
// 		len_tmp = dma_timer_settings_200MHz_len;
// 	} else if (clock_get_hz(clk_sys) == 250000000) {
// 		dma_timer_settings = dma_timer_settings_250MHz;
// 		len_tmp = dma_timer_settings_250MHz_len;
// 	} else if (clock_get_hz(clk_sys) == 260000000) {
// 		dma_timer_settings = dma_timer_settings_260MHz;
// 		len_tmp = dma_timer_settings_260MHz_len;
// 	} else {
// 		printf("Unsupported system clock frequency for getting valid sample rates\n");
// 		return;
// 	}

// 	for (size_t i = 0; i < len_tmp; i++) {
// 		rates[i] = dma_timer_settings[i].freq;
// 	}
// 	*len = len_tmp;
// }


void ads704x_dma_set_buffers(uint16_t* buf0, uint16_t* buf1, size_t len) {
	cfg.buf_rx0 = buf0;
	cfg.buf_rx1 = buf1;
	cfg.buf_len = len;
}

void ads704x_dma_channel_enable(ADS704x_DMA_CHANNEL_t ch) {
	if (ch == ADS704x_DMA_CH0 || ch == ADS704x_DMA_CH_BOTH) {
		cfg.ch0_en = true;
	}
	if (ch == ADS704x_DMA_CH1 || ch == ADS704x_DMA_CH_BOTH) {
		cfg.ch1_en = true;
	}
}

void ads704x_dma_channel_disable(ADS704x_DMA_CHANNEL_t ch) {
	if (ch == ADS704x_DMA_CH0 || ch == ADS704x_DMA_CH_BOTH) {
		cfg.ch0_en = false;
	}
	if (ch == ADS704x_DMA_CH1 || ch == ADS704x_DMA_CH_BOTH) {
		cfg.ch1_en = false;
	}
}


bool ads704x_dma_is_running(void) {
	return cfg.running;
}

void ads704x_dma_stop() {
	dma_channel_abort(cfg.dma.cs0_assert);
	dma_channel_abort(cfg.dma.tx0);
	dma_channel_abort(cfg.dma.rx0);
	dma_channel_abort(cfg.dma.cs0_deassert);
	dma_channel_abort(cfg.dma.cs1_assert);
	dma_channel_abort(cfg.dma.tx1);
	dma_channel_abort(cfg.dma.rx1);
	dma_channel_abort(cfg.dma.cs1_deassert);

	cfg.running = false;
}


void ads704x_dma_start() {
	if (!cfg.ch0_en && !cfg.ch1_en) {
		printf("Error: No channels enabled\n");
		return;
	}

	if (cfg.buf_len == 0) {
		printf("Error: Invalid buffer length\n");
		return;
	}

	// ads704x_dma_stop();
	// // TODO: Check if running
	// if (dma_channel_is_busy(cfg.dma.cs0_assert) ||
	//     dma_channel_is_busy(cfg.dma.tx0) ||
	//     dma_channel_is_busy(cfg.dma.rx0) ||
	//     dma_channel_is_busy(cfg.dma.cs0_deassert) ||
	//     dma_channel_is_busy(cfg.dma.cs1_assert) ||
	//     dma_channel_is_busy(cfg.dma.tx1) ||
	//     dma_channel_is_busy(cfg.dma.rx1) ||
	//     dma_channel_is_busy(cfg.dma.cs1_deassert))
	if (cfg.running)
	{
		printf("Error: DMA channels are busy, cannot start\n");
		return;
	}

	// Reconfigure DMA channels with new buffer addresses and transfer counts
	if (cfg.ch0_en) {
		if (cfg.buf_rx0 == NULL) {
			printf("Error: Invalid buffer address for CH0\n");
			return;
		}
		dma_channel_set_write_addr(cfg.dma.rx0, cfg.buf_rx0, false);
		dma_channel_set_transfer_count(cfg.dma.rx0, cfg.buf_len, false);
	}
	if (cfg.ch1_en) {
		if (cfg.buf_rx1 == NULL) {
			printf("Error: Invalid buffer address for CH1\n");
			return;
		}
		dma_channel_set_write_addr(cfg.dma.rx1, cfg.buf_rx1, false);
		dma_channel_set_transfer_count(cfg.dma.rx1, cfg.buf_len, false);
	}

	// Start the DMA chain by enabling the CS assert DMA for the first ADC
	if (cfg.ch0_en) {
		dma_channel_start(cfg.dma.cs0_assert);
	} else if (cfg.ch1_en) {
		dma_channel_start(cfg.dma.cs1_assert);
	}

	cfg.running = true;

}


