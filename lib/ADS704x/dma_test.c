
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/malloc.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

// #include "hardware/structs/pads_bank0.h"
#include "hardware/regs/io_bank0.h"



/**
 * IDEA:
 * The DMA can be used to read samples from the ADS7043 at precise intervals without CPU intervention.
 * The DMA is configured to capture a fixed number of samples (e.g. 256) into a buffer, and then trigger
 * an interrupt when the buffer is full. Basically, it can trigger 
 */

 /**
  * TODO:
  * Allow for two ADS7043s to be read one after the other
  * Enable self-calibration mode of ADS7043 to reduce offset error
  */


const uint TEST_SIZE = 256;

struct {
	uint32_t dma_pace;  // 
	uint32_t dma_rx;    // 

	dma_channel_config dma_pace_config;
	dma_channel_config dma_rx_config;

	// const uint TEST_SIZE = TEST_SIZE;
	uint16_t* rxbuf;

	void (*frame_complete_handler)(void);

	struct repeating_timer timer;

} adc_dma_config;



const uint16_t null_data = 0x0000;


void _frame_complete_handler();


void init_adc_dma() {
	// Set function pointer for DMA complete handler
	adc_dma_config.frame_complete_handler = _frame_complete_handler;
	// Set


	// Configure SPI
	// hw_set_bits(&spi_get_hw(spi_default)->cr1, SPI_SSPCR1_LBM_BITS);

	// Allocate RX buffer
	adc_dma_config.rxbuf = malloc(sizeof(uint16_t) * TEST_SIZE);

	adc_dma_config.dma_pace = dma_claim_unused_channel(true);
	adc_dma_config.dma_rx = dma_claim_unused_channel(true);

	// We set the inbound DMA to transfer from the SPI receive FIFO to a memory buffer paced by the SPI RX FIFO DREQ
	// We configure the read address to remain unchanged for each element, but the write
	// address to increment (so data is written throughout the buffer)
	printf("Configure SPI RX DMA\n");
	adc_dma_config.dma_rx_config = dma_channel_get_default_config(adc_dma_config.dma_rx);
	// channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_transfer_data_size(&adc_dma_config.dma_rx_config, DMA_SIZE_16);
	channel_config_set_dreq(&adc_dma_config.dma_rx_config, spi_get_dreq(spi_default, false));
	channel_config_set_read_increment(&adc_dma_config.dma_rx_config, false);
	channel_config_set_write_increment(&adc_dma_config.dma_rx_config, true);
	dma_channel_configure(adc_dma_config.dma_rx, &adc_dma_config.dma_rx_config,
	                      adc_dma_config.rxbuf,                        // write address
	                      &spi_get_hw(spi_default)->dr, // read address
	                      TEST_SIZE,                    // element count (each element is of size transfer_data_size)
	                      false                         // don't start yet
	                     );

	// TODO: Configure another DMA for the second ADS7043? Use same RX DMA?
	// Reconfigure RX DMA each sample to alternate between the two buffers for each ADS7043?

	// Configure a DMA timer to trigger 16 bit SPI transfers
	printf("Configure SPI Transaction Pace DMA\n");
	adc_dma_config.dma_pace_config = dma_channel_get_default_config(adc_dma_config.dma_pace);
	channel_config_set_transfer_data_size(&adc_dma_config.dma_pace_config, DMA_SIZE_16);
	channel_config_set_read_increment(&adc_dma_config.dma_pace_config, false);
	channel_config_set_write_increment(&adc_dma_config.dma_pace_config, false);
	// Configure timer to trigger at 200kHz given 250MHz system clock
	// 250MHz / 200kHz = 1250 cycles
	dma_timer_claim(0);
	dma_timer_set_fraction(0, 1, 1250); // 200kHz
	channel_config_set_dreq(&adc_dma_config.dma_pace_config, DREQ_DMA_TIMER0); // Timer 0
	dma_channel_configure(adc_dma_config.dma_pace, &adc_dma_config.dma_pace_config,
	                      &spi_get_hw(spi_default)->dr,    // write address
	                      &null_data,                      // read address
	                      TEST_SIZE,                       // element count
	                      false                            // don't start yet
	                     );

	/**
	 * TODO:
	 * Configure IRQ when RX complete to stop timer DMA
	 */

	dma_channel_set_irq0_enabled(adc_dma_config.dma_rx, true);
	// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
	irq_set_exclusive_handler(DMA_IRQ_0, adc_dma_config.frame_complete_handler);
	irq_set_enabled(DMA_IRQ_0, true);

	


}


void adc_dma_deinit() {
	// Free RX buffer
	free(adc_dma_config.rxbuf);

	// Release DMA channels
	dma_channel_unclaim(adc_dma_config.dma_pace);
	dma_channel_unclaim(adc_dma_config.dma_rx);
}


void _frame_complete_handler() {
	printf("DMA RX Complete!\n");
	// Stop timer DMA
	dma_channel_abort(adc_dma_config.dma_pace);

	// Process received data

}

bool _callback_start_frame_capture(__unused struct repeating_timer *t) {
	// printf("Starting DMA...\n");
	
	// Reset RX DMA
	dma_channel_set_write_addr(adc_dma_config.dma_rx, adc_dma_config.rxbuf, false);
	dma_channel_set_transfer_count(adc_dma_config.dma_rx, TEST_SIZE, false);

	// Reset timer DMA
	dma_channel_set_transfer_count(adc_dma_config.dma_pace, TEST_SIZE, false);
	
	// Start both DMAs
	dma_start_channel_mask((1u << adc_dma_config.dma_rx) | (1u << adc_dma_config.dma_pace));

	return true;
}





// Configure repeating timer to trigger at 20 Hz
// to call callback_capture_frame
void test_config_timer() {
	// Configure repeating timer to trigger at 20 Hz
	// Negative delay so means we will call repeating_timer_callback, and call
	// it again 50ms later regardless of how long the callback took to execute
	add_repeating_timer_ms(-50, _callback_start_frame_capture, NULL, &adc_dma_config.timer);
}




void test_DMA_wait() {
	printf("Wait for RX complete...\n");
	dma_channel_wait_for_finish_blocking(adc_dma_config.dma_rx);
	printf("RX complete!\n");
}






