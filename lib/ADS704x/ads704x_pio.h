#ifndef ADS704X_PIO_H
#define ADS704X_PIO_H

#include <stdio.h>
#include <stdint.h>

#include <pico/stdlib.h>

#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/regs/dreq.h>

// #include <ads704x_pio.pio.h> // auto-generated upon compilation.

typedef struct {
	PIO inst;
	int8_t offset;
	uint sm;
	pio_sm_config sm_cfg;
} ads704x_pio_t;

typedef struct {
	// spi_inst_t *inst;
	uint8_t sck;
	uint8_t miso;
	uint8_t cs;
	uint32_t spi_baudrate;
} ads704x_spi_t;

typedef struct {
	uint32_t data_chan;
	uint32_t ctrl_chan;
	dma_channel_config data_conf;
	dma_channel_config ctrl_conf;
	int timer;
	int irq;
} ads704x_dma_t;

typedef struct {
	uint8_t channel;
	ads704x_spi_t spi;
	ads704x_pio_t pio;
	ads704x_dma_t dma;

	uint32_t sample_rate;

	uint32_t buf_len;
	volatile uint16_t* buf; // Data that the reconfiguration channel will write back
									 // to the sample channel. In this case, just the
									 // address of the location of the adc samples. This
									 // value must exist with global scope since the DMA
									 // reconfiguration channel will need to writes its value
									 // back to the sample channel on regular intervals.
	// handler_func_t frame_complete_handler;
} ads704x_cfg_t;

void ads704x_init(ads704x_cfg_t *cfg, PIO pio, uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint32_t sample_rate, uint8_t channel);
void ads704x_setup_dma_stream_to_memory(ads704x_cfg_t *cfg, volatile uint16_t* starting_address, size_t sample_count);
// void ads704x_setup_dma_stream_to_memory(ads704x_cfg_t *cfg, volatile uint16_t* starting_address, size_t sample_count, uint32_t sample_rate);
// void ads704x_setup_dma_stream_to_memory_with_interrupt(ads704x_cfg_t *cfg, volatile uint16_t* starting_address, size_t sample_count, int dma_irq_source, irq_handler_t handler_func);
void ads704x_start(ads704x_cfg_t* cfg);
void ads704x_stop(ads704x_cfg_t* cfg);
// void ads704x_reset(ads704x_cfg_t *cfg);

uint32_t ads704x_get_dma_channel(ads704x_cfg_t *cfg);


// inline void clear_interrupt() {
// 	if (dma_irq_source_ == DMA_IRQ_0)
// 		dma_hw->ints0 = 1u << samp_chan_;
// 	else if (dma_irq_source_ == DMA_IRQ_1)
// 		dma_hw->ints1 = 1u << samp_chan_;
// }

#endif // ADS704X_PIO_H
