#ifndef DMA_TEST2_H
#define DMA_TEST2_H


#include <stdint.h>
#include "hardware/spi.h"
#include "hardware/dma.h"

#define TEST_SIZE 256

typedef enum {
	ADS704x_DMA_CH0 = 0b01,
	ADS704x_DMA_CH1 = 0b10,
	ADS704x_DMA_CH_BOTH = 0b11
} ADS704x_DMA_CHANNEL_t;

typedef struct {
	spi_inst_t *inst;
	uint8_t sck;
	uint8_t miso;
	uint8_t cs0;
	uint8_t cs1;
} ads704x_dma_spi_t;

typedef struct {
	uint8_t timer;
	uint8_t irq;

	uint32_t cs0_assert;
	dma_channel_config cs0_assert_cfg;

	uint32_t tx0;
	dma_channel_config tx0_cfg;

	uint32_t rx0;
	dma_channel_config rx0_cfg;

	uint32_t cs0_deassert;
	dma_channel_config cs0_deassert_cfg;

	uint32_t cs1_assert;
	dma_channel_config cs1_assert_cfg;

	uint32_t tx1;
	dma_channel_config tx1_cfg;

	uint32_t rx1;
	dma_channel_config rx1_cfg;

	uint32_t cs1_deassert;
	dma_channel_config cs1_deassert_cfg;
} _ads704x_dma_channel_t;

struct freq_div {
	uint16_t d;
	uint32_t freq;
};

extern const uint8_t dma_timer_settings_125MHz_len;
extern struct freq_div dma_timer_settings_125MHz[];
extern const uint8_t dma_timer_settings_200MHz_len;
extern struct freq_div dma_timer_settings_200MHz[];
extern const uint8_t dma_timer_settings_250MHz_len;
extern struct freq_div dma_timer_settings_250MHz[];
extern const uint8_t dma_timer_settings_260MHz_len;
extern struct freq_div dma_timer_settings_260MHz[];


void ads704x_dma_set_spi(spi_inst_t *spi, uint8_t sck_pin, uint8_t miso_pin, uint8_t cs0_pin, uint8_t cs1_pin);
void ads704x_dma_spi_init();

void ads704x_dma_set_buffers(uint16_t* buf0, uint16_t* buf1, size_t len);

int ads704x_dma_init();

uint32_t ads704x_dma_set_sample_rate(uint32_t sample_rate);
uint32_t ads704x_dma_get_sample_rate(void);

void ads704x_dma_start();
void ads704x_dma_stop();
bool ads704x_dma_is_running(void);
// bool ads704x_dma_is_finished(void);

void ads704x_dma_channel_enable(ADS704x_DMA_CHANNEL_t channel);
void ads704x_dma_channel_disable(ADS704x_DMA_CHANNEL_t channel);

void ads704x_dma_set_frame_complete_handler(void (*handler)(void));

void ads704x_dma_set_loop(bool loop);

#endif // DMA_TEST2_H