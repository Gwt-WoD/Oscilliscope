#ifndef ADS7043_H
#define ADS7043_H

#include <stdint.h>


#include "hardware/spi.h"
#include "hardware/irq.h"



/**
 * There are three ways to calculate the required SPI clock frequency and
 * acquisition time for a desired sample rate:
 * 1. Calculate clock rate such that each sample period takes the maximum
 *    period of the sample rate, minus the minimum acquisition time.
 * 2. Fixed SPI clock frequency, calculate minumum sample time.
 * 
 */



#define ADS7043_DEFAULT_SAMPLE_RATE 200000 // 200kSPS
#define ADS7043_DEFAULT_SPI_CLK_FREQ_HZ (16*MHZ)
#define ADS7043_DEFAULT_SPI_CLK_FREQ_kHZ (ADS7043_DEFAULT_SPI_CLK_FREQ_HZ / 1000u)
#define ADS7043_DEFAULT_SPI_CLK_FREQ_MHZ (ADS7043_DEFAULT_SPI_CLK_FREQ_HZ / (1000u * 1000u))



/**
 * Bit:  0   1   2   3   4   5   6   7   8   9   10  11  12  13
 * Data: 0   0   D11 D10 D9  D8  D7  D6  D5  D4  D3  D2  D1  D0
 */

// TODO: ADC sends two 0 bits, then 12 bits of data, then zero bits for the remainder of the CS
#define ADS7043_DATA_MASK 0b0011111111111100

// #define US 0.000001f
// #define MS 0.001f

// Fixed constants from datasheet
// #define ADS7043_MAX_SPI_CLK_FREQ_KHZ (160000u) // 16 MHz
// #define ADS7043_MIN_SPI_CLK_FREQ_KHZ (16u)     // 16 kHz
#define ADS7043_MIN_ACQUISITION_TIME_NS (200u)     // 200 ns
// // #define ADS7043_MIN_ACQUISITION_TIME_SEC 20*US // 20 us
// #define ADS7043_MIN_CYCLE_TIME_US (1u)
// // #define ADS7043_MIN_CONVERSION_TIME_US ()


/** 
 * The minimum acquisition time in nanoseconds required by the ADS7043 is 200 ns.
 * The max SPI clock frequency is 16 MHz, so each clock cycle is 62.5 ns.
 * Therefore, at least 4 clock cycles are needed to satisfy the acquisition time.
 * 
 * The ADS7043 can sample data upto 1 MSPS.
 * Each sample requires 14 clock cycles (2 start bits + 12 data bits).
 * At 1 MSPS, the sample period is 1 us, which allows for up to 16 clock cycles at 16 MHz.
 * This leaves 2 clock cycles for acquisition time, which is insufficient.
 */

 /**
  * With two ADCs on a single SPI bus, we are limited to a maximum sample rate a little more
  * than 500 kSPS, since each sample requires 14 clock cycles, and the acquisition time 
  * occurs during the 14 clock cycles of the other ADC. With 14*2 = 28 clock cycles per sample
  * for two ADCs, the maximum sample rate is 16 MHz / 28 = ~571 kSPS.
  */



// Helper Macros For Calculations

/**
 * SPI can operate with data sizes of 4 to 16 bits.
 * max CLK freq. of the ADS7043 is 16 MHz
 * 
 * For example, at the maximum SSPCLK (clk_peri) frequency on RP2040 of 133MHz, the maximum peak bit rate in
 * master mode is 62.5Mbps. This is achieved with the SSPCPSR register programmed with a value of 2, and the SCR[7:0]
 * field in the SSPCR0 register programmed with a value of 0.
 * 
 * 
*/
// #define ADS7043_DATA_BITS 14 // 2 start bits + 12 data bits
// #define ADS7043_TOTAL_BITS_PER_SAMPLE (14 + ADS7043_CALC_ACQUISITION_CLK_CYCLES(ADS7043_MIN_ACQUISITION_TIME_NS/1000, ADS7043_DEFAULT_SPI_CLK_FREQ_MHZ)) // Total bits per sample including acquisition time
// // #define ADS7043_SPI_DATA_SIZE 4
// #define ADS7043_SPI_DATA_SIZE 16 // Using 16 bits per transfer for simplicity
// #define ADS7043_SPI_TRANSFERS_PER_SAMPLE(total_bits, data_sz) ((total_bits + data_sz - 1u) / data_sz)


// /**
//  * Calculate the required SPI clock frequency (in Hz) for a given sample rate (in samples per second)
//  * (Samples / Second) => (Seconds / Sample) = Period
//  * Period - ADS7043_MIN_ACQUISITION_TIME_US = Time available for clocking data
//  * (Time available for clocking data) / (2 + 12) (Clocks / Sample) = Clocks / Second (Hz)
//  */ 
// #define ADS7043_CALC_CLK_FREQ_HZ(sample_rate) ( sample_rate * 14 )

/**
 * Calculate the required acquisition time in clock cycles for a given SPI clock frequency (in Hz)
 * Freq (Cycles / Second) * Time (Seconds) = Cycles
 */
// #define ADS7043_CALC_ACQUISITION_CLK_CYCLES(acqu_time, freq) (acqu_time * freq)

// #define ADS7043_CALC_ACQUISITION_TIME_CLK_CYCLES_US_MHZ(acqu_time_us, freq_mhz) (acqu_time * freq_mhz)

// /**
//  * Calculate the number of bytes to read from the ADS7043 for a given sample rate.
//  */
// #define ADS7043_CALC_READ_BYTES(sample_rate) ( (ADS7043_MIN_ACQUISITION_TIME_US * sample_rate) / 1000000 + 2 )



typedef struct ads7043_inst {
	// SPI
	spi_inst_t *spi;
	uint8_t cs_pin;
	uint8_t sck_pin;
	uint8_t miso_pin;
	uint32_t spi_baudrate;

	// Acquisition
	uint32_t sample_rate;       // Samples per second
	uint32_t frame_length;      // Number of samples to read in each burst
	uint32_t frame_interval_us; // Interval between bursts in microseconds

	// Buffer
	uint16_t *data_buff;     // Pointer to data buffer
	uint32_t data_buff_size; // Size of data buffer in samples

	// DMA
	// int dma_channel;
	// uint8_t dma_irq;

} ads7043_inst_t;


ads7043_inst_t* ads7043_new(spi_inst_t *spi, uint8_t cs_pin, uint8_t sck_pin, uint8_t miso_pin, uint32_t baudrate);

void ads7043_init(ads7043_inst_t *inst);

uint16_t ads7043_read_raw(ads7043_inst_t *inst);

// uint8_t ads7043_config_dma(ads7043_inst_t *inst, uint32_t burst_length, uint32_t burst_interval_us);

// float ads7043_read_voltage(spi_inst_t *spi, uint8_t cs_pin, float vref);


// Stream N samples to provided memory address using DMA.
void ads7043_dma_stream_to_memory(ads7043_inst_t *inst, uint16_t *buffer, size_t sample_count);

void setup_dma_stream_to_memory_with_interrupt(uint16_t *buffer, size_t sample_count, int dma_irq_source, irq_handler_t handler_func);


#endif // ADS7043_H