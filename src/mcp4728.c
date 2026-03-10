
// Copyright 2021 Ocean (iiot2k@gmail.com) 
// All rights reserved.

#include <stdint.h>
#include <stdio.h>

#include "mcp4728.h"

#include "hardware/i2c.h"

#define MCP4728_ADDR 0x60
#define MULTIWRITE   0x40
#define SINGLEWRITE  0x58

static uint8_t dac_pd[] = { MCP4728_PD_OFF, MCP4728_PD_OFF, MCP4728_PD_OFF, MCP4728_PD_OFF };
// static uint8_t dac_vref[] = { MCP4728_VREF_INT, MCP4728_VREF_INT, MCP4728_VREF_INT, MCP4728_VREF_INT };
static uint8_t dac_vref[] = { MCP4728_VREF_VDD, MCP4728_VREF_VDD, MCP4728_VREF_VDD, MCP4728_VREF_VDD };
static uint8_t dac_gain[] = { MCP4728_GAIN2, MCP4728_GAIN2, MCP4728_GAIN2, MCP4728_GAIN2 };

uint32_t baud = 100*1000;


// #define I2C_TIMEOUT_US(baud) (uint)(1.5 * 9 * (1000000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 50% margin
// #define I2C_TIMEOUT_US(baud) (uint)(5 * 9 * (1000*1000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 500% margin
#define I2C_TIMEOUT_US(baud) (uint)(10 * 9 * (1000*1000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 1000% margin

// NOTE: Replace these stubs with actual Pico SDK I2C code as needed.
static int mcp4728_i2c_write(i2c_inst_t* inst, uint8_t addr, uint8_t *data, size_t len) {
	if (!data) return -1; // Check pointer
	// i2c_write_blocking returns number of bytes written, or PICO_ERROR_GENERIC
	// if address not acknowledged, no device present.
	// int ret = i2c_write_blocking(ss.i2c_inst, ss.i2c_addr, data, len, false);
	int ret = i2c_write_timeout_per_char_us(inst, addr, data, len, false, I2C_TIMEOUT_US(baud));
	if (ret != len) {
		if (ret == PICO_ERROR_GENERIC) {
			printf("[seesaw] I2C write error: device not acknowledged\n");
			return -2; // Device not acknowledged
		} else if (ret == PICO_ERROR_TIMEOUT) {
			printf("[seesaw] I2C write timeout\n");
			return -3; // I2C timeout
		} else if (ret < 0) {
			printf("[seesaw] I2C write error: %d\n", ret);
			return -3; // Other I2C error
		} else if (ret != len) {
			printf("[seesaw] I2C write error: incomplete write, %d of %d bytes written\n", ret, len);
			return -4; // Incomplete write
		} else {
			// Should be impossible unless more than len bytes were written?
			printf("[seesaw] I2C write error: unknown error code: %d\n", ret);
			return -5; // Unknown error
		}
	}
	return 0; // Success
}



void dev_mcp4728_pd(uint8_t ch, uint8_t pd)
{
	if ((ch >= MCP4728_CH_NUM) || (pd >= MCP4728_PD_NUM))
		return;

	dac_pd[ch] = pd;
}

void dev_mcp4728_vref(uint8_t ch, uint8_t vref)
{
	if ((ch >= MCP4728_CH_NUM) || (vref >= MCP4728_VREF_NUM))
		return;

	dac_vref[ch] = vref;
}

void dev_mcp4728_gain(uint8_t ch, uint8_t gain)
{
	if ((ch >= MCP4728_CH_NUM) || (gain >= MCP4728_GAIN_NUM))
		return;
		
	dac_gain[ch] = gain;
}

static bool mcp4728_write(i2c_inst_t* i2c, uint8_t cmd, uint8_t ch, uint16_t value)
{
	if (ch >= MCP4728_CH_NUM)
		return false;

	if (value > 4095)
		value = 4095;
	
	uint8_t buffer[3];

	buffer[0] = cmd | (ch << 1);
	buffer[1] = (value >> 8) | (dac_vref[ch] << 7) | (dac_pd[ch] << 5) | (dac_gain[ch] << 4);
	buffer[2] = value & 0xFF;

	// set channel dac
    return mcp4728_i2c_write(i2c, MCP4728_ADDR, buffer, sizeof(buffer));
}

bool dev_mcp4728_set(i2c_inst_t* i2c, uint8_t ch, uint16_t value)
{
	return mcp4728_write(i2c, MULTIWRITE, ch, value);
}

bool dev_mcp4728_save(i2c_inst_t* i2c, uint8_t ch, uint16_t value)
{
	return mcp4728_write(i2c, SINGLEWRITE, ch, value);
}
