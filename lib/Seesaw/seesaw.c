
#include "seesaw.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>



// NOTE: Replace these stubs with actual Pico SDK I2C code as needed.
static int seesaw_i2c_write(uint8_t addr, uint8_t *data, size_t len) {
	int ret = i2c_write_blocking(i2c1, addr, data, len, false);
	if (ret != len) {
		printf("I2C write error: %d\n", ret);
		return -1;
	}
	return 0;
}

static int seesaw_i2c_read(uint8_t addr, uint8_t *cmd, size_t cmdlen, uint8_t *buf, size_t buflen) {
	int ret = i2c_write_blocking(i2c1, addr, cmd, cmdlen, false);
	// int ret = i2c_write_blocking_until(i2c1, addr, cmd, cmdlen, false);
	if (ret != cmdlen) {
		printf("I2C write error: %d\n", ret);
		return ret;
	}
	sleep_us(250); // Recommended delay between write and read
	ret = i2c_read_blocking(i2c1, addr, buf, buflen, false);
	if (ret != buflen) {
		printf("I2C read error: %d\n", ret);
		return ret;
	}
	return 0;
}

static int seesaw_write_register(uint8_t i2c_addr, uint8_t module, uint8_t reg, uint8_t *data, size_t len) {
	uint8_t buf[2 + len];
	buf[0] = module;
	buf[1] = reg;
	memcpy(&buf[2], data, len);
	return seesaw_i2c_write(i2c_addr, buf, 2 + len);
}

static int seesaw_read_register(uint8_t i2c_addr, uint8_t module, uint8_t reg, uint8_t *buf, size_t len) {
	uint8_t cmd[2] = {module, reg};
	return seesaw_i2c_read(i2c_addr, cmd, 2, buf, len);
}


// int seesaw_init(Seesaw_t *seesaw) {
// 	if (!seesaw) return -1;
// 	// Initialize I2C here if needed
// 	i2c_init(seesaw->i2c_inst, seesaw->frequency); // 400kHz
// 	gpio_set_function(seesaw->pin_sda, GPIO_FUNC_I2C);
// 	gpio_set_function(seesaw->pin_scl, GPIO_FUNC_I2C);
// 	gpio_pull_up(seesaw->pin_sda);
// 	gpio_pull_up(seesaw->pin_scl);
// 	return 0;
// }



/**
 * ========================================
 * GPIO
 * ========================================
 */
int seesaw_gpio_pin_mode(uint8_t i2c_addr, uint8_t pin, uint8_t pinmode) {
	// Prepare bitmask for the specified pin
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};

	int ret;

	// Set output direction
	const bool output = (pinmode == SEESAW_OUTPUT);
	ret = seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, (output ? 0x02 : 0x03), buf, 4);
	if (ret != 0) return -1;

	// Enable pullup/pulldown and set pull direction
	if (pinmode & 0b10) {
		// Enable pullup/pulldown
		ret = seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, 0x0B, buf, 4);
		if (ret != 0) return -1;
		// Set pull direction
		ret = seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, ((pinmode & 0b01u) ? 0x06 : 0x05), buf, 4);
		if (ret != 0) return -1;
	} else if (pinmode == SEESAW_INPUT) {
		ret = seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, 0x0C, buf, 4);
		if (ret != 0) return -1;
	}

	return 0;
}

int seesaw_gpio_digital_write(uint8_t i2c_addr, uint8_t pin, bool value) {
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};
	int ret = seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, (value ? 0x05 : 0x06), buf, 4);
	if (ret != 0) return -1;
	return 0;
}

int seesaw_gpio_digital_read(uint8_t i2c_addr, uint8_t pin, bool *value) {
	uint8_t inbuf[4];
	int ret = seesaw_read_register(i2c_addr, SEESAW_GPIO_BASE, 0x04, inbuf, 4);
	if (ret == 0 && value) {
		uint32_t input = ((uint32_t)inbuf[0] << 24) | ((uint32_t)inbuf[1] << 16) | ((uint32_t)inbuf[2] << 8) | inbuf[3];
		*value = (input & ((uint32_t)1 << pin)) != 0;
	}
	return ret;
}

int seesaw_gpio_digital_read_bulk(uint8_t i2c_addr, uint32_t pins, uint32_t *value) {
	uint8_t inbuf[4];
	int ret = seesaw_read_register(i2c_addr, SEESAW_GPIO_BASE, 0x04, inbuf, 4);
	if (ret == 0 && value) {
		*value = pins & (((uint32_t)inbuf[0] << 24) | ((uint32_t)inbuf[1] << 16) | ((uint32_t)inbuf[2] << 8) | inbuf[3]);
	}
	return ret;
}

int seesaw_gpio_digital_write_bulk(uint8_t i2c_addr, uint32_t pins, uint8_t value) {
	// Prepare bitmask for the specified pins
	uint8_t buf[4] = {
		(uint8_t)(pins >> 24),
		(uint8_t)(pins >> 16),
		(uint8_t)(pins >> 8),
		(uint8_t)(pins >> 0)
	};
	
	// Write set or clear based on value
	if (value) {
		return seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, 0x05, buf, 4); // SET
	} else {
		return seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, 0x06, buf, 4); // CLR
	}
}

void seesaw_gpio_enable_interrupt(uint8_t i2c_addr, uint8_t pin) {
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};
	seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, 0x08, buf, 4);
}

void seesaw_gpio_disable_interrupt(uint8_t i2c_addr, uint8_t pin) {
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};
	seesaw_write_register(i2c_addr, SEESAW_GPIO_BASE, 0x09, buf, 4);
}



/**
 * ========================================
 * Rotary Encoder
 * ========================================
 */
int seesaw_encoder_get_position(uint8_t i2c_addr, uint8_t encoder, int32_t *position) {
	uint8_t inbuf[4];
	int ret = seesaw_read_register(i2c_addr, SEESAW_ENCODER_BASE, (uint8_t)(0x30 + encoder), inbuf, 4);
	if (ret == 0 && position) {
		*position = (int32_t)((inbuf[0] << 24) | (inbuf[1] << 16) | (inbuf[2] << 8) | inbuf[3]);
	}
	return ret;
}

int seesaw_encoder_get_delta(uint8_t i2c_addr, uint8_t encoder, int32_t *delta) {
	uint8_t inbuf[4];
	int ret = seesaw_read_register(i2c_addr, SEESAW_ENCODER_BASE, (uint8_t)(0x40 + encoder), inbuf, 4);
	if (ret == 0 && delta) {
		*delta = (int32_t)((inbuf[0] << 24) | (inbuf[1] << 16) | (inbuf[2] << 8) | inbuf[3]);
	}
	return ret;
}


void seesaw_encoder_enable_interrupt(uint8_t i2c_addr, uint8_t encoder) {
	uint8_t buf[1] = {1u}; // INTEN
	seesaw_write_register(i2c_addr, SEESAW_ENCODER_BASE, (uint8_t)(0x10 + encoder), &buf[0], 1);}

void seesaw_encoder_disable_interrupt(uint8_t i2c_addr, uint8_t encoder) {
	uint8_t buf[1] = {1u}; // INTDIS
	seesaw_write_register(i2c_addr, SEESAW_ENCODER_BASE, (uint8_t)(0x20 + encoder), &buf[0], 1);
}

void seesaw_encoder_set_position(uint8_t i2c_addr, uint8_t encoder, int32_t position) {
	uint8_t buf[4] = {
		(uint8_t)(position >> 24),
		(uint8_t)(position >> 16),
		(uint8_t)(position >> 8),
		(uint8_t)(position >> 0)
	};
	seesaw_write_register(i2c_addr, SEESAW_ENCODER_BASE, (0x30 + encoder), buf, 4);
}


/**
 * ========================================
 * Neopixel
 * ========================================
 */
int seesaw_neopixel_set_pixel(uint8_t i2c_addr, uint16_t n, uint32_t color) {
	uint8_t buf[6];
	buf[0] = SEESAW_NEOPIXEL_BASE;
	buf[1] = 0x03; // NEOPIXEL_BUF
	buf[2] = (n >> 8) & 0xFF;
	buf[3] = n & 0xFF;
	buf[4] = (color >> 16) & 0xFF;
	buf[5] = (color >> 8) & 0xFF;
	// Only send R, G, B (not W)
	uint8_t buf2[7] = {buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], (uint8_t)(color & 0xFF)};
	return seesaw_i2c_write(i2c_addr, buf2, 7);
}

int seesaw_neopixel_show(uint8_t i2c_addr) {
	uint8_t buf[2] = {SEESAW_NEOPIXEL_BASE, 0x05};
	return seesaw_i2c_write(i2c_addr, buf, 2);
}

int seesaw_neopixel_set_brightness(uint8_t i2c_addr, uint8_t brightness) {
	uint8_t buf[3] = {SEESAW_NEOPIXEL_BASE, 0x04, brightness};
	return seesaw_i2c_write(i2c_addr, buf, 3);
}
