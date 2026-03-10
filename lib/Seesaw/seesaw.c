
#include "seesaw.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void (*reset_callback)(void) = NULL;
uint8_t volatile errors = 0;
// #define I2C_SDA       24
// #define I2C_SCL       25

void seesaw_set_bus_reset_recovery_callback(void (*fn)(void)) {
	reset_callback = fn;
}
void _clear_bus(Seesaw_t ss) {
	// return;
	if (errors < 2) return; // TODO: TUNE
	printf("Reseting I2C Bus..."); stdio_flush();
	
	
	
	// uint8_t data = 0xFF;
	// i2c_write_blocking(ss.i2c_inst, 0xFF, &data, 1, false);

	// address
	// i2c_write_timeout_per_char_us(ss.i2c_inst, ss.i2c_addr, )
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_read_byte_raw(ss.i2c_inst);

	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_write_byte_raw(ss.i2c_inst, 0xFF);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// i2c_read_byte_raw(ss.i2c_inst);
	// errors = 0;
	// return;

	// uint8_t scl, sda;

	// for (uint i=0; i<30; i++) {
	// 	gpio_function_t current_func = gpio_get_function(i);
	// 	if (current_func == GPIO_FUNC_I2C) {
	// 		// (i % 4 < 2) ? "i2c0" : "i2c1"
	// 		if (!(i % 4 < 2) != i2c_get_index(ss.i2c_inst)) continue;
	// 		if (i % 2 == 0) sda = i; else scl = i;
	// 	}
	// }

	// if (scl == 0 && sda == 0) return; // ERROR

	const uint8_t scl = 25;
	const uint8_t sda = 24;

	i2c_deinit(ss.i2c_inst);
	// i2c_reser
	
	gpio_set_function(scl, GPIO_FUNC_SIO);
	gpio_set_function(sda, GPIO_FUNC_SIO);
	
	gpio_init(scl);
    gpio_init(scl);
    gpio_set_dir(scl, GPIO_OUT);
    gpio_set_dir(sda, GPIO_OUT);
	
	gpio_put(sda, 1);
    gpio_put(scl, 1);
	sleep_us(10);
    // 2. Toggle SCL 10-16 times to release stuck slave
    for (int i = 0; i < 16; i++) {
        // if (gpio_get(sda)) break; // Stop if SDA released
        gpio_put(scl, 0);
        sleep_us(10);
        gpio_put(scl, 1);
        sleep_us(10);
    }

    // 3. Re-initialize I2C pins
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);
    
    // 4. Re-init I2C hardware
    i2c_init(ss.i2c_inst, ss.baud);

	printf("Done.\n");
	errors = 0;

	if (reset_callback != NULL)
		reset_callback();
	//
}



// #define I2C_TIMEOUT_US(baud) (uint)(1.5 * 9 * (1000000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 50% margin
// #define I2C_TIMEOUT_US(baud) (uint)(5 * 9 * (1000*1000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 500% margin
#define I2C_TIMEOUT_US(baud) (uint)(10 * 9 * (1000*1000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 1000% margin
// #define I2C_TIMEOUT_US(baud) (uint)((100 * 9 * 1000*1000) / baud) // 9 bits per transfer (8 data + 1 ack), with 10,000% margin
// #define I2C_TIMEOUT_US(baud) (uint)(1000*1000) // 1 Second Timeout
// #define I2C_TIMEOUT_US_2(baud, len) (uint)(len * 10 * 9 * (1000*1000 / baud)) // 9 bits per transfer (8 data + 1 ack), with 1000% margin

// NOTE: Replace these stubs with actual Pico SDK I2C code as needed.
static int seesaw_i2c_write(Seesaw_t ss, uint8_t *data, size_t len) {
	if (!data) return -1; // Check pointer
	_clear_bus(ss);
	// i2c_write_blocking returns number of bytes written, or PICO_ERROR_GENERIC
	// if address not acknowledged, no device present.
	// int ret = i2c_write_blocking(ss.i2c_inst, ss.i2c_addr, data, len, false);
	int ret = i2c_write_timeout_per_char_us(ss.i2c_inst, ss.i2c_addr, data, len, false, I2C_TIMEOUT_US(ss.baud));
	// int ret = i2c_write_timeout_us(ss.i2c_inst, ss.i2c_addr, data, len, false, I2C_TIMEOUT_US(ss.baud, len));
	if (ret != len) {
		if (ret == PICO_ERROR_GENERIC) {
			errors++;
			printf("[seesaw] I2C write error: device not acknowledged\n");
			return -2; // Device not acknowledged
		} else if (ret == PICO_ERROR_TIMEOUT) {
			errors++;
			printf("[seesaw] I2C write timeout\n");
			return -3; // I2C timeout
		} else if (ret < 0) {
			errors++;
			printf("[seesaw] I2C write error: %d\n", ret);
			return -3; // Other I2C error
		} else if (ret != len) {
			errors++;
			printf("[seesaw] I2C write error: incomplete write, %d of %d bytes written\n", ret, len);
			return -4; // Incomplete write
		} else {
			// Should be impossible unless more than len bytes were written?
			errors++;
			printf("[seesaw] I2C write error: unknown error code: %d\n", ret);
			return -5; // Unknown error
		}
	}
	errors = 0;
	return 0; // Success
}

static int seesaw_i2c_read(Seesaw_t ss, uint8_t *cmd, size_t cmdlen, uint8_t *buf, size_t buflen) {
	if (!buf || !cmd) return -1; // Check pointer
	_clear_bus(ss);
	// int ret = i2c_write_timeout_per_char_us(ss.i2c_inst, ss.i2c_addr, cmd, cmdlen, false, I2C_TIMEOUT_US(ss.baud));
	int ret = i2c_write_timeout_us(ss.i2c_inst, ss.i2c_addr, cmd, cmdlen, false, I2C_TIMEOUT_US(ss.baud));
	if (ret != cmdlen) {
		if (ret == PICO_ERROR_GENERIC) {
			errors++;
			printf("[seesaw] I2C write error: device not acknowledged\n");
			return -2; // Device not acknowledged
		} else if (ret == PICO_ERROR_TIMEOUT) {
			errors++;
			printf("[seesaw] I2C write timeout\n");
			return -3; // I2C timeout
		} else if (ret < 0) {
			errors++;
			printf("[seesaw] I2C write error: %d\n", ret);
			return -3; // Other I2C error
		} else if (ret != cmdlen) {
			errors++;
			printf("[seesaw] I2C write error: incomplete write, %d of %d bytes written\n", ret, cmdlen);
			return -4; // Incomplete write
		} else {
			// Should be impossible unless more than len bytes were written?
			errors++;
			printf("[seesaw] I2C write error: unknown error code: %d\n", ret);
			return -5; // Unknown error
		}
	}
	errors = 0;
	sleep_us(250); // Recommended delay between write and read
	// sleep_us(500); // Min delay for ADC
	// sleep_us(350); // Lets be safe
	// sleep_us(1000); // Test
	// sleep_us(5000); // Test
	ret = i2c_read_timeout_per_char_us(ss.i2c_inst, ss.i2c_addr, buf, buflen, false, I2C_TIMEOUT_US(ss.baud));
	if (ret != buflen) {
		if (ret == PICO_ERROR_GENERIC) {
			errors++;
			printf("[seesaw] I2C read error: device not acknowledged\n");
			return -2; // Device not acknowledged
		} else if (ret == PICO_ERROR_TIMEOUT) {
			errors++;
			printf("[seesaw] I2C read timeout\n");
			return -3; // I2C timeout
		} else if (ret < 0) {
			errors++;
			printf("[seesaw] I2C read error: %d\n", ret);
			return -3; // Other I2C error
		} else if (ret != buflen) {
			errors++;
			printf("[seesaw] I2C read error: incomplete read, %d of %d bytes read\n", ret, buflen);
			return -4; // Incomplete read
		} else {
			// Should be impossible unless more than len bytes were written?
			errors++;
			printf("[seesaw] I2C read error: unknown error code %d\n", ret);
			return -5; // Unknown error
		}
	}
	errors = 0;
	return 0; // Success
}

static int seesaw_write_register(Seesaw_t ss, uint8_t module, uint8_t reg, uint8_t *data, size_t len) {
	if (!data && len != 0) return -1; // Check pointer
	uint8_t buf[2 + len];
	buf[0] = module;
	buf[1] = reg;
	if (len != 0)
		memcpy(&buf[2], data, len);
	return seesaw_i2c_write(ss, buf, 2 + len);
}

static int seesaw_read_register(Seesaw_t ss, uint8_t module, uint8_t reg, uint8_t *buf, size_t len) {
	if (!buf) return -1; // Check pointer
	uint8_t cmd[2] = {module, reg};
	return seesaw_i2c_read(ss, cmd, 2, buf, len);
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
 * Status
 * ========================================
 */
int seesaw_swrst(Seesaw_t ss) {
	// uint8_t buf[1] = {1u}; // INTEN
	return seesaw_write_register(ss, SEESAW_STATUS_BASE, (uint8_t)(0x7F), (uint8_t*)NULL, 0);
}



/**
 * ========================================
 * GPIO
 * ========================================
 */
int seesaw_gpio_pin_mode(Seesaw_t ss, uint8_t pin, uint8_t pinmode) {
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
	ret = seesaw_write_register(ss, SEESAW_GPIO_BASE, (output ? 0x02 : 0x03), buf, 4);
	if (ret != 0) return -1;

	// Enable pullup/pulldown and set pull direction
	if (pinmode & 0b10) {
		// Enable pullup/pulldown
		ret = seesaw_write_register(ss, SEESAW_GPIO_BASE, 0x0B, buf, 4);
		if (ret != 0) return -1;
		// Set pull direction
		ret = seesaw_write_register(ss, SEESAW_GPIO_BASE, ((pinmode & 0b01u) ? 0x06 : 0x05), buf, 4);
		if (ret != 0) return -1;
	} else if (pinmode == SEESAW_INPUT) {
		ret = seesaw_write_register(ss, SEESAW_GPIO_BASE, 0x0C, buf, 4);
		if (ret != 0) return -1;
	}

	return 0;
}

int seesaw_gpio_digital_write(Seesaw_t ss, uint8_t pin, bool value) {
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};
	int ret = seesaw_write_register(ss, SEESAW_GPIO_BASE, (value ? 0x05 : 0x06), buf, 4);
	if (ret != 0) return -1;
	return 0;
}

int seesaw_gpio_digital_read(Seesaw_t ss, uint8_t pin, bool *value) {
	if (!value) return -1; // Check pointer
	uint8_t inbuf[4];
	int ret = seesaw_read_register(ss, SEESAW_GPIO_BASE, 0x04, inbuf, 4);
	if (ret == 0 && value) {
		uint32_t input = ((uint32_t)inbuf[0] << 24) | ((uint32_t)inbuf[1] << 16) | ((uint32_t)inbuf[2] << 8) | inbuf[3];
		if (input & ~((1u<<22)-1u)) {
			printf("ERROR: GPIO >20 set\n");
			return -2; // Non-existant pins are high, data must be bad
		}
		*value = (input & ((uint32_t)1 << pin)) != 0;
	}
	return ret;
}

int seesaw_gpio_digital_read_bulk(Seesaw_t ss, uint32_t pins, uint32_t *value) {
	if (!value) return -1; // Check pointer
	uint8_t inbuf[4];
	int ret = seesaw_read_register(ss, SEESAW_GPIO_BASE, 0x04, inbuf, 4);
	if (ret == 0 /*&& value*/) {
		*value = pins & (((uint32_t)inbuf[0] << 24) | ((uint32_t)inbuf[1] << 16) | ((uint32_t)inbuf[2] << 8) | inbuf[3]);
	}
	return ret;
}

int seesaw_gpio_digital_write_bulk(Seesaw_t ss, uint32_t pins, uint8_t value) {
	// Prepare bitmask for the specified pins
	uint8_t buf[4] = {
		(uint8_t)(pins >> 24),
		(uint8_t)(pins >> 16),
		(uint8_t)(pins >> 8),
		(uint8_t)(pins >> 0)
	};
	
	// Write set or clear based on value
	if (value) {
		return seesaw_write_register(ss, SEESAW_GPIO_BASE, 0x05, buf, 4); // SET
	} else {
		return seesaw_write_register(ss, SEESAW_GPIO_BASE, 0x06, buf, 4); // CLR
	}
}

int seesaw_gpio_enable_interrupt(Seesaw_t ss, uint8_t pin) {
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};
	return seesaw_write_register(ss, SEESAW_GPIO_BASE, 0x08, buf, 4);
}

int seesaw_gpio_disable_interrupt(Seesaw_t ss, uint8_t pin) {
	uint32_t data = (1ul << pin);
	uint8_t buf[4] = {
		(uint8_t)(data >> 24),
		(uint8_t)(data >> 16),
		(uint8_t)(data >> 8),
		(uint8_t)(data >> 0)
	};
	return seesaw_write_register(ss, SEESAW_GPIO_BASE, 0x09, buf, 4);
}

int seesaw_gpio_get_interupts(Seesaw_t ss, uint32_t *value) {
	if (!value) return -1; // Check pointer
	uint8_t inbuf[4];
	int ret = seesaw_read_register(ss, SEESAW_GPIO_BASE, 0x0A, inbuf, 4);
	if (ret == 0) {
		*value = ((uint32_t)inbuf[0] << 24) | ((uint32_t)inbuf[1] << 16) | ((uint32_t)inbuf[2] << 8) | inbuf[3];
	}
	return ret;
}


/**
 * ========================================
 * Rotary Encoder
 * ========================================
 */
int seesaw_encoder_get_position(Seesaw_t ss, uint8_t encoder, int32_t *position) {
	if (!position) return -1; // Check pointer
	uint8_t inbuf[4];
	int ret = seesaw_read_register(ss, SEESAW_ENCODER_BASE, (uint8_t)(0x30 + encoder), inbuf, 4);
	if (ret == 0 && position) {
		*position = (int32_t)((inbuf[0] << 24) | (inbuf[1] << 16) | (inbuf[2] << 8) | inbuf[3]);
	}
	return ret;
}

int seesaw_encoder_get_delta(Seesaw_t ss, uint8_t encoder, int32_t *delta) {
	if (!delta) return -1; // Check pointer
	uint8_t inbuf[4];
	int ret = seesaw_read_register(ss, SEESAW_ENCODER_BASE, (uint8_t)(0x40 + encoder), inbuf, 4);
	if (ret == 0 && delta) {
		int32_t t = (int32_t)((inbuf[0] << 24) | (inbuf[1] << 16) | (inbuf[2] << 8) | inbuf[3]);
		if (t > 20 || t < -20) *delta = 0;
		else *delta = t;
		
	}
	return ret;
}


int seesaw_encoder_enable_interrupt(Seesaw_t ss, uint8_t encoder) {
	uint8_t buf[1] = {1u}; // INTEN
	return seesaw_write_register(ss, SEESAW_ENCODER_BASE, (uint8_t)(0x10 + encoder), &buf[0], 1);
}


int seesaw_encoder_disable_interrupt(Seesaw_t ss, uint8_t encoder) {
	uint8_t buf[1] = {1u}; // INTDIS
	return seesaw_write_register(ss, SEESAW_ENCODER_BASE, (uint8_t)(0x20 + encoder), &buf[0], 1);
}

int seesaw_encoder_set_position(Seesaw_t ss, uint8_t encoder, int32_t position) {
	uint8_t buf[4] = {
		(uint8_t)(position >> 24),
		(uint8_t)(position >> 16),
		(uint8_t)(position >> 8),
		(uint8_t)(position >> 0)
	};
	return seesaw_write_register(ss, SEESAW_ENCODER_BASE, (uint8_t)(0x30 + encoder), buf, 4);
}


/**
 * ========================================
 * Neopixel
 * ========================================
 */

int seesaw_neopixel_set_pin(Seesaw_t ss, uint8_t pin) {
	uint8_t buf[1];
	buf[0] = pin;
	return seesaw_write_register(ss, SEESAW_NEOPIXEL_BASE, (uint8_t)(0x01), buf, 1);
}

int seesaw_neopixel_set_len(Seesaw_t ss, uint16_t n) {
	uint8_t buf[2];
	buf[0] = (n >> 8) & 0xFF;
	buf[1] = n & 0xFF;
	return seesaw_write_register(ss, SEESAW_NEOPIXEL_BASE, (uint8_t)(0x03), buf, 2);
}

int seesaw_neopixel_set_pixel(Seesaw_t ss, uint16_t n, uint32_t color) {
	// uint8_t buf[6];
	// buf[0] = SEESAW_NEOPIXEL_BASE;
	// buf[1] = 0x03; // NEOPIXEL_BUF
	// buf[2] = (n >> 8) & 0xFF;
	// buf[3] = n & 0xFF;
	// buf[4] = (color >> 16) & 0xFF;
	// buf[5] = (color >> 8) & 0xFF;
	// // Only send R, G, B (not W)
	// uint8_t buf2[7] = {buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], (uint8_t)(color & 0xFF)};
	// return seesaw_i2c_write(ss, buf2, 7);
	uint8_t buf[5];
	buf[0] = (n >> 8) & 0xFF;
	buf[1] = n & 0xFF;
	buf[2] = (color >> 16) & 0xFF;
	buf[3] = (color >> 8) & 0xFF;
	buf[4] = color & 0xFF;
	// Only send R, G, B (not W)
	// uint8_t buf2[7] = {buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], (uint8_t)(color & 0xFF)};
	return seesaw_write_register(ss, SEESAW_NEOPIXEL_BASE, (uint8_t)(0x04), buf, 5);
}

int seesaw_neopixel_show(Seesaw_t ss) {
	// uint8_t buf[2] = {SEESAW_NEOPIXEL_BASE, 0x05};
	// return seesaw_i2c_write(ss, buf, 2);
	return seesaw_write_register(ss, SEESAW_NEOPIXEL_BASE, (uint8_t)(0x05), NULL, 0);
}

int seesaw_neopixel_set_brightness(Seesaw_t ss, uint8_t brightness) {
	// uint8_t buf[3] = {SEESAW_NEOPIXEL_BASE, 0x04, brightness};
	// return seesaw_i2c_write(ss, buf, 3);
	return seesaw_write_register(ss, SEESAW_NEOPIXEL_BASE, (uint8_t)(0x04), &brightness, 1);
}
