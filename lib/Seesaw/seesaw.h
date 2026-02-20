
#ifndef SEESAW_H
#define SEESAW_H

// Standard includes
#include <stdint.h>
#include <stdbool.h>

// Pico SDK includes
#include "hardware/i2c.h" // For I2C types

// Pin modes
#define SEESAW_INPUT          0b00u
#define SEESAW_OUTPUT         0b01u
#define SEESAW_INPUT_PULLUP   0b10u
#define SEESAW_INPUT_PULLDOWN 0b11u


// I2C address for Seesaw (default)
#define SEESAW_DEFAULT_ADDR 0x49


// typedef enum {
// 	SEESAW_STATUS_HW_ID = 0x01,
// 	SEESAW_STATUS_VERSION = 0x02,
// 	SEESAW_STATUS_OPTIONS = 0x03,
// } SeesawStatusReg;

typedef struct {
	uint8_t i2c_addr;
	i2c_inst_t* i2c_inst;
	uint32_t frequency;
	uint8_t pin_sda;
	uint8_t pin_scl;
} Seesaw_t;


// int seesaw_init(Seesaw_t *seesaw);

// Seesaw GPIO functions
int seesaw_gpio_pin_mode(uint8_t i2c_addr, uint8_t pin, uint8_t pinmode); // Works??
int seesaw_gpio_digital_write(uint8_t i2c_addr, uint8_t pin, bool value); // Works
int seesaw_gpio_digital_read(uint8_t i2c_addr, uint8_t pin, bool *value); // Works
void seesaw_gpio_enable_interrupt(uint8_t i2c_addr, uint8_t pin); // Works??
void seesaw_gpio_disable_interrupt(uint8_t i2c_addr, uint8_t pin); // Works??

int seesaw_gpio_digital_read_bulk(uint8_t i2c_addr, uint32_t pins, uint32_t *value); // Works??
int seesaw_gpio_digital_write_bulk(uint8_t i2c_addr, uint32_t pins, uint8_t value); // Works??

// Seesaw Rotary Encoder
int seesaw_encoder_get_position(uint8_t i2c_addr, uint8_t encoder_num, int32_t *position); // Works
int seesaw_encoder_get_delta(uint8_t i2c_addr, uint8_t encoder_num, int32_t *delta); // Works
void seesaw_encoder_enable_interrupt(uint8_t i2c_addr, uint8_t encoder_num);
void seesaw_encoder_disable_interrupt(uint8_t i2c_addr, uint8_t encoder_num);
void seesaw_encoder_set_position(uint8_t i2c_addr, uint8_t encoder_num, int32_t position); // Works

// Seesaw Neopixel
int seesaw_neopixel_set_pixel(uint8_t i2c_addr, uint16_t n, uint32_t color);
int seesaw_neopixel_show(uint8_t i2c_addr);
int seesaw_neopixel_set_brightness(uint8_t i2c_addr, uint8_t brightness);

// Utility: Convert RGB to packed 24-bit color
static inline uint32_t seesaw_color(uint8_t r, uint8_t g, uint8_t b) {
	return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}


/**
 * 
 */


/**
 * ========================================
 * Register and Module Definitions
 * ========================================
 */

// Seesaw module base addresses (see Adafruit Seesaw docs)
#define SEESAW_STATUS_BASE        0x00
#define SEESAW_GPIO_BASE          0x01
#define SEESAW_SERCOM0_BASE       0x02
#define SEESAW_SERCOM1_BASE       0x03
#define SEESAW_SERCOM2_BASE       0x04
#define SEESAW_SERCOM3_BASE       0x05
#define SEESAW_TIMER_BASE         0x08
#define SEESAW_ADC_BASE           0x09
#define SEESAW_DAC_BASE           0x0A
#define SEESAW_INTERRUPT_BASE     0x0B
#define SEESAW_DAP_BASE           0x0C
#define SEESAW_EEPROM_BASE        0x0D
#define SEESAW_NEOPIXEL_BASE      0x0E
#define SEESAW_TOUCH_BASE         0x0F
#define SEESAW_KEYPAD_BASE        0x10
#define SEESAW_ENCODER_BASE       0x11
#define SEESAW_SPECTRUM_BASE      0x12



/**
 * ========================================
 * Pin Definitions (ATtiny817)
 * ========================================
 */

enum SeesawPin {
	SEESAW_PIN_PA4 = 0,
	SEESAW_PIN_PA5,
	SEESAW_PIN_PA6,
	SEESAW_PIN_PA7,
	SEESAW_PIN_PB7,
	SEESAW_PIN_PB6,
	SEESAW_PIN_PB5,
	SEESAW_PIN_PB4,
	SEESAW_PIN_PB3,
	SEESAW_PIN_PB2,
	SEESAW_PIN_PB1,
	SEESAW_PIN_PB0,
	SEESAW_PIN_PC0,
	SEESAW_PIN_PC1,
	SEESAW_PIN_PC2,
	SEESAW_PIN_PC3,
	SEESAW_PIN_PC4,
	SEESAW_PIN_PC5,
	SEESAW_PIN_PA1,
	SEESAW_PIN_PA2,
	SEESAW_PIN_PA3
};


#define SEESAW_PIN_0 SEESAW_PIN_PA4
#define SEESAW_PIN_1 SEESAW_PIN_PA5
#define SEESAW_PIN_2 SEESAW_PIN_PA6
#define SEESAW_PIN_3 SEESAW_PIN_PA7
#define SEESAW_PIN_5 SEESAW_PIN_PB6
#define SEESAW_PIN_6 SEESAW_PIN_PB5
#define SEESAW_PIN_7 SEESAW_PIN_PB4
#define SEESAW_PIN_8 SEESAW_PIN_PB3
#define SEESAW_PIN_9 SEESAW_PIN_PB2
#define SEESAW_PIN_10 SEESAW_PIN_PB1
#define SEESAW_PIN_11 SEESAW_PIN_PB0
#define SEESAW_PIN_12 SEESAW_PIN_PC0
#define SEESAW_PIN_13 SEESAW_PIN_PC1
#define SEESAW_PIN_14 SEESAW_PIN_PC2
#define SEESAW_PIN_15 SEESAW_PIN_PC3
#define SEESAW_PIN_16 SEESAW_PIN_PC4
#define SEESAW_PIN_17 SEESAW_PIN_PC5
#define SEESAW_PIN_18 SEESAW_PIN_PA1
#define SEESAW_PIN_19 SEESAW_PIN_PA2
#define SEESAW_PIN_20 SEESAW_PIN_PA3


#define SEESAW_PIN_LED SEESAW_PIN_5
#define SEESAW_PIN_SCL SEESAW_PIN_11
#define SEESAW_PIN_SDA SEESAW_PIN_10
#define SEESAW_PIN_TXD SEESAW_PIN_9
#define SEESAW_PIN_RXD SEESAW_PIN_8



#endif // SEESAW_H
