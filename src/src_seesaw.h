#ifndef SRC_SEESAW_H
#define SRC_SEESAW_H

#include <stdint.h>
#include <stdbool.h>

#include "stdlib.h"
#include <stdio.h>
#include "time.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "seesaw.h"

#include "pins.h" // Pin mappings

Seesaw_t ss = {
    .i2c_inst = I2C_PORT,
    .i2c_addr = SEESAW_DEFAULT_ADDR
};

typedef struct {
    uint32_t decade;   // 1, 10, 100, 1000, ...
    uint8_t digit;     // 1–9
} LogValue_t;


typedef struct {
	uint8_t btn_pin; // Pin on Seesaw MCU, not this one
	int32_t position;
	int32_t delta;
	bool btn;

	LogValue_t value;
	// int32_t* var;      // Pointer to value

	int32_t lim_upper;
	uint32_t lim_lower;

} enc_t;


const uint8_t enc_cnt = 3; // SET ME TO THE NUMBER OF ENCODERS YOU HAVE
// const uint8_t button_pins[] = {9, 18, 12}; // Pins on Seesaw MCU, not this one

enc_t encoders[3];
// enc_t encoders[enc_cnt] = {
// 	{
// 		.btn_pin = 9,
// 		.position = 0,
// 		.delta = 0,
// 		.btn = 0
// 	},
// 	{
// 		.btn_pin = 18,
// 		.position = 0,
// 		.delta = 0,
// 		.btn = 0
// 	},
// 	{
// 		.btn_pin = 12,
// 		.position = 0,
// 		.delta = 0,
// 		.btn = 0
// 	}
// };





// bool buttons[enc_cnt];
// int32_t position[enc_cnt];
// int32_t position_calc[enc_cnt];
// int32_t delta[enc_cnt];


// Sample the user input at least once every 100ms. 40ms allows for multiple
// samples incase one is missed, taking into account the time it takes for
// each sample.
// A better solution would be to use a retry mechanism so a single failed 
// read doesn't cause the current sample to be aborted.
const absolute_time_t delay_us = (40*1000); // 40ms
absolute_time_t alarm_time;


void seesaw_setup() {
	// User Input I2C Initialisation. Using it at 400Khz.
	printf("Initialising I2C...\n");
	// Returns actual baudrate which may differ from requested baud
	// ss.baud = i2c_init(I2C_PORT, 400*1000); // 400kHz
	ss.baud = i2c_init(I2C_PORT, 100*1000); // 400kHz
	/**
	 * NOTE: We may need stronger pull-ups for higher baud rates (2.2k for 400kHz)
	 */
	printf("I2C baudrate set to: %d Hz\n", ss.baud);
	gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_SDA);
	gpio_pull_up(I2C_SCL);

	// Configure seesaw
	printf("Configuring Seesaw...\n");

	encoders[0].btn_pin = 9;
	encoders[1].btn_pin = 18;
	encoders[2].btn_pin = 12;


	while(seesaw_gpio_pin_mode(ss, SEESAW_PIN_LED, SEESAW_OUTPUT)); // Set pin 5 (onboard LED) as output
	seesaw_gpio_digital_write_bulk(ss, (1ul << SEESAW_PIN_LED), 1); // Turn off onboard LED

	for (int i = 0; i < enc_cnt; i++) {
		enc_t* _enc = &(encoders[i]);
		printf("Configuring encoder %d and button pin %d...\n", i, _enc->btn_pin);
		printf("Enabling encoder interrupt...\n");
		while(seesaw_encoder_enable_interrupt(ss, i)); // Enable interrupt for encoder
		printf("\tSetting encoder position to 0...\n");
		while(seesaw_encoder_set_position(ss, i, 0)); // Reset encoder position to 0
		printf("\tSetting button pin mode...\n");
		while(seesaw_gpio_pin_mode(ss, _enc->btn_pin, SEESAW_INPUT_PULLUP)); // Set button pins as input with pull-up

		_enc->delta = 0;
		_enc->position = 0;
		_enc->btn = false;
		_enc->lim_lower = 1;
		_enc->lim_upper = 0;
	}

	// H Scale
	while(seesaw_encoder_set_position(ss, 0, 20*1000/500)); // Reset encoder position to 20000
	// V Scale
	while(seesaw_encoder_set_position(ss, 1, 1000/100)); // Reset encoder position to 1000
	// Trigger
	while(seesaw_encoder_set_position(ss, 2, 1000/100)); // Reset encoder position to 1000


	alarm_time = get_absolute_time() + delay_us;
}


void seesaw_run() {
	// Read user input periodically without using blocking sleep
	// In the future we will have interrupts for this
	if (time_reached(alarm_time)) {
		absolute_time_t old = alarm_time;
		alarm_time = get_absolute_time() + delay_us;

		/**
		 * Note for Paul
		 * There are two ways to read the encoder position:
		 * 1. Read the delta (change in position) since the last read
		 * 2. Read the absolute position that is tracked by the Seesaw MCU
		 * The first method gives you more control over the absolute value
		 * of the position (Like setting upper and lower limits without
		 * having to send seesaw commands to manually adjust the position)
		 * The second method is less prone to missing user input.
		 * 
		 * NOTE: If you call seesaw_encoder_get_position() it will count as
		 * a read and reset the delta value to zero. If you want to use both
		 * delta and abs. position, you need to read the delta first.
		 */

		for (int i = 0; i < enc_cnt; i++) {
			enc_t* _enc = &(encoders[i]);
			int32_t d = 0;
			if (seesaw_encoder_get_delta(ss, i, &d)) {
				printf("Error reading encoder delta\n");
			} else {
				_enc->delta += d;
			}
			// if (seesaw_encoder_get_position(ss, i, &(_enc->position)) != 0) {
			// 	// printf("Error reading encoder position\n");
			// }
			// // Change the data
			// if (_enc->position <= 0) {
			// 	encoders[i].position = 1;
			// 	if (seesaw_encoder_set_position(ss, i, 1)) {
			// 		// printf("Error setting encoder position\n");
			// 	}
			// }
			// sleep_us(250); // Small delay to avoid overloading the seesaw
			if (seesaw_gpio_digital_read(ss, _enc->btn_pin, &(_enc->btn)) != 0) {
				// printf("Error reading button state\n");
			}
			// sleep_us(250); // Small delay to avoid overloading the seesaw
		}

		// HorizontalScale = encoders[0].position*500;
		// VerticalScale = encoders[1].position*100;
		// TriggerVoltage = encoders[2].position*100;

		// printf("Time: %ld us    Overshoot: %ld us\t", (uint32_t)get_absolute_time(), (uint32_t)absolute_time_diff_us(old, get_absolute_time()));
		// for (int i = 0; i < num_enc; i++) {
		//     // printf("Enc %d: Pos Calc = %03d, Pos = %03d, Delta = %03d    ", i, position_calc[i], position[i], delta[i]);
		//     printf("Enc %d Pos: %04ld Btn: %d    ", i, position[i], !buttons[i]);
		//     // printf("%04ld (%01u)   ", position[i], !buttons[i]);
		//     stdio_flush();
		//     // printf("Enc %d Pos: %03d    ", i, position_calc[i]);
		// }
		// // printf("VS: %u", VerticalScale);
		// printf("\n");
	}
}

// Updates the provided value pointed to by var using the internally tracked
// values
// void seesaw_update_val(uint8_t enc, int32_t* var) {
// 	if (enc >= enc_cnt)
// 		return;
// 	LogValue_t* v = &(encoders[enc].value);
// 	*var = v->digit * v->decade;
// }

void seesaw_update_val(uint8_t enc, int32_t* var) {
	if (enc >= enc_cnt && !var)
		return;
	*var += encoders[enc].delta;
	encoders[enc].delta = 0;
}


// void seesaw_update_val_from(uint8_t enc, int32_t* var) {
// 	if (enc >= enc_cnt)
// 		return;
// 	LogValue_t* v = &(encoders[enc].value);
// 	*var = v->digit * v->decade;
// }

void seesaw_set_val(uint8_t enc, int32_t var) {

}



void encoder_step(LogValue_t *log, int32_t delta) {
    if (delta > 0) { // clockwise
        if (log->digit < 9) {
            log->digit++;
        }
        else {
            log->digit = 1;
            log->decade *= 10;
        }
    }
    else { // counter-clockwise
        if (log->digit > 1) {
            log->digit--;
        }
        else {
            if (log->decade > 1) {
                log->decade /= 10;
                log->digit = 9;
            }
        }
    }
}

// void encoder_step_from(LogValue_t *log, int32_t delta, int32_t val) {
//     if (delta > 0) { // clockwise
//         if (log->digit < 9) {
//             log->digit++;
//         }
//         else {
//             log->digit = 1;
//             log->decade *= 10;
//         }
//     }
//     else { // counter-clockwise
//         if (log->digit > 1) {
//             log->digit--;
//         }
//         else {
//             if (log->decade > 1) {
//                 log->decade /= 10;
//                 log->digit = 9;
//             }
//         }
//     }
// }

// void encoder_step_new(int32_t* val, int32_t delta) {
// 	int32_t r = *val % 10;
// 	int32_t v = *val / 10;
// 	while (!r && ) { // While no remainder and
// 		r = v % 10;
// 		v = v / 10;
// 	}
// 	if (v > 10) {

// 	}
// }



#endif