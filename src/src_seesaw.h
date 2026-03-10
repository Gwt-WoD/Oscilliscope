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

Seesaw_t ss0 = {
    .i2c_inst = I2C_PORT,
    .i2c_addr = 0x49
    // .i2c_addr = SEESAW_DEFAULT_ADDR
};
Seesaw_t ss1 = {
    .i2c_inst = I2C_PORT,
    .i2c_addr = 0x4A
    // .i2c_addr = SEESAW_DEFAULT_ADDR
};

typedef struct {
    uint32_t decade;   // 1, 10, 100, 1000, ...
    uint8_t digit;     // 1–9
} LogValue_t;

typedef struct {
    uint32_t max;
    uint32_t step;
} ValueRange_t;

typedef struct {
    // 0b00: unsigned
    // 0b01: signed
    // 0b11: signed symmetric
    // uint8_t s;

    uint32_t value;
    uint32_t reset;
    uint32_t min;
    size_t ranges_cnt;
    ValueRange_t* ranges;
} DynamicValue_t;

// Uses encoder output as index to array of preset values
typedef struct {
    uint32_t i;
    uint32_t reset; // i value to reset/default to
    uint32_t num_values;
    int32_t* values; // change to void pointer for any data type?
} MappedValue_t;

typedef struct {
    Seesaw_t* ss;
    uint8_t n;
    uint8_t btn_pin; // Pin on Seesaw MCU, not this one

    int32_t position;
    int32_t delta;
    bool btn;

    // LogValue_t value;
    // DynamicValue_t value;
    // int32_t* var;      // Pointer to value

    int32_t lim_upper;
    int32_t lim_lower;

} enc_t;


enum {
    ENC_VSCL,
    // ENC_VSHFT,
    ENC_HSCL,
    // ENC_HSHFT,
    ENC_TRIG,
    // ENC_USR,
    ENC_CNT
};

const uint8_t enc_cnt = ENC_CNT; // SET ME TO THE NUMBER OF ENCODERS YOU HAVE
// const uint8_t button_pins[] = {9, 18, 12}; // Pins on Seesaw MCU, not this one

const size_t DecadeRange_cnt = 10;
const ValueRange_t DecadeRange[] = {
    {.max = 10, .step = 1},
    {.max = 100, .step = 10},
    {.max = 1000, .step = 100},
    {.max = 10 * 1000, .step = 1000},
    {.max = 100 * 1000, .step = 10 * 1000},
    {.max = 1000 * 1000, .step = 100 * 1000},
    {.max = 10 * 1000 * 1000, .step = 1000 * 1000},
    {.max = 100 * 1000 * 1000, .step = 10 * 1000 * 1000},
    {.max = 1000 * 1000 * 1000, .step = 100 * 1000 * 1000},
    {.max = 2 * 1000 * 1000 * 1000, .step = 1000 * 1000 * 1000}
};



enc_t encoders[ENC_CNT];
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

enum {
    BTN_START,
    BTN_SINGLE,
    BTN_BACK,
    BTN_EXIT,
    BTN_CH1,
    BTN_CH2,
    BTN_CNT
};

const uint8_t btn_pins[BTN_CNT] = {
    8,  // BTN0 PIN_PB3 8
    9,  // BTN1 PIN_PB2 9
    18, // BTN2 PIN_PA1 18
    0,  // BTN3 PIN_PA4 0
    19, // BTN4 PIN_PA2 19
    20, // BTN5 PIN_PA3 20
};

bool buttons[BTN_CNT];

// bool buttons[enc_cnt];
// int32_t position[enc_cnt];
// int32_t position_calc[enc_cnt];
// int32_t delta[enc_cnt];


// Sample the user input at least once every 100ms. 40ms allows for multiple
// samples incase one is missed, taking into account the time it takes for
// each sample.
// A better solution would be to use a retry mechanism so a single failed 
// read doesn't cause the current sample to be aborted.
const absolute_time_t delay_us = (33*1000); // 33ms
absolute_time_t alarm_time;

void seesaw_reset();

void seesaw_setup() {

    // User Input I2C Initialisation. Using it at 400Khz.
    printf("Initialising I2C...\n");
    // Returns actual baudrate which may differ from requested baud
    // ss0.baud = i2c_init(I2C_PORT, 400*1000); // 400kHz
    // ss0.baud = i2c_init(I2C_PORT, 200*1000); // 200kHz
    // ss0.baud = i2c_init(I2C_PORT, 100*1000); // 100kHz
    // ss0.baud = i2c_init(I2C_PORT, 50*1000); // 50kHz
    // ss0.baud = i2c_init(I2C_PORT, 25*1000); // 25kHz
    // ss0.baud = i2c_init(I2C_PORT, 20*1000); // 20kHz
    // ss0.baud = i2c_init(I2C_PORT, 12*1000); // 16kHz
    ss0.baud = i2c_init(I2C_PORT, 10*1000); // 10kHz
    ss1.baud = ss0.baud;
    /**
     * NOTE: We may need stronger pull-ups for higher baud rates (2.2k for 400kHz)
     */
    printf("I2C baudrate set to: %d Hz\n", ss0.baud);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    gpio_set_drive_strength(I2C_SDA, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C_SCL, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    gpio_set_drive_strength(I2C_SDA, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C_SCL, GPIO_DRIVE_STRENGTH_12MA);
    

    // Configure seesaw
    printf("Configuring Seesaw...\n");

    // When the i2c bus is reset after a lock up,
    // reset the seesaw devices and reconfigure them because they sometimes stay
    // in a bad state
    seesaw_set_bus_reset_recovery_callback(seesaw_reset);

    // Software Reset
    seesaw_swrst(ss0);
    seesaw_swrst(ss1);

    // encoders[0].btn_pin = 9;
    // encoders[1].btn_pin = 18;
    // encoders[2].btn_pin = 12;
    encoders[ENC_VSCL].btn_pin = 14; // ss1
    // encoders[ENC_VSHFT].btn_pin = 0;  // ss1
    encoders[ENC_HSCL].btn_pin = 17; // ss1
    // encoders[ENC_HSHFT].btn_pin = 18; // ss1
    encoders[ENC_TRIG].btn_pin = 1;  // ss0
    // encoders[ENC_USR].btn_pin = 5;  // ss0

    encoders[ENC_VSCL].ss = &ss1; // ss1
    // *encoders[ENC_VSHFT].ss = &ss1; // ss1
    encoders[ENC_HSCL].ss = &ss1; // ss1
    // *encoders[ENC_HSHFT].ss = &ss1; // ss1
    encoders[ENC_TRIG].ss = &ss0; // ss0
    // *(encoders[ENC_USR].ss) = ^ss0; // ss0

    encoders[ENC_VSCL].n = 0; // ss1
    // encoders[ENC_VSHFT].n = 1; // ss1
    encoders[ENC_HSCL].n = 2; // ss1
    // encoders[ENC_HSHFT].n = 3; // ss1
    encoders[ENC_TRIG].n = 0; // ss0
    // encoders[ENC_USR].n = 1; // ss0


    while(seesaw_gpio_pin_mode(ss0, 13, SEESAW_OUTPUT)); // Set pin 5 (onboard LED) as output
    // seesaw_gpio_digital_write_bulk(ss0, (1ul << 13), 1); // Turn off onboard LED (PIN_PC1)
    seesaw_gpio_digital_write_bulk(ss0, (1ul << 13), 0); // Turn on onboard LED (PIN_PC1)

    while(seesaw_gpio_pin_mode(ss1, 4, SEESAW_OUTPUT)); // Set pin 5 (onboard LED) as output
    // seesaw_gpio_digital_write_bulk(ss1, (1ul << 4), 1); // Turn off onboard LED (PIN_PB7)
    seesaw_gpio_digital_write_bulk(ss1, (1ul << 4), 0); // Turn on onboard LED (PIN_PB7)


    for (int i = 0; i < enc_cnt; i++) {
        enc_t* _enc = &(encoders[i]);

        printf("Configuring encoder %d and button pin %d...\n", i, _enc->btn_pin);
        printf("Enabling encoder interrupt...\n");
        while(seesaw_encoder_enable_interrupt(*(_enc->ss), _enc->n)); // Enable interrupt for encoder
        printf("\tSetting encoder position to 0...\n");
        while(seesaw_encoder_set_position(*(_enc->ss), _enc->n, 0)); // Reset encoder position to 0
        int32_t d;
        while(seesaw_encoder_get_delta(*(_enc->ss), _enc->n, &d)); // Reset encoder delta to 0
        printf("\tSetting button pin mode...\n");
        while(seesaw_gpio_pin_mode(*(_enc->ss), _enc->btn_pin, SEESAW_INPUT_PULLUP)); // Set button pins as input with pull-up
        while(seesaw_gpio_enable_interrupt(*(_enc->ss), _enc->btn_pin));

        _enc->delta = 0;
        _enc->position = 0;
        // _enc->btn = false;
        _enc->btn = true; // Active low
        // _enc->lim_lower = 1;
        // _enc->lim_upper = 0;
    }

    for (uint i = 0; i < BTN_CNT; i++) {
        while(seesaw_gpio_pin_mode(ss0, btn_pins[i], SEESAW_INPUT_PULLUP)); // Set button pins as input with pull-up
        while(seesaw_gpio_enable_interrupt(ss0, btn_pins[i])); // Enable interrupt for encoder
    }

    // // H Scale
    // while(seesaw_encoder_set_position(encoders[ENC_HSCL].ss, encoders[ENC_HSCL].n, 20*1000/500)); // Reset encoder position to 20000
    // // V Scale
    // while(seesaw_encoder_set_position(encoders[ENC_VSCL].ss, encoders[ENC_VSCL].n, 1000/100)); // Reset encoder position to 1000
    // // Trigger
    // while(seesaw_encoder_set_position(encoders[ENC_TRIG].ss, encoders[ENC_TRIG].n, 1000/100)); // Reset encoder position to 1000

    seesaw_neopixel_set_pin(ss0, 12u); // PIN_PC0
    seesaw_neopixel_set_len(ss0, 4*3);
    seesaw_neopixel_set_pixel(ss0, 0*3, seesaw_color(0,20,0));
    seesaw_neopixel_set_pixel(ss0, 1*3, seesaw_color(20,0,0));
    seesaw_neopixel_set_pixel(ss0, 2*3, seesaw_color(20,0,0));
    seesaw_neopixel_set_pixel(ss0, 3*3, seesaw_color(0,0,20));
    // seesaw_neopixel_set_brightness(ss0, )
    seesaw_neopixel_show(ss0);

    alarm_time = get_absolute_time() + delay_us;
}

void seesaw_reset() {

    seesaw_swrst(ss0);
    seesaw_swrst(ss1);

    while(seesaw_gpio_pin_mode(ss0, 13, SEESAW_OUTPUT)); // Set pin 5 (onboard LED) as output
    // seesaw_gpio_digital_write_bulk(ss0, (1ul << 13), 1); // Turn off onboard LED (PIN_PC1)
    seesaw_gpio_digital_write_bulk(ss0, (1ul << 13), 0); // Turn on onboard LED (PIN_PC1)

    while(seesaw_gpio_pin_mode(ss1, 4, SEESAW_OUTPUT)); // Set pin 5 (onboard LED) as output
    // seesaw_gpio_digital_write_bulk(ss1, (1ul << 4), 1); // Turn off onboard LED (PIN_PB7)
    seesaw_gpio_digital_write_bulk(ss1, (1ul << 4), 0); // Turn on onboard LED (PIN_PB7)


    for (int i = 0; i < enc_cnt; i++) {
        enc_t* _enc = &(encoders[i]);

        printf("Configuring encoder %d and button pin %d...\n", i, _enc->btn_pin);
        printf("Enabling encoder interrupt...\n");
        while(seesaw_encoder_enable_interrupt(*(_enc->ss), _enc->n)); // Enable interrupt for encoder
        printf("\tSetting encoder position to 0...\n");
        while(seesaw_encoder_set_position(*(_enc->ss), _enc->n, 0)); // Reset encoder position to 0
        printf("\tSetting button pin mode...\n");
        while(seesaw_gpio_pin_mode(*(_enc->ss), _enc->btn_pin, SEESAW_INPUT_PULLUP)); // Set button pins as input with pull-up
        while(seesaw_gpio_enable_interrupt(*(_enc->ss), _enc->btn_pin));

        _enc->delta = 0;
        _enc->position = 0;
        // _enc->btn = false;
        _enc->btn = true; // Active low
        // _enc->lim_lower = 1;
        // _enc->lim_upper = 0;
    }

    for (uint i = 0; i < BTN_CNT; i++) {
        while(seesaw_gpio_pin_mode(ss0, btn_pins[i], SEESAW_INPUT_PULLUP)); // Set button pins as input with pull-up
        while(seesaw_gpio_enable_interrupt(ss0, btn_pins[i])); // Enable interrupt for encoder
    }

    seesaw_neopixel_set_pin(ss0, 12u); // PIN_PC0
    seesaw_neopixel_set_len(ss0, 4*3);
    seesaw_neopixel_set_pixel(ss0, 0*3, seesaw_color(0,20,0));
    seesaw_neopixel_set_pixel(ss0, 1*3, seesaw_color(20,0,0));
    seesaw_neopixel_set_pixel(ss0, 2*3, seesaw_color(20,0,0));
    seesaw_neopixel_set_pixel(ss0, 3*3, seesaw_color(0,0,20));
    // seesaw_neopixel_set_brightness(ss0, )
    seesaw_neopixel_show(ss0);


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

        bool check_ss0 = 0;
        bool check_ss1 = 0;
        uint32_t value = 0;
        if (!seesaw_gpio_get_interupts(ss0, &value))
            check_ss0 = value;
        else check_ss0 = 1;
        if (!seesaw_gpio_get_interupts(ss1, &value))
            check_ss1 = value;
        else check_ss1 = 1;

        if (!check_ss0 && !check_ss1) {
            // uint64_t t = get_absolute_time() - old;
            // printf("Capture took %llu us\n", t);
            return;
        }
        
        // Get buttons
        if (check_ss0) {
            uint32_t btn_mask = 0;
            for (int i = 0; i < BTN_CNT /*&& check_ss0*/; i++) {
                btn_mask |= (1<<btn_pins[i]);
                // if (seesaw_gpio_digital_read(ss0, btn_pins[i], &(buttons[i])) != 0) {
                //     printf("Error reading button state\n");
                // }
                // sleep_us(100); // Small delay to avoid overloading the seesaw
            }
            for (int i = 0; i < enc_cnt; i++) {
                if (encoders[i].ss == &ss0)
                    btn_mask |= (1<<(encoders[i].btn_pin));
            }
            uint32_t btn_state = 0;
            if (seesaw_gpio_digital_read_bulk(ss0, btn_mask, &btn_state) != 0) {
                printf("Error reading button state\n");
            } else {
                for (int i = 0; i < enc_cnt; i++) {
                    if (encoders[i].ss == &ss0)
                        encoders[i].btn = btn_state & (1<<(encoders[i].btn_pin));
                }
                for (int i = 0; i < BTN_CNT; i++) {
                    buttons[i] = btn_state & (1<<btn_pins[i]);
                }
            }
        }

        // Get buttons
        if (check_ss1) {
            uint32_t btn_mask = 0;
            for (int i = 0; i < enc_cnt; i++) {
                if (encoders[i].ss == &ss1)
                    btn_mask |= (1<<(encoders[i].btn_pin));
            }
            uint32_t btn_state = 0;
            if (seesaw_gpio_digital_read_bulk(ss1, btn_mask, &btn_state) != 0) {
                printf("Error reading button state\n");
            } else {
                for (int i = 0; i < enc_cnt; i++) {
                    if (encoders[i].ss == &ss1)
                        encoders[i].btn = btn_state & (1<<(encoders[i].btn_pin));
                }
            }
        }

        for (int i = 0; i < enc_cnt; i++) {
            enc_t* _enc = &(encoders[i]);
            int32_t d = 0;

            if (((_enc->ss == &ss0) && !check_ss0) ||
                ((_enc->ss == &ss1) && !check_ss1))
                continue;

            if (seesaw_encoder_get_delta(*(_enc->ss), _enc->n, &d) != 0) {
                printf("Error reading encoder delta\n");
            } else {
                _enc->delta += d;
            }
            // if (seesaw_encoder_get_position(*(_enc->ss), _enc->n, &(_enc->position)) != 0) {
            // 	// printf("Error reading encoder position\n");
            // }
            // // Change the data
            // if (_enc->position <= 0) {
            // 	encoders[i].position = 1;
            // 	if (seesaw_encoder_set_position(*(_enc->ss), _enc->n, 1)) {
            // 		// printf("Error setting encoder position\n");
            // 	}
            // }
            // sleep_us(250); // Small delay to avoid overloading the seesaw
            // sleep_us(100); // Small delay to avoid overloading the seesaw
            // if (seesaw_gpio_digital_read(*(_enc->ss), _enc->btn_pin, &(_enc->btn)) != 0) {
            //     printf("Error reading button state\n");
            // }
            // sleep_us(250); // Small delay to avoid overloading the seesaw
            sleep_us(100); // Small delay to avoid overloading the seesaw
        }

        // HorizontalScale = encoders[0].position*500;
        // VerticalScale = encoders[1].position*100;
        // TriggerVoltage = encoders[2].position*100;

        // printf("Time: %ld us    Overshoot: %ld us\t", (uint32_t)get_absolute_time(), (uint32_t)absolute_time_diff_us(old, get_absolute_time()));
        // for (int i = 0; i < enc_cnt; i++) {
        //     // printf("Enc %d: Pos Calc = %03d, Pos = %03d, Delta = %03d    ", i, position_calc[i], position[i], delta[i]);
        //     printf("Enc %d Pos: %04ld Btn: %d    ", i, encoders[i].position, !encoders[i].btn);
        //     // printf("%04ld (%01u)   ", position[i], !buttons[i]);
        //     stdio_flush();
        //     // printf("Enc %d Pos: %03d    ", i, position_calc[i]);
        // }
        // // printf("VS: %u", VerticalScale);
        // printf("\n");
        // uint64_t t = get_absolute_time() - old;
        // printf("Capture took %llu us\n", t);
    }
}

uint8_t _seesaw_btn_prev_state = 0;
uint8_t seesaw_check_buttons(uint8_t btn_mask, uint8_t edge_mask) {
    uint8_t state = 0;
    for (uint8_t i = 0; i < BTN_CNT; i++) {
        state |= (buttons[i] << i);
    }
    uint8_t changed = state ^ _seesaw_btn_prev_state;
    uint8_t rising = changed & state; // Button press events
    uint8_t falling = changed & ~state; // Button press events
    uint8_t result = 0;
    if (edge_mask & RISING_EDGE) result |= rising & btn_mask;
    if (edge_mask & FALLING_EDGE) result |= falling & btn_mask;
    // Update prev_state only for the buttons we're interested in
    _seesaw_btn_prev_state |= state & btn_mask; // Set bits for buttons that are currently pressed
    _seesaw_btn_prev_state &= ~(~state & btn_mask); // Clear bits for buttons that are now released
    // printf("Button state: %02X    Changed: %02X    Rising: %02X    Falling: %02X\n", state, changed, rising, falling);
    return result;
}


uint8_t seesaw_check_enc_buttons(uint8_t btn_mask, uint8_t edge_mask) {
    static uint8_t prev_state = 0;
    uint8_t state = 0;
    for (uint8_t i = 0; i < ENC_CNT; i++) {
        enc_t* _enc = &(encoders[i]);
        state |= (_enc->btn << i);
    }
    uint8_t changed = state ^ prev_state;
    uint8_t rising = changed & state; // Button press events
    uint8_t falling = changed & ~state; // Button press events
    uint8_t result = 0;
    if (edge_mask & RISING_EDGE) result |= rising & btn_mask;
    if (edge_mask & FALLING_EDGE) result |= falling & btn_mask;
    // Update prev_state only for the buttons we're interested in
    prev_state |= state & btn_mask; // Set bits for buttons that are currently pressed
    prev_state &= ~(~state & btn_mask); // Clear bits for buttons that are now released
    // printf("Button state: %02X    Changed: %02X    Rising: %02X    Falling: %02X\n", state, changed, rising, falling);
    return result;
}


// Updates the provided value pointed to by var using the internally tracked
// values
// void seesaw_update_val(uint8_t enc, int32_t* var) {
// 	if (enc >= enc_cnt)
// 		return;
// 	LogValue_t* v = &(encoders[enc].value);
// 	*var = v->digit * v->decade;
// }

void seesaw_update_val(uint8_t enc, int32_t* var, bool reverse) {
    if (enc >= enc_cnt || !var)
        return;
    if (reverse)
        *var -= encoders[enc].delta;
    else
        *var += encoders[enc].delta;
    encoders[enc].delta = 0;
}

void seesaw_update_dyn_val(uint8_t enc, DynamicValue_t* dv) {
    if (enc >= enc_cnt || !dv || !dv->ranges)
        return;

    int32_t steps = encoders[enc].delta;
    encoders[enc].delta = 0;

    while (steps != 0) {
        uint32_t current = dv->value;

        /* Find the active range */
        // ValueRange_t *r = dv->ranges;
        uint i = 0;
        for (; current >= dv->ranges[i].max; i++) {
            if (i+1 == dv->ranges_cnt) {
                dv->value = dv->ranges[i].max;
                break;
            }
        }
        ValueRange_t *r = &(dv->ranges[i]);

        uint32_t step = r->step;

        // Step forward
        if (steps > 0) {
            uint32_t next = current + step;

            if (next > r->max)
                next = r->max;

            dv->value = next;
            steps--;
        }
        // Step Backward
        else {
            uint32_t prev;

            if (current <= step)
                prev = 0;
            else
                prev = current - step;

            if (prev < dv->min) {
                // Don't go below min
                prev = dv->min;
            } else if (i) {
                // if crossing range boundary, stop at boundary
                if (prev < dv->ranges[i-1].max) {
                    prev = dv->ranges[i-1].max;
                }
            }

            dv->value = prev;
            steps++;
        }

        // If we are at the min and not stepping away from it (increasing)
        if (dv->value == dv->min && steps < 0)
            break;
    }

    // return dv->value;
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