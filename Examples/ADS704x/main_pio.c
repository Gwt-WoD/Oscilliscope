#include <stdio.h>
#include <stdlib.h> // For malloc and free
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"


#include "hardware/pio.h"
// #include "ads704x_pio.pio.h"
#include "ads704x_pio.h"

// Overclocking settings
#define OVERCLOCKING_ENABLED 1 // Define this before including overclock.h
#include "overclock.h"


#define ADC_SPI       spi1
#define ADC_PIN_SCK   14
#define ADC_PIN_MISO  12
#define ADC_PIN_CS0   9
#define ADC_PIN_CS1   13

#define USBBOOT_BTN 1

volatile uint16_t test_buff_0[1024];
volatile uint16_t test_buff_1[1024];



void _test_raw16_to_float(uint16_t raw);



int main() {
    gpio_init(USBBOOT_BTN);
    gpio_set_dir(USBBOOT_BTN, GPIO_IN);
    // Initialize serial output
    stdio_init_all();
    // Looking for USB connection. Waiting until serial is opened
    while (!stdio_usb_connected()) {
        sleep_ms(50);
        if (!gpio_get(USBBOOT_BTN)) break;
    }
    sleep_ms(50);
    printf("Serial connected!\n");

    // Overclock core
    int ret = overclock_core(OC_PRESETS[OC_FREQ_270], true); // WARNING: Requires 1.20V - Do at your own risk!
    // This is a good choice for creating the most sample rate options using the DMA timer
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_260], true); // Use this
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_250], true);
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_200], true);
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_125], true); // Default
    if (ret < 0) {
        printf("Overclocking failed with error code: %d\n", ret);
        if (ret == OC_ERR_RESUS_OCCURRED) {
            stdio_init_all();
            printf("A resus event occurred during overclocking. System has been reset to safe frequency.\n");
        }
    } else {
        printf("Overclocking successful! New frequency: %d kHz\n", ret);
    }

    ads704x_cfg_t cfg_0, cfg_1;

    ads704x_init(&cfg_0, pio0, ADC_PIN_CS0, ADC_PIN_SCK, ADC_PIN_MISO, 0);
    // ads704x_init(&cfg_1, pio0, ADC_PIN_CS1, ADC_PIN_SCK, ADC_PIN_MISO, 1);

    ads704x_setup_dma_stream_to_memory(&cfg_0, test_buff_0, 1024);
    // ads704x_setup_dma_stream_to_memory(&cfg_1, test_buff_1, 1024);

    ads704x_start(&cfg_0);
    // ads704x_start(&cfg_1);

    uint32_t dma_chan_0 = ads704x_get_dma_channel(&cfg_0);
    // uint32_t dma_chan_1 = ads704x_get_dma_channel(&cfg_1);

    dma_channel_wait_for_finish_blocking(dma_chan_0);
    // dma_channel_wait_for_finish_blocking(dma_chan_1);

    for (int i = 0; i < 1024; i++) {
        float voltage0 = ( test_buff_0[i] / (float)((1<<12)-1) ) * 2.8f * 2.0f;
        // float voltage1 = ( test_buff_1[i] / (float)((1<<12)-1) ) * 2.8f * 2.0f;
        // printf("Sample %d: ADC0 = %u (%.3f V), ADC1 = %u (%.3f V)\n", i, test_buff_0[i], voltage0, test_buff_1[i], voltage1);
        printf("Sample %d: %u (%.3f V)\n", i, test_buff_0[i], voltage0);
    }

    while (1) {
        sleep_ms(1000);
    }
}



void _test_raw16_to_float(uint16_t raw) {
	// Raw value * 2x attenuation, then scale to voltage based on 12-bit resolution and 2.8V reference
	// float voltage1 = (raw / (float)(1 << (12 - 2))) * 2.8f;
	// float voltage2 = (raw / (float)(1 << 12)) * 2.8f; // Assuming 3.3V reference
	// float voltage1 = ( ((raw & ADS7043_DATA_MASK) >> 2) / (float)((1<<12)-1) ) * 2.8f * 2.0f;
	float voltage1 = ( raw / (float)((1<<12)-1) ) * 2.8f * 2.0f;
	// Print raw binary value and calculated voltage
	// printf("Raw: 0x%04X (0b", raw);
	// for (int i = 15; i >= 0; i--) {
	// 	printf("%d", (raw >> i) & 1);
	// }
	// printf("), Voltage: %.3f V\n", voltage1);
	printf("%.3f V\n", voltage1);
}