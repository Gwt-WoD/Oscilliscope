#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "time.h"

// Pico
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/spi.h"
#include "hardware/pio.h"

// ADC
#include "ads704x_dma.h"

// Display
// #include "pico-displayDrivs/ili9341/ili9341.h"
// #include "pico-displayDrivs/gfx/gfx.h"
// #include "pico-displayDrivs/gfx/font.h"
#include "ili9341.h"
#include "gfx.h"
#include "font.h"

// Seesaw Driver
#include "seesaw.h"

// Overclocking settings
#define OVERCLOCKING_ENABLED 1 // Define this before including overclock.h
#include "overclock.h"


#include "pins.h" // Pin mappings


#define IS_RGBW false
#include "pico/status_led.h"
// #include "ws2812.pio.h"

#include "blink.pio.h"


Seesaw_t ss = {
	.i2c_inst = I2C_PORT,
	.i2c_addr = SEESAW_DEFAULT_ADDR
};


void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}



void DataProcessor(float* VC1, float* VC2, int NS, float VS, float Trigger);
void DispDriver(float* VC1, float* VC2, int NS, float HS, float VS, float Trigger);


void core1_main();


int main() {
    // Initialize serial output
    stdio_init_all();
    // Looking for USB connection. Waiting until serial is opened
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(50);

    printf("Serial connected!\n");


    // Overclock core
    const bool VERBOSE_OVERCLOCK = true;
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_270], VERBOSE_OVERCLOCK); // WARNING: Requires 1.20V - Do at your own risk!
    // This is a good choice for creating the most sample rate options using the DMA timer
    int ret = overclock_core(OC_PRESETS[OC_FREQ_260], VERBOSE_OVERCLOCK); // Use this
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_250], VERBOSE_OVERCLOCK);
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_200], VERBOSE_OVERCLOCK);
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_125], VERBOSE_OVERCLOCK); // Default
    if (ret < 0) {
        printf("Overclocking failed with error code: %d\n", ret);
        if (ret == OC_ERR_RESUS_OCCURRED) {
            stdio_init_all();
            printf("A resus event occurred during overclocking. System has been reset to safe frequency.\n");
        }
    } else {
        printf("Overclocking successful! New frequency: %d kHz\n", ret);
    }
    

    //DEBUG: Srand for if we want to do random stuff
    srand(time(NULL));

    
    // Display SPI initialisation
    printf("Initialising Display SPI...\n");
    /**
     * I believe SPI0 and SPI1 share the same clock divider.
     * The ADC operates at a max clock of 16MHz
     */
    spi_init(DISP_SPI, 16*1000*1000); // 16 MHz
    gpio_set_function(DISP_PIN_RST,  GPIO_FUNC_SPI);
    gpio_set_function(DISP_PIN_CS,   GPIO_FUNC_SIO); // Uhhh... I think this should be GPIO_FUNC_SPI... -Michael
    gpio_set_function(DISP_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(DISP_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(DISP_PIN_DC,   GPIO_FUNC_SPI); // And this should be GPIO_FUNC_SIO! -Michael
    
    // Chip select is active-low. Reset to high state.
    gpio_set_dir(DISP_PIN_CS, GPIO_OUT);
    gpio_put(DISP_PIN_CS, 1);

    // Display initialisation
    printf("Configuring LCD...\n");
    LCD_initDisplay();
    LCD_setRotation(1);
    GFX_createFramebuf();


    // PIO Blinking example
    // PIO pio = pio0;
    // uint offset = pio_add_program(pio, &blink_program);
    // printf("Loaded program at %d\n", offset);
    // blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);

        // PIO pio = pio0;
    // uint offset = pio_add_program(pio, &blink_program);
    // printf("Loaded program at %d\n", offset);
    // // blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    // blink_pin_forever(pio, 0, offset, 13, 3);
    // status_led_init();
    // colored_status_led_set_on_with_color(PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 255, 0)); // Set status LED to green to indicate successful initialization and overclocking
    bool rc = status_led_init();
    hard_assert(rc);
    hard_assert(colored_status_led_supported());
    colored_status_led_set_on_with_color(PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 255, 0)); // Set status LED to green to indicate successful initialization and overclocking
    

    // Launch second core
    printf("Starting core 1...\n");
    multicore_launch_core1(core1_main);
    // Wait for core 1 to signal that it's done with initialization and ready to go
    uint32_t r = multicore_fifo_pop_blocking();


    //TODO: testing time :)


    printf("Initialization complete!\n");


    //DEBUG: X value for validation cosines
    float x = 0;


    //in V
    float TriggerVoltage = 1;
    //in us
    int HorizontalScale = 1000;
    //in mV
    int VerticalScale = 1000;
    //in kHz
    int SamplingRate = 200;

    int c;
    while (true) {
        
        //convert to microseconds 1kHz = 1000us and convert to number of samples
        int NumSamples = HorizontalScale/(1000/SamplingRate);
    

        //initialize both arrays
        float DataArray1[NumSamples+3];
        float DataArray2[NumSamples+3];


        //DEBUG: Validation cosines
        int i = 0;
        while (i < NumSamples){
            float y = (2.5 * cos(2 * M_PI * x)) + 2.5;
            float y2 = (0.5 * cos(3 * M_PI * x)) + 2.5;
            x = x + 0.01;
            DataArray1[i] = y;
            DataArray2[i] = y2;
            i++;
        }


        //DATA PROCESSOR BLOCK
        DataProcessor(DataArray1, DataArray2, NumSamples, VerticalScale, TriggerVoltage);

        //DISPLAY DRIVER BLOCK     
        DispDriver(DataArray1, DataArray2, NumSamples, HorizontalScale, VerticalScale, TriggerVoltage);   

        c++;
    }
}



void core1_main() {


    // User Input I2C Initialisation. Using it at 400Khz.
    printf("Initialising I2C...\n");
    // Returns actual baudrate which may differ from requested baud
    ss.baud = i2c_init(I2C_PORT, 400*1000); // 400kHz
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
    seesaw_gpio_pin_mode(ss, SEESAW_PIN_LED, SEESAW_OUTPUT); // Set pin 5 (onboard LED) as output
    seesaw_gpio_digital_write_bulk(ss, (1ul << SEESAW_PIN_LED), 1); // Turn off onboard LED
    const uint8_t button_pins[] = {9, 18, 12}; // Pins on Seesaw MCU, not this one
    const uint8_t num_enc = 3; // SET ME TO THE NUMBER OF ENCODERS YOU HAVE
    for (int i = 0; i < num_enc; i++) {
        printf("Configuring encoder %d and button pin %d...\n", i, button_pins[i]);
        printf("Enabling encoder interrupt...\n");
        seesaw_encoder_enable_interrupt(ss, i); // Enable interrupt for encoder
        printf("\tSetting encoder position to 0...\n");
        seesaw_encoder_set_position(ss, i, 0); // Reset encoder position to 0
        printf("\tSetting button pin mode...\n");
        seesaw_gpio_pin_mode(ss, button_pins[i], SEESAW_INPUT_PULLUP); // Set button pins as input with pull-up
    }
    bool buttons[num_enc];
    int32_t position[num_enc];
    int32_t position_calc[num_enc];
    int32_t delta[num_enc];
    for (int i = 0; i < num_enc; i++) {
        position[i] = 0;
        position_calc[i] = 0;
        delta[i] = 0;
        buttons[i] = false;
    }


    // Sample the user input at least once every 100ms. 40ms allows for multiple
    // samples incase one is missed, taking into account the time it takes for
    // each sample.
    // A better solution would be to use a retry mechanism so a single failed 
    // read doesn't cause the current sample to be aborted.
    const absolute_time_t delay_us = (40*1000); // 40ms
    absolute_time_t alarm_time = get_absolute_time() + delay_us;
    const absolute_time_t delay1_us = (1000*1000); // 1s
    absolute_time_t alarm1_time = get_absolute_time() + delay_us;

    // Signal to core 0 that we're done with initialization and ready to go
    multicore_fifo_push_blocking(1);

    // Main loop for core 1
    while (true) {
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
        
            bool bad = false;
            for (int i = 0; i < num_enc; i++) {
                // if (seesaw_encoder_get_delta(ss, i, &delta[i]) == 0)
                //     position_calc[i] += delta[i];
                // else {
                //     bad = true;
                //     printf("Error reading encoder delta\n");
                // }
                if (seesaw_encoder_get_position(ss, i, &position[i]) != 0) {
                    bad = true;
                    // printf("Error reading encoder position\n");
                }
                sleep_us(250); // Small delay to avoid overloading the seesaw
                if (seesaw_gpio_digital_read(ss, button_pins[i], &buttons[i]) != 0) {
                    bad = true;
                    // printf("Error reading button state\n");
                }
                sleep_us(250); // Small delay to avoid overloading the seesaw
            }
            
            if (bad) {
                // printf("Error reading from Seesaw\n");
                continue; // Skip printing if there was an error reading from Seesaw
            }
            // printf("Time: %ld us    Overshoot: %ld us\t", (uint32_t)get_absolute_time(), (uint32_t)absolute_time_diff_us(old, get_absolute_time()));
            for (int i = 0; i < num_enc; i++) {
                // printf("Enc %d: Pos Calc = %03d, Pos = %03d, Delta = %03d    ", i, position_calc[i], position[i], delta[i]);
                printf("Enc %d Pos: %04ld Btn: %d    ", i, position[i], !buttons[i]);
                // printf("Enc %d Pos: %03d    ", i, position_calc[i]);
            }
            printf("\n");
        }
    }
}



//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER
//DESC: Example descriptions
//voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
void DispDriver(float* VC1, float* VC2, int NS, float HS, float VS, float Trigger) {

    GFX_clearScreen();
    GFX_setCursor(0, 0);
    GFX_fillScreen(GFX_RGB565(50,50,50));
    
    //value prints
    GFX_printf("VC1: The Max is %f\n", VC1[NS]);
    GFX_printf("VC1: The Min is %f\n", VC1[NS + 1]);
    GFX_printf("VC2: The Max is %f\n", VC2[NS]);
    GFX_printf("VC2: The Min is %f\n", VC2[NS + 1]);
    GFX_printf("Trigger: The Trigger Voltage is %f volts\n", Trigger);
    GFX_printf("VS: 1 tick = %f volts\n", (VS / 1000));
    GFX_printf("HS: 1 tick = %f ms\n", (HS / 10000));
    

    //channel prints
    int i = 0;
    while(i < NS){
        GFX_drawPixel(((300 * i) / NS) + 10, 200 - floor(VC1[i] * (20000/VS)), GFX_RGB565(255,0,0));
        GFX_drawPixel(((300 * i) / NS) + 10, 200 - floor(VC2[i] * (20000/VS)), GFX_RGB565(0,0,255));
        i++;
    }

    //box around functions
    GFX_drawFastHLine(10, 100, 300,GFX_RGB565(255,255,255));
    GFX_drawFastHLine(10, 200, 300,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(10, 100, 100,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(310, 100, 100,GFX_RGB565(255,255,255));
    //voltage markings
    GFX_drawFastHLine(10, 120, 10,GFX_RGB565(255,255,255));
    GFX_drawFastHLine(10, 140, 10,GFX_RGB565(255,255,255));
    GFX_drawFastHLine(10, 160, 10,GFX_RGB565(255,255,255));
    GFX_drawFastHLine(10, 180, 10,GFX_RGB565(255,255,255));
    //time markings
    GFX_drawFastVLine(40, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(70, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(100, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(130, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(160, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(190, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(220, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(250, 190, 10,GFX_RGB565(255,255,255));
    GFX_drawFastVLine(280, 190, 10,GFX_RGB565(255,255,255));

    //SEND ITTTTT!!!
    GFX_flush();

}

//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DATA PROCESSOR
//DESC: Takes in two arrays of data and does some math. spits them out on the other side
//voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
void DataProcessor(float* VC1, float* VC2, int NS, float VS, float Trigger) {
    //VC1 Variables
    float MinVC1 = 100;
    float MaxVC1 = 0;
    bool TriggerFlagVC1 = 0;
    int NumTriggersVC1 = 0;
    
    //VC2 Variables
    float MinVC2 = 100;
    float MaxVC2 = 0;
    bool TriggerFlagVC2 = 0;
    int NumTriggersVC2 = 0;
    int j = 0;
    
    //loop from the start to the end of the arrays and calculate important values
    while(j < NS){
        
        if(VC1[j] > MaxVC1){
            MaxVC1 = VC1[j];
        }
        
        if(VC2[j] > MaxVC2){
            MaxVC2 = VC2[j];
        }
        
        if(VC1[j] < MinVC1){
            MinVC1 = VC1[j];
        }

        if(VC2[j] < MinVC2){
            MinVC2 = VC2[j];
        }

        j++;
    }



    //Use the last couple of slots for important values
    VC1[NS] = MaxVC1; 
    VC1[NS + 1] = MinVC1;
    VC1[NS + 2] = NumTriggersVC1;
   
    //DEBUG: Serial PRint of special values
    //printf("VC1: The Max is %f\n", VC1[NS]);
    //printf("VC1: The Min is %f\n", VC1[NS + 1]);
    //printf("VC1: The Number of Triggers is %f\n", VC1[NS + 2]);
    
    VC2[NS] = MaxVC2; 
    VC2[NS + 1] = MinVC2;
    VC2[NS + 2] = NumTriggersVC2;

    //DEBUG: Serial PRint of special values
    //printf("VC2: The Max is %f\n", VC2[NS]);
    //printf("VC2: The Min is %f\n", VC2[NS + 1]);
    //printf("VC2: The Number of Triggers is %f\n", VC2[NS + 2]);

}