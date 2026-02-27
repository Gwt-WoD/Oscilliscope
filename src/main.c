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
#include "ili9341.h"
#include "gfx.h"
#include "font.h"

// Seesaw Driver
#include "seesaw.h"

// Overclocking settings
#define OVERCLOCKING_ENABLED 1 // Define this before including overclock.h
#include "overclock.h"


#include "pins.h" // Pin mappings



#include "blink.pio.h"


Seesaw_t ss = {
	.i2c_inst = I2C_PORT,
	.i2c_addr = SEESAW_DEFAULT_ADDR
};

const size_t DataArray_len = (16*1024);
// const size_t DataArray_len = (16*1024 / 2); // For performance
// const size_t DataArray_len = (16*1024 / 4); // For performance
uint16_t DataArray0[16*1024];
uint16_t DataArray1[16*1024];


volatile float HorizontalScale = 1000;     // uS
volatile uint16_t VerticalScale = 1000;    // mV
volatile uint16_t TriggerVoltage = 1000;   // mV
volatile uint32_t SamplingRate = 200*1000; // Hz
uint32_t NumSamples;


typedef struct {
    uint16_t max;
    uint16_t min;
    uint16_t NumTriggers;
} VC_stats_t;

VC_stats_t VC1_stats;
VC_stats_t VC2_stats;


void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}


void DispDriverFrameSetup();
void DataProcessor(uint16_t VC1, uint16_t VC2, uint NS, uint16_t VS, uint16_t Trigger);
void DispDriver(uint16_t VC1, uint16_t VC2, uint NS, float HS, uint16_t VS, uint16_t Trigger, float fps, int i);
void DispDriverFrameFinish(uint16_t VC1, uint16_t VC2, uint NS, float HS, uint16_t VS, uint16_t Trigger, float fps, int i);


void core1_main();


int main() {
    // Initialize serial output
    stdio_init_all();
    // Looking for USB connection. Waiting until serial is opened
    // while (!stdio_usb_connected()) {
    //     sleep_ms(50);
    // }
    // sleep_ms(50);
    // printf("Serial connected!\n");


    { // Limit scope
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
        printf("Getting pll_sys clock speed");
        // Get pll_sys clock speed
        uint32_t _pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
        printf("\tpll_sys: %d\n", _pll_sys*1000);
        printf("Configuring clk_peri source to pll_sys with no division...\n");
        clock_configure(clk_peri, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, _pll_sys*1000, _pll_sys*1000);
        printf("\tclk_peri: %d\n", clock_get_hz(clk_peri));

        // Configure ~10 KHz clock output for probe calibration and testing
        printf("Configuring Freq. Gen. GPIO output...\n");
        // clock_gpio_init(AFE_FREQ_GEN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, 25000);
        clock_gpio_init(AFE_FREQ_GEN, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, (uint16_t)(ret/10)); // 10 KHz output for calibration
    }
    

    //DEBUG: Srand for if we want to do random stuff
    srand(time(NULL));

    
    // Display SPI initialisation
    printf("Initialising Display SPI...\n");
    /**
     * I believe SPI0 and SPI1 share the same clock divider.
     * The ADC operates at a max clock of 16MHz
     */
    // spi_init(DISP_SPI, 16*1000*1000); // 16 MHz
    uint32_t baud = spi_init(DISP_SPI, 65*1000*1000); // 65 MHz
    printf("SPI Baud set to %u\n", baud);
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
    PIO pio = pio0;
    { // Limit scope of offset variable
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
        // blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
        blink_pin_forever(pio, 0, offset, 13, 3);
    }


    // Launch second core
    printf("Starting core 1...\n");
    multicore_launch_core1(core1_main);
    // Wait for core 1 to signal that it's done with initialization and ready to go
    uint32_t r = multicore_fifo_pop_blocking();


    //TODO: testing time :)


    printf("Initialization complete!\n");


    //DEBUG: Validation cosines
    for (int x=0; x < DataArray_len; x++) {
        // ((uint16_t*)data_buffer0)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
        // ((uint16_t*)data_buffer1)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
        // Normalized period to length of data array => freq. * 2pi/len * x
        uint16_t y = 1000.0 * ((2.5 * cos(2 * (2.0*M_PI) * x / DataArray_len)) + 2.5);
        uint16_t y2 = 1000.0 * ((0.75 * cos(7.5 * (2.0*M_PI) * x / DataArray_len)) + 3.0);
        DataArray0[x] = y;
        DataArray1[x] = y2;
    }
    
    volatile uint32_t start, end;
    volatile uint32_t start_avg, end_avg; // TODO: 
    volatile uint32_t max, min;
    volatile float fps;
    volatile int c;
    start_avg = time_us_32();
    while (true) {
        if (c >= 20) {
            c = 0;
            end_avg = end;
            fps = (float)((20.0*1000.0*1000.0)/(end - start_avg));
            // printf("FPS: %.03f    Avg: %.02f ms    Max: %.02f ms    Min: %.02f ms\n", (float)((20.0*1000.0*1000.0)/(end - start_avg)), (float)((end - start_avg)/(20.0*1000)), (float)(max/1000.0), (float)(min/1000.0));
            max = 0;
            min = UINT32_MAX;
            start_avg = time_us_32();
        }
        start = time_us_32();

        //convert to microseconds 1Hz = 1000,000us and convert to number of samples
        NumSamples = HorizontalScale / (1000*1000 / SamplingRate); // Untested
        // NumSamples = fminf(DataArray_len, (HorizontalScale / (1000*1000 / SamplingRate))); // Untested


        VC1_stats.max = 0;
        VC1_stats.min = UINT16_MAX;

        VC2_stats.max = 0;
        VC2_stats.min = UINT16_MAX;
       
        int TriggerFlag = 0;
        int i = 0;
        int TriggerLocation = 0;

        //DISPLAY DRIVER BLOCK 
        DispDriverFrameSetup();

        while(i < DataArray_len){
            //if we are still triggering
            if(TriggerFlag > 0){
                //DATA PROCESSOR BLOCK
                DataProcessor(DataArray0[i], DataArray1[i], NumSamples, VerticalScale, TriggerVoltage);
                //DISPLAY DRIVER BLOCK     
                DispDriver(DataArray0[i], DataArray1[i], NumSamples, HorizontalScale, VerticalScale, TriggerVoltage, fps, i - TriggerLocation);
                
                //decriment to count down to zero
                TriggerFlag--;
            }else{
                //check if you are near the trigger voltage(TODO: edit these values as needed)
                if(i > 0){
                    if(((DataArray0[i - 1] < TriggerVoltage) && (DataArray0[i] > TriggerVoltage)) || ((DataArray0[i - 1] > TriggerVoltage) && (DataArray0[i] < TriggerVoltage))){
                        //take as many samples as needed
                        TriggerFlag = NumSamples - 1;
                        TriggerLocation = i;
                        //DATA PROCESSOR BLOCK
                        DataProcessor(DataArray0[i], DataArray1[i], NumSamples, VerticalScale, TriggerVoltage);
                        //DISPLAY DRIVER BLOCK     
                        DispDriver(DataArray0[i], DataArray1[i], NumSamples, HorizontalScale, VerticalScale, TriggerVoltage, fps, i - TriggerLocation);
                    }
                }
            }
            i++;
        }
        //DISPLAY DRIVER BLOCK    
        DispDriverFrameFinish(DataArray0[i], DataArray1[i], NumSamples, HorizontalScale, VerticalScale, TriggerVoltage, fps, i);
        

        end = time_us_32();
        volatile uint32_t t = end - start;
        if (t > max) max = t;
        if (t < min) min = t;
        c++;
    }
}



void core1_main() {


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
    const uint8_t button_pins[] = {9, 18, 12}; // Pins on Seesaw MCU, not this one
    const uint8_t num_enc = 3; // SET ME TO THE NUMBER OF ENCODERS YOU HAVE
    { // Limit scope of err variable
        while(seesaw_gpio_pin_mode(ss, SEESAW_PIN_LED, SEESAW_OUTPUT)); // Set pin 5 (onboard LED) as output
    seesaw_gpio_digital_write_bulk(ss, (1ul << SEESAW_PIN_LED), 1); // Turn off onboard LED
    for (int i = 0; i < num_enc; i++) {
        printf("Configuring encoder %d and button pin %d...\n", i, button_pins[i]);
        printf("Enabling encoder interrupt...\n");
            while(seesaw_encoder_enable_interrupt(ss, i)); // Enable interrupt for encoder
        printf("\tSetting encoder position to 0...\n");
            while(seesaw_encoder_set_position(ss, i, 0)); // Reset encoder position to 0
        printf("\tSetting button pin mode...\n");
            while(seesaw_gpio_pin_mode(ss, button_pins[i], SEESAW_INPUT_PULLUP)); // Set button pins as input with pull-up
        }
        // H Scale
        while(seesaw_encoder_set_position(ss, 0, 20*1000/500)); // Reset encoder position to 20000
        // V Scale
        while(seesaw_encoder_set_position(ss, 1, 1000/100)); // Reset encoder position to 1000
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
                //printf("Error reading from Seesaw\n");
                continue; // Skip printing if there was an error reading from Seesaw
            }

            // Change the data
            if (position[0] <= 0) {
                position[0] = 1;
                seesaw_encoder_set_position(ss, 0, 1);
            }
            if (position[1] <= 0) {
                position[1] = 1;
                seesaw_encoder_set_position(ss, 1, 1);
            }
            HorizontalScale = position[0]*500;
            VerticalScale = position[1]*100;

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
}

//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER FRAME SETUP
//DESC: Takes in two arrays of data and does some math. spits them out on the other side

void DispDriverFrameSetup() {
    const uint16_t white = GFX_RGB565(255,255,255);
    const uint16_t grey = GFX_RGB565(127,127,127);
    const uint16_t dark_grey = GFX_RGB565(50,50,50);

    GFX_setCursor(0, 0);
    GFX_fillScreen(dark_grey);


    //voltage markings
    GFX_drawFastHLine(10, 120, 300, grey);
    GFX_drawFastHLine(10, 140, 300, grey);
    GFX_drawFastHLine(10, 160, 300, grey);
    GFX_drawFastHLine(10, 180, 300, grey);
    //time markings
    GFX_drawFastVLine(40, 100, 100, grey);
    GFX_drawFastVLine(70, 100, 100, grey);
    GFX_drawFastVLine(100, 100, 100, grey);
    GFX_drawFastVLine(130, 100, 100, grey);
    GFX_drawFastVLine(160, 100, 100, grey);
    GFX_drawFastVLine(190, 100, 100, grey);
    GFX_drawFastVLine(220, 100, 100, grey);
    GFX_drawFastVLine(250, 100, 100, grey);
    GFX_drawFastVLine(280, 100, 100, grey);

}


//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER
//DESC: Example descriptions
//voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
void DispDriver(uint16_t VC1, uint16_t VC2, uint NS, float HS, uint16_t VS, uint16_t Trigger, float fps, int i) {
    // Draw waveforms to disp buffer
    // Width in pixels * sample# / number of samples
    const uint32_t x = ((300 * i) / NS) + 10;
    GFX_drawPixel(x, fmaxf(100, (200 - floor(VC1 * (20.0/VS)))), GFX_RGB565(255,0,0));
    GFX_drawPixel(x, fmaxf(100, (200 - floor(VC2 * (20.0/VS)))), GFX_RGB565(0,0,255));
    //Trigger line (yellow)
    GFX_drawFastHLine(10, Trigger, fmaxf(100, 200 - floor(Trigger * (20.0/VS))), GFX_RGB565(255,255,0));

}

//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER FRAME SETUP
//DESC: Takes in two arrays of data and does some math. spits them out on the other side

void DispDriverFrameFinish(uint16_t VC1, uint16_t VC2, uint NS, float HS, uint16_t VS, uint16_t Trigger, float fps, int i) {
    const uint16_t white = GFX_RGB565(255,255,255);
    const uint16_t grey = GFX_RGB565(127,127,127);
    const uint16_t dark_grey = GFX_RGB565(50,50,50);
    
    
    GFX_setTextBack(dark_grey);
    GFX_setTextColor(white);
    //value prints

    // Channel 1 Max
    GFX_printf("Ch. 1 Max: ");
    if (VC1_stats.max >= 1000) GFX_printf("%.02f V\n", (float)(VC1_stats.max / 1000.0)); // V
    else GFX_printf("%03.00f mV\n", (float)(VC1_stats.max)); // mV
    // Channel 1 Min
    GFX_printf("Ch. 1 Min: ");
    if (VC1_stats.min >= 1000) GFX_printf("%.02f V\n", (float)(VC1_stats.min / 1000.0)); // V
    else GFX_printf("%3.00f mV\n", (float)(VC1_stats.min)); // mV
    // Channel 2 Max
    GFX_printf("Ch. 2 Max: ");
    if (VC2_stats.max >= 1000) GFX_printf("%.02f V\n", (float)(VC2_stats.max / 1000.0)); // V
    else GFX_printf("%3.00f mV\n", (float)(VC2_stats.max)); // mV
    // Channel 2 Min
    GFX_printf("Ch. 2 Min: ");
    if (VC2_stats.min >= 1000) GFX_printf("%.02f V\n", (float)(VC2_stats.min / 1000.0)); // V
    else GFX_printf("%3.00f mV\n", (float)(VC2_stats.min)); // mV
    // Trigger
    GFX_printf("Trigger: ");
    if (Trigger >= 1000) GFX_printf("%.02f V\n", (float)(Trigger / 1000.0)); // V
    else GFX_printf("%3.00f mV\n", (float)(Trigger)); // mV
    // Vertical Scale
    GFX_printf("Volt. Scale: ");
    if (VS >= 1000) GFX_printf("%.02f V/div\n", (float)(VS / 1000.0)); // V
    else GFX_printf("%3.00f mV/div\n", (float)(VS)); // mV
    // Horizontal Scale
    GFX_printf("Time Scale: ");
    if (HS >= 1000) GFX_printf("%.01f ms/div\n", (float)(HS / 1000.0)); // mS
    else GFX_printf("%3.00f us/div\n", (float)(HS)); // uS

    //box around functions
    GFX_drawFastHLine(10, 100, 300, white);
    GFX_drawFastHLine(10, 200, 300, white);
    GFX_drawFastVLine(10, 100, 100, white);
    GFX_drawFastVLine(310, 100, 100, white);

    //SEND ITTTTT!!!
    GFX_flush();

}


//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DATA PROCESSOR
//DESC: Takes in two arrays of data and does some math. spits them out on the other side
//voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
void DataProcessor(uint16_t VC1, uint16_t VC2, uint NS, uint16_t VS, uint16_t Trigger) {
    

        
    if(VC1 > VC1_stats.max){
        VC1_stats.max = VC1;
    }
    
    if(VC2 > VC2_stats.max){
        VC2_stats.max = VC2;
    }
    
    if(VC1 < VC1_stats.min){
        VC1_stats.min = VC1;
    }

    if(VC2 < VC2_stats.min){
        VC2_stats.min = VC2;
    }


    // //Use the last couple of slots for important values
    // VC1[NS] = MaxVC1;
    // VC1[NS + 1] = MinVC1;
    // VC1[NS + 2] = NumTriggersVC1;
   
    // //DEBUG: Serial PRint of special values
    // //printf("VC1: The Max is %f\n", VC1[NS]);
    // //printf("VC1: The Min is %f\n", VC1[NS + 1]);
    // //printf("VC1: The Number of Triggers is %f\n", VC1[NS + 2]);
    
    // VC2[NS] = MaxVC2;
    // VC2[NS + 1] = MinVC2;
    // VC2[NS + 2] = NumTriggersVC2;

    // //DEBUG: Serial PRint of special values
    // //printf("VC2: The Max is %f\n", VC2[NS]);
    // //printf("VC2: The Min is %f\n", VC2[NS + 1]);
    // //printf("VC2: The Number of Triggers is %f\n", VC2[NS + 2]);

}