#include <stdio.h>
#include "math.h"

// Pico
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/rand.h"
#include "hardware/spi.h"

// ADC
#include "ads704x_dma.h"

// Display
#include "ili9341.h"
#include "gfx.h"
#include "font.h"

// Seesaw Driver
#include "seesaw.h"
// Seesaw application code
#include "src_seesaw.h"

// Overclocking settings
#define OVERCLOCKING_ENABLED 1 // Define this before including overclock.h
#include "overclock.h"

#include "pins.h" // Pin mappings


const size_t DataArray_len = (20*1024);
// const size_t DataArray_len = (16*1024 / 2); // For performance
// const size_t DataArray_len = (16*1024 / 4); // For performance
uint16_t DataArray0[20*1024];
uint16_t DataArray1[20*1024];
uint16_t DispBuff[320*240]; // 320x240 pixels, 16-bit color depth


volatile float HorizontalScale = 1000;     // uS
volatile uint16_t VerticalScale = 1000;    // mV
volatile uint16_t TriggerVoltage = 1000;   // mV
volatile uint32_t SamplingRate = 200*1000; // Hz
uint32_t NumSamples;

const uint16_t light_yellow = GFX_RGB565(200,200,0);


typedef struct {
    uint16_t max;
    uint16_t min;
    uint16_t NumTriggers;
} VC_stats_t;

VC_stats_t VC1_stats;
VC_stats_t VC2_stats;


void overclock(bool verbose);

inline void DataProcessor(uint16_t VC1, uint16_t VC2);
inline void DispDriverFrameSetup();
inline void DispDriver(uint16_t VC1, uint16_t VC2, uint NS, float HS, uint16_t VS, uint16_t i);
inline void DispDriverFrameFinish(float HS, uint16_t VS, uint16_t Trigger, float fps);

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

    // Overclock (with verbose output)
    overclock(true);

    
    // Display SPI initialisation
    // printf("Initialising Display SPI...\n");
    // // spi_init(DISP_SPI, 16*1000*1000); // 16 MHz
    // // uint32_t baud = spi_init(DISP_SPI, 65*1000*1000); // 65 MHz
    // uint32_t baud = spi_init(DISP_SPI, 67*1000*1000); // 67 MHz @ 270MHz sys
    // printf("SPI Baud set to %u MHz\n", baud/(1000*1000));
    // gpio_set_function(DISP_PIN_RST,  GPIO_FUNC_SPI);
    // gpio_set_function(DISP_PIN_CS,   GPIO_FUNC_SIO); // Uhhh... I think this should be GPIO_FUNC_SPI... -Michael
    // gpio_set_function(DISP_PIN_SCK,  GPIO_FUNC_SPI);
    // gpio_set_function(DISP_PIN_MOSI, GPIO_FUNC_SPI);
    // gpio_set_function(DISP_PIN_DC,   GPIO_FUNC_SPI); // And this should be GPIO_FUNC_SIO! -Michael
    
    // // Chip select is active-low. Reset to high state.
    // gpio_set_dir(DISP_PIN_CS, GPIO_OUT);
    // gpio_put(DISP_PIN_CS, 1);

    // Display initialisation
    printf("Configuring LCD...\n");
    // LCD_setPins(DISP_PIN_DC, DISP_PIN_CS, DISP_PIN_RST, DISP_PIN_SCK, DISP_PIN_MOSI);
    // LCD_setSPIperiph(DISP_SPI);
    LCD_initDisplay();
    LCD_setRotation(1);
    // GFX_createFramebuf();
    GFX_setFramebuf(DispBuff);


    // Launch second core
    printf("Starting core 1...\n");
    multicore_launch_core1(core1_main);
    // Wait for core 1 to signal that it's done with initialization and ready to go
    multicore_fifo_pop_blocking();


    printf("Initialization complete!\n");


    //DEBUG: Validation cosines
    for (uint16_t x=0; x < DataArray_len; x++) {
        // ((uint16_t*)data_buffer0)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
        // ((uint16_t*)data_buffer1)[i] = (uint16_t)floor((2*1000 * cos(2 * M_PI * i))) + 2.5;
        // Normalized period to length of data array => freq. * 2pi/len * x
        uint16_t y = 1000.0f * ((2.5f * cosf(7.5f * (2.0f*M_PI) * x / DataArray_len)) + 2.5f);
        uint16_t y2 = 1000.0f * ((0.75f * cosf(15.0f * (2.0f*M_PI) * x / DataArray_len)) + 3.0f);
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
        if (c >= 10) {
            c = 0;
            end_avg = end;
            fps = ((float)(10*1000*1000)/(end - start_avg));
            // printf("FPS: %.03f    Avg: %.02f ms    Max: %.02f ms    Min: %.02f ms\n", (float)((10.0f*1000.0f*1000.0f)/(end - start_avg)), (float)((end - start_avg)/(20.0f*1000)), (float)(max/1000.0f), (float)(min/1000.0f));
            max = 0;
            min = UINT32_MAX;
            start_avg = time_us_32();
        }
        start = time_us_32();

        //convert to microseconds 1Hz = 1000,000us and convert to number of samples
        NumSamples = HorizontalScale / (1000*1000 / SamplingRate);

        // DEBUG: Validation cosines
        // IMPORTANT: (Wiggle wiggle wiggle)
        // const float phase1 = 2.0f * ((float)get_rand_32() / UINT32_MAX); // jitter is 200.0% of period
        // const float phase2 = 0.10f * ((float)get_rand_32() / UINT32_MAX); // jitter is 10.0% of period
        // for (uint x=0; x < DataArray_len; x++) {
        // for (uint x=0; x < DataArray_len; x+=2) { // Half resolution
        //     // Normalized period to length of data array => freq. * 2pi/len * x
        //     uint16_t y = 1000.0f * ((2.5f * cosf((7.5f * (2.0f*M_PI) * x / DataArray_len) + (phase1*(4.0f*M_PI)))) + 2.5f);
        //     uint16_t y2 = 1000.0f * ((0.75f * cosf((15.0f * (2.0f*M_PI) * x / DataArray_len) + (phase2*(4.0f*M_PI)))) + 3.0f);
        //     DataArray0[x] = y;
        //     DataArray1[x] = y2;
        //     if (x+1 < DataArray_len) {
        //         DataArray0[x+1] = y;
        //         DataArray1[x+1] = y2;
        //     }
        // }

        // Reset Stats
        VC1_stats.max = 0;
        VC1_stats.min = UINT16_MAX;
        VC2_stats.max = 0;
        VC2_stats.min = UINT16_MAX;

        //DISPLAY DRIVER BLOCK
        DispDriverFrameSetup();

        bool TriggerFlag = false;
        uint16_t TriggerLocation = DataArray_len;
        for (uint16_t i = 0; (i < (NumSamples + TriggerLocation) && i < DataArray_len); i++) {
            if (i && !TriggerFlag) {
                // Trigger on rising edge and falling edge
                // if (((DataArray0[i - 1] < TriggerVoltage) && (DataArray0[i] > TriggerVoltage)) ||
                //     ((DataArray0[i - 1] > TriggerVoltage) && (DataArray0[i] < TriggerVoltage))) {
                // Trigger on rising edge
                if ((DataArray0[i - 1] < TriggerVoltage) && (DataArray0[i] > TriggerVoltage)) {
                    // Take as many samples as needed
                    TriggerFlag = true;
                    TriggerLocation = i;
                }
            }
            if (TriggerFlag) {
        //DATA PROCESSOR BLOCK
                DataProcessor(DataArray0[i], DataArray1[i]);
                //DISPLAY DRIVER BLOCK
                DispDriver(DataArray0[i], DataArray1[i], NumSamples, HorizontalScale, VerticalScale, (i - TriggerLocation));
            } else { // Always draw the trigger line
                const uint32_t x = ((300 * i) / NumSamples) + 10;
                if (x < 310) {
                    int16_t y_trigger = 200 - (int16_t)truncf(TriggerVoltage * (20.0f/VerticalScale));
                    y_trigger = ((y_trigger > 100) ? y_trigger : 100); // max()
                    LCD_WaitForDMA();
                    if (x % 2) GFX_drawPixel(x, y_trigger, light_yellow);
                }
            }
            // //DATA PROCESSOR BLOCK
            // DataProcessor(DataArray0, DataArray1, NumSamples, VerticalScale, TriggerVoltage);
            // //DISPLAY DRIVER BLOCK
            // LCD_WaitForDMA();
            // DispDriver(DataArray0, DataArray1, NumSamples, HorizontalScale, VerticalScale, TriggerVoltage, fps);
        }

        //DISPLAY DRIVER BLOCK     
        DispDriverFrameFinish(HorizontalScale, VerticalScale, TriggerVoltage, fps);
        

        end = time_us_32();
        volatile uint32_t t = end - start;
        if (t > max) max = t;
        if (t < min) min = t;
        c++;
    }
}

void core1_main() {

    seesaw_setup();

    // Signal to core 0 that we're done with initialization and ready to go
    multicore_fifo_push_blocking(1);

    // Main loop for core 1
    while (true) {
        seesaw_run();

        // Warning! Certified Bullshit Ahead...

        int32_t t = HorizontalScale / 500;
        seesaw_update_val(0, &t);
        HorizontalScale = abs(t) * 500;
        // 
        t = VerticalScale / 100;
        seesaw_update_val(1, &t);
        VerticalScale = abs(t) * 100;
        // 
        t = TriggerVoltage / 100;
        seesaw_update_val(2, &t);
        TriggerVoltage = abs(t) * 100;

        if (HorizontalScale < 500)
            HorizontalScale = 500;
        if (HorizontalScale > 100*1000)
            HorizontalScale = 100*1000;
        if (VerticalScale < 100)
            VerticalScale = 100;
        if (VerticalScale > 5*1000)
            VerticalScale = 5*1000;
        if (TriggerVoltage < 100)
            TriggerVoltage = 100;
        if (TriggerVoltage > 25*1000)
            TriggerVoltage = 25*1000;
    }
}




const uint16_t white = GFX_RGB565(255,255,255);
const uint16_t grey = GFX_RGB565(127,127,127);
const uint16_t dark_grey = GFX_RGB565(20,20,20);
const uint16_t red = GFX_RGB565(255,0,0);
const uint16_t green = GFX_RGB565(0,255,0);
const uint16_t blue = GFX_RGB565(0,0,255);
const uint16_t yellow = (green | red);



//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER FRAME SETUP
//DESC: Sets up the layouts of the frame

void DispDriverFrameSetup() {

    GFX_setCursor(0, 0);

    LCD_WaitForDMA();

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
//DESC: Graphs two waveforms on top of the layout set in the frame setup function
//voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
void DispDriver(uint16_t VC1, uint16_t VC2, uint NS, float HS, uint16_t VS, uint16_t i) {
    // Draw waveforms to disp buffer
    // Width in pixels * sample# / number of samples
    const uint32_t x = ((300 * i) / NS) + 10;

    int16_t y1 = 200 - (int16_t)truncf(VC1 * (20.0f/VS));
    y1 = ((y1 > 100) ? y1 : 100); // max()
    int16_t y2 = 200 - (int16_t)truncf(VC2 * (20.0f/VS));
    y2 = ((y2 > 100) ? y2 : 100); // max()

    int16_t y_trigger = 200 - (int16_t)truncf(TriggerVoltage * (20.0f/VS));
    y_trigger = ((y_trigger > 100) ? y_trigger : 100); // max()

    LCD_WaitForDMA();
    if (x % 2) GFX_drawPixel(x, y_trigger, light_yellow);
    GFX_drawPixel(x, y1, red);
    GFX_drawPixel(x, y2, blue);
}


//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER FRAME FINISH
//DESC: finishes off the frame by displaying the calculated values and sends it to the display

void DispDriverFrameFinish(float HS, uint16_t VS, uint16_t Trigger, float fps) {

    GFX_setTextBack(dark_grey);
    // GFX_setTextColor(white);

    LCD_WaitForDMA();

    // Print FPS
    GFX_setTextColor(red);
    GFX_setCursor(5*(63-11),0);
    fps = fminf(fps, 99.999f);
    GFX_printf("FPS: %1.03f\n", fps);

    // Reset
    GFX_setTextColor(white);
    GFX_setCursor(0,0);

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
void DataProcessor(uint16_t VC1, uint16_t VC2) {

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
}
    


void overclock(bool verbose) {
    // Overclock core
    const bool VERBOSE_OVERCLOCK = verbose; // This is stupid
    int ret = overclock_core(OC_PRESETS[OC_FREQ_270], VERBOSE_OVERCLOCK); // WARNING: Requires 1.20V - Do at your own risk!
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_270_1_25V], VERBOSE_OVERCLOCK); // WARNING: Requires 1.25V - Do at your own risk!
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_270_1_30V], VERBOSE_OVERCLOCK); // WARNING: Requires 1.30V - Do at your own risk!
    // This is a good choice for creating the most sample rate options using the DMA timer
    // int ret = overclock_core(OC_PRESETS[OC_FREQ_260], VERBOSE_OVERCLOCK); // Use this
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



// //--------------------------------------------------------------------------------------------------------------------------
// //FUNCTION: DISPLAY DRIVER
// //DESC: Example descriptions
// //voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
// void DispDriver(uint16_t* VC1, uint16_t* VC2, uint NS, float HS, uint16_t VS, uint16_t Trigger, float fps) {
//     const uint16_t white = GFX_RGB565(255,255,255);
//     const uint16_t grey = GFX_RGB565(127,127,127);
//     const uint16_t dark_grey = GFX_RGB565(50,50,50);
//     const uint16_t red = GFX_RGB565(255,0,0);
//     const uint16_t blue = GFX_RGB565(0,0,255);

//     // GFX_clearScreen();
//     GFX_fillScreen(dark_grey);
//     GFX_setCursor(0, 0);
//     GFX_setTextBack(dark_grey);
//     GFX_setTextColor(white);

//     //value prints
//     GFX_setTextColor(red);
//     GFX_setCursor(5*(63-11),0);
//     fps = fminf(fps, 99.999f);
//     GFX_printf("FPS: %1.03f\n", fps);
//     GFX_setTextColor(white);
//     GFX_setCursor(0,0);
//     // Channel 1 Max
//     GFX_printf("Ch. 1 Max: ");
//     if (VC1_stats.max >= 1000) GFX_printf("%.02f V\n", (float)(VC1_stats.max / 1000.0f)); // V
//     else GFX_printf("%03.00f mV\n", (float)(VC1_stats.max)); // mV
//     // Channel 1 Min
//     GFX_printf("Ch. 1 Min: ");
//     if (VC1_stats.min >= 1000) GFX_printf("%.02f V\n", (float)(VC1_stats.min / 1000.0f)); // V
//     else GFX_printf("%3.00f mV\n", (float)(VC1_stats.min)); // mV
//     // Channel 2 Max
//     GFX_printf("Ch. 2 Max: ");
//     if (VC2_stats.max >= 1000) GFX_printf("%.02f V\n", (float)(VC2_stats.max / 1000.0f)); // V
//     else GFX_printf("%3.00f mV\n", (float)(VC2_stats.max)); // mV
//     // Channel 2 Min
//     GFX_printf("Ch. 2 Min: ");
//     if (VC2_stats.min >= 1000) GFX_printf("%.02f V\n", (float)(VC2_stats.min / 1000.0f)); // V
//     else GFX_printf("%3.00f mV\n", (float)(VC2_stats.min)); // mV
//     // Trigger
//     GFX_printf("Trigger: ");
//     if (Trigger >= 1000) GFX_printf("%.02f V\n", (float)(Trigger / 1000.0f)); // V
//     else GFX_printf("%3.00f mV\n", (float)(Trigger)); // mV
//     // Vertical Scale
//     GFX_printf("Volt. Scale: ");
//     if (VS >= 1000) GFX_printf("%.02f V/div\n", (float)(VS / 1000.0f)); // V
//     else GFX_printf("%3.00f mV/div\n", (float)(VS)); // mV
//     // Horizontal Scale
//     GFX_printf("Time Scale: ");
//     if (HS >= 1000) GFX_printf("%.01f ms/div\n", (float)(HS / 1000.0f)); // mS
//     else GFX_printf("%3.00f us/div\n", (float)(HS)); // uS//value prints

//     // GFX_flush_rows(0, 100);

//     //voltage markings
//     GFX_drawFastHLine(10, 120, 300, grey);
//     GFX_drawFastHLine(10, 140, 300, grey);
//     GFX_drawFastHLine(10, 160, 300, grey);
//     GFX_drawFastHLine(10, 180, 300, grey);
//     //time markings
//     GFX_drawFastVLine(40, 100, 100, grey);
//     GFX_drawFastVLine(70, 100, 100, grey);
//     GFX_drawFastVLine(100, 100, 100, grey);
//     GFX_drawFastVLine(130, 100, 100, grey);
//     GFX_drawFastVLine(160, 100, 100, grey);
//     GFX_drawFastVLine(190, 100, 100, grey);
//     GFX_drawFastVLine(220, 100, 100, grey);
//     GFX_drawFastVLine(250, 100, 100, grey);
//     GFX_drawFastVLine(280, 100, 100, grey);

//     // Draw waveforms to disp buffer
//     for (uint32_t i=0; i < NS && i < DataArray_len; i++){
//         // Width in pixels * sample# / number of samples
//         const uint32_t x = ((300 * i) / NS) + 10;
//         int16_t y = 200 - (int16_t)truncf(VC1[i] * (20.0f/VS));
//         y = ((y > 100) ? y : 100); // Pick larger number
//         GFX_drawPixel(x, y, red);
//         y = 200 - (int16_t)truncf(VC2[i] * (20.0f/VS));
//         y = ((y > 100) ? y : 100); // Pick larger number
//         GFX_drawPixel(x, y, blue);
//     }

//     //box around functions
//     GFX_drawFastHLine(10, 100, 300, white);
//     GFX_drawFastHLine(10, 200, 300, white);
//     GFX_drawFastVLine(10, 100, 100, white);
//     GFX_drawFastVLine(310, 100, 100, white);

//     // GFX_flush_rect(10, 100, 300, 100);
//     // LCD_WaitForDMA();

    

//     // GFX_flush_rows(100, 100);
//     // LCD_WaitForDMA();



//     //SEND ITTTTT!!!
//     GFX_flush();

// }

// //--------------------------------------------------------------------------------------------------------------------------
// //FUNCTION: DATA PROCESSOR
// //DESC: Takes in two arrays of data and does some math. spits them out on the other side
// //voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
// void DataProcessor(uint16_t* VC1, uint16_t* VC2, uint NS, uint16_t VS, uint16_t Trigger) {
//     //VC1 Variables
//     uint16_t MinVC1 = UINT16_MAX;
//     uint16_t MaxVC1 = 0;
//     bool TriggerFlagVC1 = 0;
//     int NumTriggersVC1 = 0;
    
//     //VC2 Variables
//     uint16_t MinVC2 = UINT16_MAX;
//     uint16_t MaxVC2 = 0;
//     bool TriggerFlagVC2 = 0;
//     int NumTriggersVC2 = 0;
    
//     //loop from the start to the end of the arrays and calculate important values
//     for (int i=0; i < NS && i < DataArray_len; i++) {
        
//         if(VC1[i] > MaxVC1){
//             MaxVC1 = VC1[i];
//         }
        
//         if(VC2[i] > MaxVC2){
//             MaxVC2 = VC2[i];
//         }
        
//         if(VC1[i] < MinVC1){
//             MinVC1 = VC1[i];
//         }

//         if(VC2[i] < MinVC2){
//             MinVC2 = VC2[i];
//         }
//     }


//     VC1_stats.max = MaxVC1;
//     VC1_stats.min = MinVC1;
//     VC1_stats.NumTriggers = NumTriggersVC1;

//     VC2_stats.max = MaxVC2;
//     VC2_stats.min = MinVC2;
//     VC2_stats.NumTriggers = NumTriggersVC2;


//     // //Use the last couple of slots for important values
//     // VC1[NS] = MaxVC1;
//     // VC1[NS + 1] = MinVC1;
//     // VC1[NS + 2] = NumTriggersVC1;
   
//     // //DEBUG: Serial PRint of special values
//     // //printf("VC1: The Max is %f\n", VC1[NS]);
//     // //printf("VC1: The Min is %f\n", VC1[NS + 1]);
//     // //printf("VC1: The Number of Triggers is %f\n", VC1[NS + 2]);
    
//     // VC2[NS] = MaxVC2;
//     // VC2[NS + 1] = MinVC2;
//     // VC2[NS + 2] = NumTriggersVC2;

//     // //DEBUG: Serial PRint of special values
//     // //printf("VC2: The Max is %f\n", VC2[NS]);
//     // //printf("VC2: The Min is %f\n", VC2[NS + 1]);
//     // //printf("VC2: The Number of Triggers is %f\n", VC2[NS + 2]);

// }

