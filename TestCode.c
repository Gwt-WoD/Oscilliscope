#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "stdlib.h"
#include "math.h"
#include "time.h"
#include "pico-displayDrivs/ili9341/ili9341.h"
#include "pico-displayDrivs/gfx/gfx.h"
#include "pico-displayDrivs/gfx/font.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_RST 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_TX 19
#define PIN_DC 20

#include "blink.pio.h"

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    blink_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);

    printf("Blinking pin %d at %d Hz\n", pin, freq);

    // PIO counter program takes 3 more cycles in total than we pass as
    // input (wait for n + 1; mov; jmp)
    pio->txf[sm] = (125000000 / (2 * freq)) - 3;
}

//void core1_main(){
    // gpio_init(20);
    // gpio_init(22);
    // gpio_init(25);
    // gpio_set_dir(20, GPIO_IN);
    // gpio_set_dir(22, GPIO_OUT);
    // gpio_set_dir(25, GPIO_OUT);
    // bool g;
    // while(1){
    //     gpio_put(22,g);
    //     gpio_put(25,g);
    //     g = !g;
    //     sleep_ms(2000);
    // }
// }

//--------------------------------------------------------------------------------------------------------------------------
//FUNCTION: DISPLAY DRIVER
//DESC: Example descriptions
//voltage channel 1, voltage channel 2, number of samples, horizontal scale(us), vertical scale(mV), trigger voltage(mV)
void DispDriver(float* VC1, float* VC2, int NS, float HS, float VS, float Trigger){

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
void DataProcessor(float* VC1, float* VC2, int NS, float VS, float Trigger)
{
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

int main()
{
    
    //looking for USB connection. Waiting until serial is opened
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }
    sleep_ms(1000);
    

    //DEBUG: Srand for if we want to do random stuff
    srand(time(NULL));

    //launch in multicore mode    
    //multicore_launch_core1(core1_main);

    
    
    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 16*1000*1000);
    gpio_set_function(PIN_RST, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
    gpio_set_function(PIN_DC, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi



    // PIO Blinking example
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    printf("Loaded program at %d\n", offset);
    blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
    
    

    //TODO: testing time :)  
    
    LCD_initDisplay();
    LCD_setRotation(1);
    GFX_createFramebuf();

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
