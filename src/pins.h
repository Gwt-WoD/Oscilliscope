#ifndef PINS_H
#define PINS_H

// Debug pins
#define USR_LED       0
#define USBBOOT_BTN   1
#define NEOPIXEL      2

#define TX_BTN_0      4
#define RX_BTN_1      5


// Display pins
#define DISP_SPI      spi0
#define DISP_PIN_SCK  18
#define DISP_PIN_MOSI 19
#define DISP_PIN_CS   21

#define DISP_PIN_DC   22
#define DISP_PIN_RST  27
#define DISP_PIN_INT  26
#define DISP_PIN_LITE 16

#define DISP_CARD_CS  17 // SD Card


// User Input pins
#define USR_INT_0     7
#define USR_INT_1     8


// AFE Pins
#define ADC_SPI       spi1
#define ADC_PIN_SCK   14
#define ADC_PIN_MISO  12
#define ADC_PIN_CS0   13
#define ADC_PIN_CS1   25

#define EXT_TRIG      6
#define TRIG_0        3
#define TRIG_1        15

#define AFE_RST       10
#define AFE_INT       11

#define AFE_FREQ_GEN  23


// Shared I2C Pins
#define I2C_PORT      i2c0
#define I2C_SDA       4
#define I2C_SCL       5


// TODO: Add More


#endif // PINS_H