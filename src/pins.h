// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define DISP_SPI      spi0
#define DISP_PIN_RST  16
#define DISP_PIN_CS   17
#define DISP_PIN_SCK  18
#define DISP_PIN_TX   19
#define DISP_PIN_DC   20


#define ADC_SPI       spi1
#define ADC_PIN_SCK   14
#define ADC_PIN_MISO  12
#define ADC_PIN_CS0   13
#define ADC_PIN_CS1   25

// TODO: Add More