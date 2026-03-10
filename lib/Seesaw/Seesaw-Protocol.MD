
# SeeSaw Protocol Notes

## SeeSaw I2C Command Format

| I2C Address    | Register Base  | Command Code   | Data Byte 0    | Data Byte 1    | ...            |
|----------------|----------------|----------------|----------------|----------------|----------------|
| 7 bits         | 1 byte         | 1 byte         | Optional       | Optional       | Optional       |

**Adafruit Seesaw I2C Command Format:**

- **I2C Address:** 7-bit address of the Seesaw device.
- **Register Base:** 1 byte indicating the functional block (e.g., GPIO, TIMER).
- **Command Code:** 1 byte specifying the operation within the block.
- **Data Bytes:** Optional data for the command (e.g., pin numbers, values).

*Example Write Sequence:*
```
[I2C Address][Register Base][Command Code][Data 0][Data 1]...[Data N]
```
*Example Read Sequence:*
```
[I2C Address][Register Base][Command Code] → [Read Data 0][Read Data 1]...[Read Data N]
```

## Writing Data

A seesaw write command consists of the standard I2C write header (with the R/W bit set to 0), followed by 2 register bytes followed by zero or more data bytes.

The first register byte is the module base register address. Each module (GPIO, ADC, DAC, etc.) has it's own unique 8 bit base identifier.

The second register byte is the module function register address. This byte specifies the desired register within the module to be written.

Thus we have up to 254 modules available (0x00 is reserved) and 255 functions per module - plenty to allow all sorts of different capabilities!

In code, this may look like this (using the Arduino wire I2C object):

```cpp
void Adafruit_seesaw::write(uint8_t moduleBase, uint8_t moduleFunction, uint8_t *buf, uint8_t num)
{ 
	Wire.beginTransmission((uint8_t)_i2caddr);
	Wire.write((uint8_t)moduleBase); //module base register address
	Wire.write((uint8_t)moduleFunction); //module function register address
	Wire.write((uint8_t *)buf, num); //data bytes
	Wire.endTransmission();
}
```

## Reading Data
A register read is accomplished by first sending the standard I2C write header, followed by the two register bytes corresponding to the data to be read. Allow a short delay, and then send a standard I2C read header (with the R/W bit set to 1) to read the data.

The length of the required delay depends on the data that is to be read. These delays are discussed in the sections specific to each module.

In code, this may look like this  (using the Arduino wire I2C object):

```cpp
void Adafruit_seesaw::read(uint8_t moduleBase, uint8_t moduleFunction, uint8_t *buf, uint8_t num, uint16_t delay)
{
  Wire.beginTransmission((uint8_t)_i2caddr);
  Wire.write((uint8_t)moduleBase); //module base register address
  Wire.write((uint8_t)moduleFunction); //module function register address
  Wire.endTransmission();

  delayMicroseconds(delay);

  Wire.requestFrom((uint8_t)_i2caddr, num);

  for(int i=0; i<num; i++){
    buf[i] = Wire.read();
  }
}
```

## Base Register Summery

| Register Name           | Address (Hex) | Description                          |
|-------------------------|---------------|--------------------------------------|
| STATUS_BASE             | 0x00          | Status and information               |
| GPIO_BASE               | 0x01          | GPIO control                         |
| SERCOM0_BASE            | 0x02          | SERCOM (I2C, UART, SPI)              |
| TIMER_BASE              | 0x08          | Timer functions                      |
| ADC_BASE                | 0x09          | Analog-to-digital converter          |
| DAC_BASE                | 0x0A          | Digital-to-analog converter          |
| INTERRUPT_BASE          | 0x0B          | Interrupt control                    |
| DAP_BASE                | 0x0C          | Debug Access Port                    |
| EEPROM_BASE             | 0x0D          | EEPROM access                        |
| NEOPIXEL_BASE           | 0x0E          | NeoPixel (WS2812) control            |
| TOUCH_BASE              | 0x0F          | Touch input                          |
| KEYPAD_BASE             | 0x10          | Keypad matrix                        |
| ENCODER_BASE            | 0x11          | Rotary encoder                       |

**Note:** Each base register may have sub-registers for specific functions. Refer to the Adafruit Seesaw documentation for detailed usage.

**Note:** Command codes are used in conjunction with the register base addresses. Refer to the Seesaw datasheet for full command structure and additional commands.

## Status Function

### Function Register Summary

| Register Address | Register Name | Register Size | R/W  | Description                                |
|------------------|---------------|---------------|------|--------------------------------------------|
| 0x01             | HW_ID         | 1 byte        | R    | Get Hardware ID Code                       |
| 0x02             | VERSION       | 4 bytes       | R    | Get Product Code and Date Code             |
| 0x03             | OPTIONS       | 4 bytes       | R    | Read if spesific options have been enabled |
| 0x04             | TEMP          | 4 bytes       | R    | Read the on board Temperature              |
| 0x7F             | SWRST         | 0 bytes       | W    | Initiate a software reset                  |

### Function Register Description

**0x01 - HW_ID:**

| Byte 0           |
| ---------------- |
| Hardware ID Code |

A single byte value that provides the hardware specific Hardware ID Code.

**0x02 - VERSION:**

| Byte 0           | Byte 1           | Byte 2        | Byte 3        |
| ---------------- | ---------------- | ------------- | ------------- |
| Product Code MSB | Product Code LSB | Date Code MSB | Date Code LSB |

The Product Code is a unique 16 bit number assigned to each product.

The Date Code is a 16 bit value that indicates the year, month, and day of the firmware build:

Top 5 bits are day of the month (1-31)
Middle 4 bits are month (1-12)
Bottom 7 bits are 20xx year (00-99)
Here's a Python function for decoding a Date Code into year, month, and day:

```python
def datecode_decode(code):
    year = code & 0x7F           // bottom 7 bits are 20xx year (00-99)
    month = (code >> 7) & 0x0F   // middle 4 bits are month (1-12)
    day = (code >> 11) & 0x1F    // top 5 bits are day of month (1-31)
    return year, month, day
```

**0x03 - OPTIONS:**

| Byte 0      | Byte 1  | Byte 2  | Byte 3      |
| ----------- | ------- | ------- | ----------- |
| Options MSB | Options | Options | Options LSB |

Options is a 32 bit field register indicating if a specific option (ADC, PWM, etc.) has been enabled or not within the firmware. If enabled, the bit reads a 1, otherwise 0.

Bit locations for options are defined below:

```cpp
#define SEESAW_STATUS_BASE 0x00
#define SEESAW_GPIO_BASE 0x01
#define SEESAW_SERCOM0_BASE 0x02
#define SEESAW_SERCOM1_BASE 0x03
#define SEESAW_SERCOM2_BASE 0x04
#define SEESAW_SERCOM3_BASE 0x05
#define SEESAW_SERCOM4_BASE 0x06
#define SEESAW_SERCOM5_BASE 0x07
#define SEESAW_TIMER_BASE 0x08
#define SEESAW_ADC_BASE 0x09
#define SEESAW_DAC_BASE 0x0A
#define SEESAW_INTERRUPT_BASE 0x0B
#define SEESAW_DAP_BASE 0x0C
#define SEESAW_EEPROM_BASE 0x0D
#define SEESAW_NEOPIXEL_BASE 0x0E
#define SEESAW_TOUCH_BASE 0x0F
#define SEESAW_KEYPAD_BASE 0x10
#define SEESAW_ENCODER_BASE 0x11
```

**0x04 - TEMP:**

| Byte 0          | Byte 1      | Byte 2      | Byte 3          |
| --------------- | ----------- | ----------- | --------------- |
| Temperature MSB | Temperature | Temperature | Temperature LSB |

The on board Temperature of the device, as a signed 32 bit integer

**0x7F - SWRST:**

Writing to the register address will initiate a software reset.

## GPIO Function

The GPIO module provides every day input and outputs. You'll get logic GPIO pins that can act as outputs or inputs. With pullups or pulldowns. When inputs, you can also create pin-change interrupts that are routed the the IRQ pin.

On SAMD09-based boards the GPIO is 3V only. On ATtiny-based boards, the GPIO logic is whatever the power pin is, 3V or 5V.

The module base register address for the GPIO module is 0x01.

### Function Register Summary

| Register Address | Register Name | Register Size | R/W  | Description                                |
|------------------|---------------|---------------|------|--------------------------------------------|
| 0x02             | DIRSET        | 32 bits       | W    |  |
| 0x03             | DIRCLR        | 32 bits       | w    |  |
| 0x04             | GPIO          | 32 bits       | R/W  |  |
| 0x05             | SET           | 32 bits       | W    |  |
| 0x06             | CLR           | 32 bits       | W    |  |
| 0x07             | TOGGLE        | 32 bits       | W    |  |
| 0x08             | INTENSET      | 32 bits       | W    |  |
| 0x09             | INTENCLR      | 32 bits       | W    |  |
| 0x0A             | INTFLAG       | 32 bits       | R    |  |
| 0x0B             | PULLENSET     | 32 bits       | W    |  |
| 0x0C             | PULLENCLR     | 32 bits       | W    |  |

Writes of GPIO function registers should contain 4 data bytes (32 bits) following the initial register data bytes. Each bit in these registers represents a GPIO pin on PORTA of the seesaw device.

If the corresponding pin does not exist on the SeeSaw device, then reading or writing the bit has no effect.

We decided to go with this method to make GPIO toggling fast (rather than having one i2c transaction per individual pin control) but the host processor will need to do a little work to keep the pins identified.

### Pin Mapping

**GPIO register setup on ATTiny8x7 (same as megaTinyCore pin mapping):**

| Bit  | 0   | 1   | 2   | 3   | 4   | 5   | 6   | 7   | 8   | 9   | 10  | 11  | 12  | 13  | 14  | 15  | 16  | 17  | 18  | 19  | 20  |
|------|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|
| Pin  | PA4 | PA5 | PA6 | PA7 | PB7 | PB6 | PB5 | PB4 | PB3 | PB2 | PB1 | PB0 | PC0 | PC1 | PC2 | PC3 | PC4 | PC5 | PA1 | PA2 | PA3 |

### Function Register Description

**DIRSET (0x02, 32 bits, Write Only):**

Writing a 1 to any bit in this register sets the direction of the corresponding pin to OUTPUT.

Writing zeros to this register has no effect.

**DIRCLR (0x03, 32 bits, Write Only):**

Writing a 1 to any bit in this register sets the direction of the corresponding pin to INPUT.

Writing zeros to this register has no effect.

**GPIO (0x04, 32 bits, Read/Write):**

When this register is written, all bits that are set to 0 will have their corresponding pins set LOW.

All bits that are set to 1 will have their corresponding pins set HIGH.

Reading this register reads all pins on of the seesaw device. On the Attiny series please wait at least 250uS between command write and data read to allow the data to be read and formatted for retrieval. Reading this register will also reset the IRQ pin if it was configured.

**SET (0x05, 32 bits, Write Only):**

Writing a 1 to any bit in this register writes the corresponding pin HIGH.

Writing zeros to this register has no effect.

**CLR (0x06, 32 bits, Write Only):**

Writing a 1 to any bit in this register writes the corresponding pin LOW.

Writing zeros to this register has no effect.

**TOGGLE (0x07, 32 bits, Write Only):**

Writing a 1 to any bit in this register toggles the corresponding pin.

Writing zeros to this register has no effect.

**INTENSET (0x08, 32 bits, Write Only):**

Writing a 1 to any bit in this register enables the interrupt on the corresponding pin. When the value on this pin changes, the corresponding bit will be set in the INTFLAG register.

Writing zeros to this register has no effect.

**INTENCLR (0x09, 32 bits, Write Only):**

Writing a 1 to any bit in this register disables the interrupt on the corresponding pin.

Writing zeros to this register has no effect.

**INTFLAG (0x0A, 32 bits, Read Only):**

This register hold the status of all GPIO interrupts. When an interrupt fires, the corresponding bit in this register gets set. Reading this register clears all interrupts. This will also reset the IRQ pin if it was configured.

Writing to this register has no effect.

**PULLENSET (0x0B, 32 bits, Write Only):**

Writing a 1 to any bit in this register enables the internal pullup or pulldown on the corresponding pin. The pull direction (up/down) is determined by the GPIO (output) value - if the corresponding GPIO register bit is low,  its a pulldown. High, its a pullup.

Writing zeros to this register has no effect.

**PULLENCLR (0x0C, 32 bits, Write Only):**

Writing a 1 to any bit in this register disables the pull up/down on the corresponding pin.

Writing zeros to this register has no effect.

## Encoder Function

### Function Register Summary

| Register Address | Register Name   | Register Size | R/W  | Description                           |
|------------------|-----------------|---------------|------|---------------------------------------|
| 0x00             | Status          | ?             | ?    | Read encoder status                   | Read encoder interrupt flags
| 0x10             | Interrupt Set   | 1 byte        | W    | Enable encoder interrupts             |
| 0x20             | Interrupt Clear | 1 byte        | W    | Disable encoder interrupts            |
| 0x30             | Position        | 4 bytes       | R/W  | Read encoder position                 |
| 0x40             | Delta           | 4 bytes       | R    | Read encoder movement since last read |

More than one encoder can be supported. The upper 4 bits of the function register address specifies the function (summarized in table above) while the lower 4 bits indicates the encoder number, starting with 0 for the 1st encoder. For example, to read the position (0x30) of the 2nd encoder (0x01), the resulting function register address would be 0x31 (0x30 | 0x01).

### Function Register Description

**0x00 - Status:**

Currently unused.

**0x10 - Interrupt Set:**

Writing a 1 to any bit in this register will enable the interrupt for the specified encoder. The interrupt will fire when the encoder changes position.

**0x20 - Interrupt Clear:**

Writing a 1 to any bit in this register will disable the interrupt for the specified encoder.

**0x30 - Position:**

| Byte 0       | Byte 1 | Byte 2 | Byte 3       |
| ------------ | ------ | ------ | ------------ |
| Position MSB | ...    | ...    | Position LSB |

Reading this register returns the current encoder position. Writing to this register sets the current position to the specified value. The position value is a signed 32 bit integer.

**0x40 - Delta:**

| Byte 0    | Byte 1 | Byte 2 | Byte 3    |
| --------- | ------ | ------ | --------- |
| Delta MSB | ...    | ...    | Delta LSB |

Reading this register returns the change (delta) in position, as a signed 32 bit integer value, since the last read. The delta will also be reset to zero.

## NeoPixel Function

### Function Register Summary

| Register Address | Register Name | Register Size | R/W  | Description                                 |
|------------------|---------------|---------------|------|---------------------------------------------|
| 0x01             | PIN           | 8 bits        | W    | Set NeoPixel output pin                     |
| 0x02             | SPEED         | 8 bits        | W    | Set NeoPixel speed (400/800kHz)             |
| 0x03             | BUF_LENGTH    | 16 bits       | W    | Set NeoPixel pixel buffer (index, RGB data) |
| 0x04             | BUF           | 32 bytes      | W    | Set NeoPixel brightness (0-255)             |
| 0x05             | SHOW          | none          | W    | Show NeoPixel buffer                        |

### Function Register Description

**PIN (0x01, 8bits, Write Only):**

This register sets the pin number (PORTA) that is used for the NeoPixel output.

**SPEED (0x02, 8bits, Write Only):**

The protocol speed.

0x00 = 400khz

0x01 = 800khz (default)

**BUF_LENGTH (0x03, 16bits LE, Write Only):**

the number of bytes currently used for the pixel array. This is dependent on when the pixels you are using are RGB or RGBW. 2 Bytes, little endian order

**BUF (0x04, 32 bytes, Write Only):**

The data buffer. The first 2 bytes are the start address, and the data to write follows. Data should be written in blocks of maximum size 30 bytes at a time.

| Bytes 0 - 1   | Bytes 2 - 32 |
|---------------|--------------|
| Start address | Data         |

**SHOW (0x05, no args, Write Only):**

Sending the SHOW command will cause the output to update. There's no arguments/data after the command
