
// NodeMCU has weird pin mapping.
// Pin numbers written on the board itself do not correspond to ESP8266 GPIO pin numbers.

#define D0 16 // Connected to LED. GPIO16 does support PWM.
#define D1 5  // I2C Bus SCL (clock)
#define D2 4  // I2C Bus SDA (data)
#define D3 0
#define D4 2  // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO 
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS)
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

#define SD1 8   // SDD1
#define SD0 7   // SDD0, MISO
#define SD2 9   // SDD2 - do not use!
#define SD3 10  // SDD3

#define SDCLK 6  // SDCLK, CLK
#define SDCMD 11 // SDCMD, CMD

//If you want to use NodeMCU pin 5, use D5 for pin number, and it will be translated to 'real' GPIO pin 14.

int latchPin = D0;  // Latch pin 12 of 74HC595
int clockPin = D3; // Clock pin 11 of 74HC595
int dataPin = SD3;  // Data pin 14 of 74HC595

bool ledOn = true;    // LEDs are currently turned on or off

#include <Shifty.h>

// uses Shifty Arduino library
//      https://www.arduino.cc/reference/en/libraries/shifty/
//      https://github.com/johnnyb/Shifty
//

// Declare the shift register
Shifty shift; 

void setup() 
{
  // Set the number of bits you have (multiples of 8)
  shift.setBitCount(8);

  // Set the data, clock, and latch pins you are using
  // This also sets the pinMode for these pins
  //shift.setPins(11, 12, 8); // data on pin 11, clock on pin 12, and latch on pin 8
  shift.setPins(dataPin, clockPin, latchPin); 
}

// single-bit mode:
void loop() 
{
  int i = 0;
  
  for (; i < 8; i++) // Turn all the LEDs ON one by one.
  {
    shift.writeBit(i, HIGH);
    delay(500);
  }

  for (; i >= 0; --i) // Turn all the LEDs OFF one by one.
  {
    shift.writeBit(i, LOW);
    delay(500);
  }
}

/*
// batch mode:
void loop() 
{
  shift.batchWriteBegin();
  for (int i = 0; i < 8; i++)
  {
    shift.writeBit(i, ledOn);
    delay(50);
  }
  shift.batchWriteEnd();
  delay(500);

  ledOn = !ledOn;
}
*/
