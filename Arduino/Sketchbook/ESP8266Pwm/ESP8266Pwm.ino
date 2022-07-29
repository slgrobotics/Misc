
// See https://randomnerdtutorials.com/esp8266-pwm-arduino-ide/

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
#define D8 15 // SPI Bus SS (CS) - use D0 instead
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

#define SD1 8   // SDD1
#define SD0 7   // SDD0, MISO
#define SD2 9   // SDD2 - do not use!
#define SD3 10  // SDD3

#define SDCLK 6  // SDCLK, CLK
#define SDCMD 11 // SDCMD, CMD

// See https://www.electronicwings.com/nodemcu/nodemcu-spi-with-arduino-ide
// Note: D8/HCS - boot fails if it is pulled high, see https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
// If you want to use NodeMCU pin 5, use D5 for pin number, and it will be translated to 'real' GPIO pin 14.

// Built in LED:
const int pinLED = D0;  // HiLetgo, NodeMCU D0 
//const int pinLED = 0;   // Adafruit Feather HUZZAH

#include <Shifty.h>

// uses Shifty Arduino library
//      https://www.arduino.cc/reference/en/libraries/shifty/
//      https://github.com/johnnyb/Shifty
//

// Declare the shift register (74HC595)
Shifty shift;

const int latchPin = D0;  // Latch pin 12 of 74HC595
const int clockPin = D3;  // Clock pin 11 of 74HC595
const int dataPin = SD3;  // Data pin 14 of 74HC595

// we command PWM directly, and dir/stop/brake via shift register
const int rightPwmPin = D8;
const int leftPwmPin = D7;

// bit 0 connected to yellow LED for testing
const int testBit = 0;
const int rightDirBit = 1;
const int leftDirBit = 2;
const int stopBit = 3;
const int brakeBit = 4;

const int delayMs = 20;
bool dir = true;  // true - forward
int power = 0;    // max 255

void setup()
{
  Serial.begin(115200);   // start serial 
  delay(100);

  // Set the number of bits you have (multiples of 8)
  shift.setBitCount(8);

  // Set the data, clock, and latch pins you are using
  // This also sets the pinMode for these pins
  //shift.setPins(11, 12, 8); // data on pin 11, clock on pin 12, and latch on pin 8
  shift.setPins(dataPin, clockPin, latchPin); 

  pinMode(rightPwmPin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);

  // make sure motors are stopped:
  analogWrite(rightPwmPin, 0);
  analogWrite(leftPwmPin, 0);

  shift.batchWriteBegin();
  shift.writeBit(testBit, LOW);
  shift.writeBit(stopBit, LOW);
  shift.writeBit(brakeBit, LOW);
  shift.writeBit(rightDirBit, LOW);
  shift.writeBit(leftDirBit, LOW);
  shift.batchWriteEnd();
}

// Generated PWM frequency will be 1.00 kHz - quite precise.
// ESP devices are 3.3V levels

void loop()
{
  for(; power < 255; power++){   
    motorCycle(dir, power);
  }

  for(; power > 0; power--){
    motorCycle(dir, power);
  }

  dir = !dir;

  //shift.writeBit(stopBit, HIGH);  // feather the wheels, no matter what other pins are
  //shift.writeBit(brakeBit, HIGH); // apply braking torque, resisting rotation moderately
  
}

void motorCycle(bool dir, int power)
{
  shift.batchWriteBegin();
  shift.writeBit(testBit, dir ? LOW : HIGH);
  shift.writeBit(rightDirBit, dir ? LOW : HIGH);
  shift.writeBit(leftDirBit, dir ? HIGH : LOW);
  shift.batchWriteEnd();
  
  analogWrite(rightPwmPin, power);
  analogWrite(leftPwmPin, power);
  delay(delayMs);
}
