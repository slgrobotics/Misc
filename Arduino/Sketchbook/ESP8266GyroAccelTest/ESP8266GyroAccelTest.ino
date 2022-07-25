/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read gyroscope data

   see https://github.com/EmotiBit/EmotiBit_BMI160 - does not work!
       http://www.esp8266learning.com/esp8266-bmi160-sensor-example.php
*/
      
// use https://github.com/hanyazou/BMI160-Arduino - install manually! EmotiBit_BMI160 does not work!
// Works on UNO and ESP8266 in SPI or I2C mode.

#include <BMI160Gen.h>

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
#define SD2 9   // SDD2
#define SD3 10  // SDD3

#define SDCLK 6  // SDCLK, CLK
#define SDCMD 11 // SDCMD, CMD


//If you want to use NodeMCU pin 5, use D5 for pin number, and it will be translated to 'real' GPIO pin 14.

// Built in LED:
int pinLED = 16;  // HiLetgo 
//int pinLED = 0;   // Adafruit Feather HUZZAH 

//const int bmi160csPin = 10; // Arduino UNO;
const int bmi160csPin = D0;
const int bmi160interruptPin = D1;  // not necessary to connect
const int i2c_addr = 0x69;  // see I2cdetect library example scan

/* 
 *  See https://www.electronicwings.com/nodemcu/nodemcu-spi-with-arduino-ide
   Connections for SPI mode:
      BMI160       NodeMCU
       CS            D0 GPIO16
       SCL           D5 HSCLK
       SA0           D6 HMISO
       SDA           D7 HMOSI
       INT1          D1 GPIO5 not necessary to connect
   Note: D8/HCS - boot fails if it is pulled high, see https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
 */
 
void setup() 
{
  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  delay(1000);

  // initialize device
  Serial.println("Initializing IMU device...");

  //BMI160.begin(bmi160csPin, bmi160interruptPin);  // same as below, SPI_MODE
  BMI160.begin(BMI160GenClass::SPI_MODE, bmi160csPin);
  //BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  Serial.print("BMI160.begin finished");

  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 250 degrees/second
  BMI160.setAccelerometerRange(2);  // supported values: 2, 4, 8, 16 (G)

  // Set the gyro range to 250 degrees/second
  BMI160.setGyroRange(250);        // supported values: 125, 250, 500, 1000, 2000 (degrees/second)
  Serial.println("Initializing IMU device...done.");
}

void loop() 
{
  //Serial.println("...loop...");
  
  int axRaw, ayRaw, azRaw;         // raw accelerometer values
  float ax, ay, az;

  int gxRaw, gyRaw, gzRaw;         // raw gyro values
  float gx, gy, gz;

  // read raw gyro measurements from device
  //BMI160.readGyro(gxRaw, gyRaw, gzRaw);
  BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  // convert the raw accelerometer data to G's
  ax = convertRawAccelerometer(axRaw);
  ay = convertRawAccelerometer(ayRaw);
  az = convertRawAccelerometer(azRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);

  // display tab-separated gyro x/y/z values
  Serial.print("a:\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.print(az);
  Serial.print("   **********   ");

  Serial.print("g:\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.print(gz);
  Serial.println();

  delay(500);
}

float convertRawAccelerometer(int aRaw) 
{
  // since we are using 2 G's range
  // -2 maps to a raw value of -32768
  // +2 maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

float convertRawGyro(int gRaw) 
{
  // since we are using 125 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}
