/*

  Based on https://github.com/iavorvel/MyLD2410 "sensor_data" example

  Compile for ESP32 Devkitc V4 -- Arduino IDE choose "DOIT ESP32 DEVKIT V1"
  Install "MyLD2410" library - using Library Manager or manually
  
  Opens LD2410B, prints:

    Both moving and stationary, distance: 64cm
     MOVING    = 91@69cm 
     signals->[ 51 91 35 88 68 16 6 12 7 ] thresholds:[ 50 50 40 30 20 15 15 15 15 ]
     STATIONARY= 100@69cm 
     signals->[ 0 0 100 100 100 100 100 55 26 ] thresholds:[ 0 0 40 40 30 30 20 20 20 ]
    Light level: 105
    Output level: HIGH

 =================================   
  
  This program reads all data received from
  the HLK-LD2410 presence sensor and periodically
  prints the values to the serial monitor.
  
  Several #defines control the behavior of the program:
  #define SERIAL_BAUD_RATE sets the serial monitor baud rate
  #define ENHANCED_MODE enables the enhanced (engineering)
  mode of the sensor. Comment that line to switch to basic mode.
  #define DEBUG_MODE enables the printing of debug information
  (all reaceived frames are printed). Comment the line to disable
  debugging.

  Communication with the sensor is handled by the 
  "MyLD2410" library Copyright (c) Iavor Veltchev 2024

  Use only hardware UART at the default baud rate 256000,
  or change the #define LD2410_BAUD_RATE to match your sensor.
  For ESP32 or other boards that allow dynamic UART pins,
  modify the RX_PIN and TX_PIN defines

  Connection diagram:
  Arduino/ESP32 RX  -- TX LD2410 
  Arduino/ESP32 TX  -- RX LD2410
  Arduino/ESP32 GND -- GND LD2410
  Provide sufficient power to the sensor Vcc (200mA, 5-12V) 
*/
#if defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_LEONARDO)
//ARDUINO_SAMD_NANO_33_IOT RX_PIN is D1, TX_PIN is D0
//ARDUINO_AVR_LEONARDO RX_PIN(RXI) is D0, TX_PIN(TXO) is D1
#define sensorSerial Serial1
#elif defined(ARDUINO_XIAO_ESP32C3) || defined(ARDUINO_XIAO_ESP32C6)
//RX_PIN is D7, TX_PIN is D6
#define sensorSerial Serial0
#elif defined(ESP32)
//Other ESP32 device - choose available GPIO pins
#define sensorSerial Serial1
#if defined(ARDUINO_ESP32S3_DEV)
#define RX_PIN 18
#define TX_PIN 17
#else
#define RX_PIN 16
#define TX_PIN 17
#endif
#else
#error "This sketch only works on ESP32, Arduino Nano 33IoT, and Arduino Leonardo (Pro-Micro)"
#endif

// User defines
// #define DEBUG_MODE
#define ENHANCED_MODE
#define SERIAL_BAUD_RATE 115200

//Change the communication baud rate here, if necessary
#include "MyLD2410.h"
//#define LD2410_BAUD_RATE 256000

#ifdef DEBUG_MODE
MyLD2410 sensor(sensorSerial, true);
#else
MyLD2410 sensor(sensorSerial);
#endif

unsigned long nextPrint = 0, printEvery = 1000;  // print every second

void printValue(const byte &val) {
  Serial.print(' ');
  Serial.print(val);
}

void printData() {
  Serial.print(sensor.statusString());
  if (sensor.presenceDetected()) {
    Serial.print(", distance: ");
    Serial.print(sensor.detectedDistance());
    Serial.print("cm");
  }
  Serial.println();

  if (sensor.movingTargetDetected()) {

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    
    Serial.print(" MOVING    = ");
    Serial.print(sensor.movingTargetSignal());
    Serial.print("@");
    Serial.print(sensor.movingTargetDistance());
    Serial.print("cm ");
    if (sensor.inEnhancedMode()) {
      Serial.print("\n signals->[");
      sensor.getMovingSignals().forEach(printValue);
      Serial.print(" ] thresholds:[");
      sensor.getMovingThresholds().forEach(printValue);
      Serial.print(" ]");
    }
    Serial.println();
  } else {
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off
  }

  if (sensor.stationaryTargetDetected()) {
    Serial.print(" STATIONARY= ");
    Serial.print(sensor.stationaryTargetSignal());
    Serial.print("@");
    Serial.print(sensor.stationaryTargetDistance());
    Serial.print("cm ");
    if (sensor.inEnhancedMode()) {
      Serial.print("\n signals->[");
      sensor.getStationarySignals().forEach(printValue);
      Serial.print(" ] thresholds:[");
      sensor.getStationaryThresholds().forEach(printValue);
      Serial.print(" ]");
    }
    Serial.println();
  }

  if (sensor.inEnhancedMode() && (sensor.getFirmwareMajor() > 1)) { 
    Serial.print("Light level: ");
    Serial.println(sensor.getLightLevel());
    Serial.print("Output level: ");
    Serial.println((sensor.getOutLevel()) ? "HIGH" : "LOW");
    
    //digitalWrite(LED_BUILTIN, (sensor.getOutLevel()) ? HIGH : LOW);   // turn the LED on (HIGH is the voltage level)
  }

  Serial.println();
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
#if defined(ARDUINO_XIAO_ESP32C3) || defined(ARDUINO_XIAO_ESP32C6) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_LEONARDO)
  sensorSerial.begin(LD2410_BAUD_RATE);
#else
  sensorSerial.begin(LD2410_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
#endif
  delay(2000);
  Serial.println(__FILE__);
  if (!sensor.begin()) {
    Serial.println("Failed to communicate with the sensor.");
    while (true) {}
  }

#ifdef ENHANCED_MODE
  sensor.enhancedMode();
#else
  sensor.enhancedMode(false);
#endif

  pinMode(LED_BUILTIN, OUTPUT); // Pin 2

  delay(nextPrint);
}

void loop() {
  if ((sensor.check() == MyLD2410::Response::DATA) && (millis() > nextPrint)) {
    nextPrint = millis() + printEvery;
    printData();
  }
}
