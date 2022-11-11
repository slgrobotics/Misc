/*
 * Copyright (c) 2021..., Sergei Grichine
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *    
 * this is a no-warranty no-liability permissive license - you do not have to publish your changes,
 * although doing so, donating and contributing is always appreciated
 */

//#define TRACE

// Note Arduino Leonardo odd features:
//    1. boot takes 10 seconds, LED blinks (bootloader waiting). 
//          If it becomes a problem, see https://forum.arduino.cc/t/how-to-reduce-arduino-micro-leonardo-bootup-time/161222
//    2. when USB is disconnected, and TRACE defined it will hang or wait 250ms on calls to Serial.print()
   
#define LED_PIN 13
#define DBG1_PIN 2
#define DBG2_PIN 3

// Adjusted to work for SmartLab ReCon 6 - see RpiPico_ReCon6  - Sep 2021

#include <SPI.h>  
#include <Pixy.h>

// This is the main Pixy object.
// My Pixy camera was trained on 3 colors (signatures) - Orange, Yellow, Blue
Pixy pixy;

// Sonars library:
#include <NewPing.h>

volatile int sonarLF, sonarF, sonarRF;

void setup()
{
#ifdef TRACE
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for USB serial port to connect or timeout. Needed for native USB port only - Leonardo
  }
#endif // TRACE

  // Leonardo TX1 (5V TTL), connected via 1K/680Ohm voltage divider
  //    to Raspberry Pi Pico Pin 12, GP9 RX 3.3V TTL level!
  Serial1.begin(115200);

  // DEBUG pins:
  pinMode(LED_PIN, OUTPUT);
  pinMode(DBG1_PIN, OUTPUT);
  pinMode(DBG2_PIN, OUTPUT);

#ifdef TRACE
  Serial.print("IP: starting...\n");
#endif // TRACE

  pixyInit();

  sonarsInit();

#ifdef TRACE
  Serial.print("OK: setup completed\n");
#endif // TRACE
}

void loop()
{ 
  static int loopcnt = 0;

  digitalWrite(DBG1_PIN, HIGH); // start of loop

  if (loopcnt % 1000 == 0) 
  {
    //digitalWrite(DBG2_PIN, HIGH); // start of print cycle
    digitalWrite(LED_PIN,0);  // reset what was raised when Pixy blocks were found

    printSonars();    // we spend 10ms here. 
    
    //digitalWrite(DBG2_PIN, LOW); // end of print
  }

  //digitalWrite(DBG2_PIN, HIGH);
  processSonars();  // we spend 14us here, with rare occurencies of 500us
  //digitalWrite(DBG2_PIN, LOW);
  
  //digitalWrite(DBG2_PIN, HIGH);
  processPixy();
  //digitalWrite(DBG2_PIN, LOW);

  loopcnt++;

  // most of non-print loops take 100us. Some will be 200us. Rare ones - 600us.
  digitalWrite(DBG1_PIN, LOW); // end of loop
  // typically outside the loop we spend 4us. Can be as high as 17us.
}
