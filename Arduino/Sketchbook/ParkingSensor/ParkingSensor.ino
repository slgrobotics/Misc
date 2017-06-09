/* Parking Car Assist Ultrasound Sensor
 *------------------
 *
 * Reads values from a 4-sensor ultrasound unit (sold widely for around US $20 on Amazon)
 * and writes the values to the serial port.
 *
 * Copyright (c) 2012...2016..., Sergei Grichine   http://trackroamer.com, https://github.com/slgrobotics
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
 * although doing so, donating and contributing code and feedback is always appreciated
 * https://www.paypal.me/vitalbytes
 */
 
//#define PARKINGSENSOR_MODEL_1
#define PARKINGSENSOR_MODEL_2

#define USE_VOTING

#include <digitalWriteFast.h>
#include <Wire.h>


boolean runTest = true;           // blink all LEDs at startup


// all you need is to tap into the data wire that goes from sensor processor to display, and of course connect the ground.
// Car parking sensor decoding port; use pin 3 because it is connected to external interrupt 1 (INT1)
int mParkingSensorPin = 3;

int ledPin = 13;   // LED connected to digital pin 13
//int dbgPin = 8;  // digital pin 8 used for debugging

int ledFRYPin = 12; // LED - Front Right Yellow
int ledFRRPin = A0; // LED - Front Right Red
int ledFLYPin = 11; // LED - Front Left Yellow
int ledFLRPin = 10; // LED - Front Left Red

int ledBRYPin = 2;  // LED - Back Right Yellow
int ledBRRPin = 4;  // LED - Back Right Red
int ledBLYPin = 5;  // LED - Back Left Yellow
int ledBLRPin = 6;  // LED - Back Left Red

// readings in centimeters:
volatile int rangeFRcm;
volatile int rangeFLcm;
volatile int rangeBRcm;
volatile int rangeBLcm;

#ifdef USE_VOTING
byte rangeVotingMatrix[4][3]; // four channels, 3 voted positions
#endif // USE_VOTING


#ifdef PARKINGSENSOR_MODEL_1

unsigned long BETWEEN_PACKETS_MS = 30;   // time in milliseconds between the end of the packet and start of the next one (250ms), minus some.

// we expect data pulse max around 950us and min around 300us - actually the pulses are 400 and 800 us. Values below are min and max us/16:
unsigned long PULSE_MIN = 20;
unsigned long PULSE_MAX = 60;
byte BADPULSE_DISCRIMINATOR = 80;
byte ONEZERO_DISCRIMINATOR = 35;

#endif //PARKINGSENSOR_MODEL_1


#ifdef PARKINGSENSOR_MODEL_2

unsigned long BETWEEN_PACKETS_MS = 20;   // time in milliseconds between the end of the packet and start of the next one (30ms), minus 20-30%

// we expect data pulse max around 250us and min around 80us - actually the pulses are 100 and 200 us. Values below are min and max us/16:
unsigned long PULSE_MIN = 5;
unsigned long PULSE_MAX = 15;
byte BADPULSE_DISCRIMINATOR = 18;
byte ONEZERO_DISCRIMINATOR = 9;

#endif //PARKINGSENSOR_MODEL_2


// these are volatile because they are changed in interrupt handler.
// otherwise the compiler will assume they are unchanged and replace with constants, or put them in registers.
// volatile variables are always fetched from RAM and stored directly in RAM, no optimization applied.
// We only need to declare volatile those variables that are shared between interrupt routine and loop.
volatile boolean psiChanged;

// processed car parking sensors data:
volatile byte psiData[8];	// raw bits from psiRawData converted to distance codes (decimeters in fact) as reported by each of 4 (or 8? - assuming a 8-channel device will some time be used) sensors.

void setup()
{
  Serial.begin(115200);

  pinMode(mParkingSensorPin, INPUT);  // Set signal pin to input

  pinMode(ledPin, OUTPUT);            // Sets the digital pin as output
  //pinMode(dbgPin, OUTPUT);          // Sets the digital pin as output

  pinMode(ledFRYPin, OUTPUT);         // Set LED pins as output
  pinMode(ledFRRPin, OUTPUT);
  pinMode(ledFLYPin, OUTPUT);
  pinMode(ledFLRPin, OUTPUT);
  
  pinMode(ledBRYPin, OUTPUT);
  pinMode(ledBRRPin, OUTPUT);
  pinMode(ledBLYPin, OUTPUT);
  pinMode(ledBLRPin, OUTPUT);

  // we will get an interrupt on both falling and rising edges:  
  attachInterrupt(1, interrup_isr_onchange, CHANGE);
}

void loop() {
  if(runTest)
  {
    test();
    runTest = false;
  }
  
  if(psiChanged)
  {
    psiChanged = false;

    printValue();
    
    setLeds();
  }
  delay(50);
}

int redTreshold = 40;
int yellowTreshold = 80;

void setLeds()
{
  digitalWrite(ledFRYPin, LOW);
  digitalWrite(ledFRRPin, LOW);
  digitalWrite(ledFLYPin, LOW);
  digitalWrite(ledFLRPin, LOW);
  digitalWrite(ledBRYPin, LOW);
  digitalWrite(ledBRRPin, LOW);
  digitalWrite(ledBLYPin, LOW);
  digitalWrite(ledBLRPin, LOW);

  if(rangeFRcm >= 0)
  {
    if(rangeFRcm < redTreshold)
    {
      digitalWrite(ledFRRPin, HIGH);
    }
    else if(rangeFRcm < yellowTreshold)
    {
      digitalWrite(ledFRYPin, HIGH);
    }
  }
  
  if(rangeFLcm >= 0)
  {
    if(rangeFLcm < redTreshold)
    {
      digitalWrite(ledFLRPin, HIGH);
    }
    else if(rangeFLcm < yellowTreshold)
    {
      digitalWrite(ledFLYPin, HIGH);
    }
  }
    
  if(rangeBRcm >= 0)
  {
    if(rangeBRcm < redTreshold)
    {
      digitalWrite(ledBRRPin, HIGH);
    }
    else if(rangeBRcm < yellowTreshold)
    {
      digitalWrite(ledBRYPin, HIGH);
    }
  }
    
  if(rangeBLcm >= 0)
  {
    if(rangeBLcm < redTreshold)
    {
      digitalWrite(ledBLRPin, HIGH);
    }
    else if(rangeBLcm < yellowTreshold)
    {
      digitalWrite(ledBLYPin, HIGH);
    }
  }
}

// ===============================================================================
void test()
{
  for(int i=0; i < 5 ;i++)
  {
    testOne(ledFRYPin);
    testOne(ledFRRPin);
    testOne(ledFLYPin);
    testOne(ledFLRPin);
    
    testOne(ledBRYPin);
    testOne(ledBRRPin);
    testOne(ledBLYPin);
    testOne(ledBLRPin);
  }
}

void testOne(int led)
{
  digitalWrite(led, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  digitalWrite(ledPin, LOW);
  //delay(100);
}

// ===============================================================================
// nice to have a LED blinking when signal is captured OK. Good for debugging too.
void mLED_Red_On()
{
  digitalWrite(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void mLED_Red_Off()
{
  digitalWrite(ledPin, LOW);    // turn the LED off by making the voltage LOW
}

/*
// some debug related stuff:
void mLED_Dbg_On()
{
 digitalWrite(dbgPin, HIGH);   // turn the LED on (HIGH is the voltage level)
}
 
void mLED_Dbg_Off()
{
 digitalWrite(dbgPin, LOW);    // turn the LED off by making the voltage LOW
}
*/



void printValue() 
{
  Serial.print("!SNR:");
  Serial.print(rangeFRcm);  // front right
  Serial.print(",");
  Serial.print(rangeFLcm);  // front left
  Serial.print(",");
  Serial.print(rangeBLcm);  // back left
  Serial.print(",");
  Serial.print(rangeBRcm);  // back right
  /*     Serial.print("\t");
   Serial.print(psiData[4]);  // raw byte
   Serial.print("\t");
   Serial.print(psiData[5]);  // raw byte
   Serial.print("\t");
   Serial.print(psiData[6]);  // raw byte
   Serial.print("\t");
   Serial.print(psiData[7]);  // raw byte
   */
  Serial.print("\n");
  /*
   Serial.print(psiRawData[0]);
   Serial.print("***");
   Serial.print(psiRawData[1]);
   Serial.print("***");
   Serial.print(psiRawData[2]);
   Serial.print("***");
   Serial.print(psiRawData[3]);
   Serial.print("\n");
   */
}



