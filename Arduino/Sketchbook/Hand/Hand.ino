
//#define SERIAL_ROBOREALM
#define SERIAL_MY

#include <Wire.h>
#include <Servo.h>

const int palmSensorPin = A3;    // Analog input pin that the opto pair sensor is attached to (100K in collector)
//const int potPin = A2;           // Analog input pin connected to 10K POT - not any more
const int servoCurrentPin = A1;  // Analog input pin attached to 1 Ohm resistor between thumb servo ground wire and ground
const int led = 13;

// A4  - White,  A5 - Red,  Ground - Black  - an RC cable for I2C

enum GrabState { GRAB_NONE, GRAB_RELEASE, GRAB_FULL };

Servo servoThumb;  // create servo object to control thumb
Servo servoIndex;
Servo servoMiddle;
Servo servoPinky;
Servo servoWrist;

int cnt = 0;
int palmSensorThreshold = 80;
long timeToHoldGrabMs = 2000;

int thumbAngle = 160; // released
int indexAngle = 150;
int middleAngle = 70;
int pinkyAngle = 100;
int wristAngle = 90;

int palmSensorValue;
int servoCurrentValue;

bool isPalmSensorActivated = false;

void setup()
{
  Serial.begin(115200);

  InitializeSerial();

  InitializeI2c();

  pinMode(led, OUTPUT);

  servoThumb.attach(4);
  servoIndex.attach(6);
  servoMiddle.attach(7);
  servoPinky.attach(2);
  servoWrist.attach(8);

  ResetGrab();
}

void loop()
{
  palmSensorValue = analogRead(palmSensorPin);      // no direct light - around 260; a Coke can pretty close - < 200, down to 50
  servoCurrentValue = analogRead(servoCurrentPin);  // idle - 0; 0.3A

  isPalmSensorActivated = palmSensorValue < palmSensorThreshold;
  digitalWrite(led, isPalmSensorActivated ?  HIGH : LOW);

  //int potValue = 511; //analogRead(potPin);           // 0 to 1023
  //wristAngle = map(potValue, 0, 1023, 0, 180);        // scale pot value to use it with the wrist servo (angle between 0 and 180)

  while(CheckSerial())
  {
    ;  // false if no input, true if there is something to read
  }

  if ((cnt % 10) == 0)
  {
    Serial.print(palmSensorValue); Serial.print("  ");
    Serial.print(wristAngle); Serial.print("  ");
    Serial.println(servoCurrentValue);
  }

  if(!CheckGrab())
  {
    // we are not grabbing anything, servos can be controlled by remote.
    RemoteControl();
  }
  
  cnt++;
  delay(20);
}

void RemoteControl()
{
  servoThumb.write(thumbAngle);
  //servoThumb.write(140);
  servoIndex.write(indexAngle);
  servoMiddle.write(middleAngle);
  servoPinky.write(pinkyAngle);
  servoWrist.write(wristAngle);
}

