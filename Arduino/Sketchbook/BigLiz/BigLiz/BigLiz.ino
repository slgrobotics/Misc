
/*****************************************************************************************************************
 *
 * Big Liz robot has a rotating head (camera is mounted there) and a bubble blower ("gun").
 * For debugging and extra fun there is a two-servo turret (pan and tilt) with a laser pointer mounted on it. 
 * Everything is controlled over serial-over-USB by lines containing channel and value.
 * Sensor values can be reported back to PC in the same manner.
 * 
 * May 2013 - Meetup group "http://www.meetup.com/Make-OC/events/110310252/?_af_eid=110310252&a=uc1_vm&_af=event"
 *
 *****************************************************************************************************************/
#include <PID_v1.h>
#include <Servo.h>

Servo gunPanServo;
Servo gunTiltServo;

//#define USE_SERVO_FOR_CAMERA

#ifdef USE_SERVO_FOR_CAMERA
Servo cameraPanServo;
#endif  // USE_SERVO_FOR_CAMERA

// for PID see https://github.com/br3ttb/Arduino-PID-Library  and  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ 

// Define Variables used by PID:
double HeadPanSetpoint, HeadPanMeasured, HeadPanMotorPower;

// Specify the links and initial tuning parameters:
PID myPID(&HeadPanMeasured, &HeadPanMotorPower, &HeadPanSetpoint, 1.5, 3.0, 0.01, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE - for Servo-like  800..2200 "us" scale

// Head pan (turret) "servo" 5KOhm feedbak potentiometer:
const int analogInPin = A3;  // Analog input pin that the potentiometer is attached to

// standard LED
const int led = 13;

// laser pointer for alignment:
const int laser1 = 8;

// connect arduino pins 2...5 to motor shield in1...in4 (Direction)  and pin 9,10 to EnA,EnB (PWM)
// PWM pins must be one of 3, 5, 6, 9, 10, or 11 for Arduino Uno
// For pins 3,9,10,11 it is approximately 488 Hz. For pins 5 and 6, it is about 977 Hz
const int EnA = 5;  
const int EnB = 6;  
const int in1 = 2;
const int in2 = 3;                        
const int in3 = 4;                          
const int in4 = 7;                          

const int HeadPanMotor   = 1;                          
const int BubbleFanMotor = 2;                          

int mappedFeedback;
unsigned long PrintTimeIntervalMs = 100;
unsigned long lastPrintTime;

int gunPowerMks;

void setup()
{
  pinMode(led, OUTPUT);     
  pinMode(laser1, OUTPUT);
  
  // to make things simpler, we operate all actuators using Servo-like microsecond scale, 800 to 2200
  
  gunPanServo.attach(12);     // attaches the servo on pin 12 to the servo object 
  gunTiltServo.attach(10);

#ifdef USE_SERVO_FOR_CAMERA
  cameraPanServo.attach(11);
#endif  // USE_SERVO_FOR_CAMERA
  
  // camera pan is under PID control:
  //HeadPanSetpoint = 0;     // 500 and 300 scales
  HeadPanSetpoint = 1500;    // Servo-like  800..2200 "us" scale
  
  gunPowerMks = 800;         // Servo-like  800 - off

  Serial.begin(57600);

  //turn the PID on and set parameters:
  myPID.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID
  myPID.SetOutputLimits(-255.0, 255.0);  // match to maximum PID outputs in both directions
  myPID.SetSampleTime(10);               // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval.
  
  enableMotors();
}

void loop()
{
  /*
  int value;

  for(value = -255 ; value <= 255; value+=1)
  {
    SetMotorPower(1, value);
    SetMotorPower(2, value);
    delay(50);
  } 

  SetMotorPower(1, 0);
  SetMotorPower(2, 0);
  delay(3000);
  */
  
  // if there's any serial available, read it:
  if (Serial.available() > 0)
  {
    // look for the asterisk *. That's the beginning of your sentence:
    if (Serial.read() != '*')
    {
      return;
    }

    // look for the next four valid integers in the incoming serial stream:
    int channel = Serial.parseInt(); 
    int cmd = Serial.parseInt(); 
    int cmdValue = Serial.parseInt(); 
    int checksum = Serial.parseInt(); 

    if(channel + cmd + cmdValue + checksum != 0)
    {
      // bad transmission
      digitalWrite(led, 1);    // indicates trouble
      return;
    }
     
    digitalWrite(led, 0);  // indicates normal operation
    
    switch (channel)
    {
      case 1:  // pan 
        HeadPanSetpoint = cmdValue;
        break;
      case 2:  // bubble blower fan power (a.k.a. gun power)
        gunPowerMks = cmdValue;
        break;
      case 3:  // gun pan
        gunPanServo.writeMicroseconds(cmdValue);
        break;
      case 4:  // gun tilt
        gunTiltServo.writeMicroseconds(cmdValue);
        break;
      case 5:  // calibration laser
        digitalWrite(laser1, cmdValue > 0);   // turn the laser1 on or off
        break;
    }
  }

  headPanRotation();
  setGunPower();

  // print variables for debugging and communication to PC:
  unsigned long now = millis();
  unsigned long timeSinceLastPrint = (now - lastPrintTime);
  if(timeSinceLastPrint >= PrintTimeIntervalMs)
  {
    lastPrintTime = now;
    Serial.print("*");
    Serial.print(HeadPanSetpoint);
    Serial.print(" ");
    Serial.print(mappedFeedback);
    Serial.print(" ");
    Serial.println(HeadPanMotorPower);
    
    digitalWrite(laser1, HIGH);   // turn the laser1 on
  }
  
  delay(5);
}

const int deadzone = 3;

/// using PID rotates head to position defined by "HeadPanSetpoint"
void headPanRotation()
{
#ifdef USE_SERVO_FOR_CAMERA
  cameraPanServo.writeMicroseconds(HeadPanSetpoint);
#else  // do not USE_SERVO_FOR_CAMERA - use motor with 5K feedback pot
  // read the feedback pot analog value:
  int feedbackPotValue = analogRead(analogInPin);   // value read from the pot: 710 head turned all way to the left; 295 - to the right; 504 - middle     

  //Serial.println(feedbackPotValue);

  // actual rotation range is: Left - 50 degrees, Right - 52 degrees.
  mappedFeedback = map(feedbackPotValue, 710, 295, 800, 2200);    // map(value, fromLow, fromHigh, toLow, toHigh)          // Servo-like  800..2200 "us" scale

  //HeadPanMeasured = abs(HeadPanSetpoint - mappedFeedback) < deadzone ? HeadPanSetpoint : mappedFeedback;
  HeadPanMeasured = mappedFeedback;

  myPID.Compute();
  SetMotorPower(1, HeadPanMotorPower);
#endif  // USE_SERVO_FOR_CAMERA
}

void setGunPower()
{
  int mappedGunPower = map(gunPowerMks, 800, 2200, 0, 210);    // map(value, fromLow, fromHigh, toLow, toHigh)          // Servo-like  800..2200 "us" scale
  SetMotorPower(2, mappedGunPower);
}

// ========================= region Motors ==============================

// there are two motors, connected via a common L298N Dual H Bridge Module (see eBay) 
// one motor controls Head rotation via PID and feedback closed loop.
// another motor is in bubble blower - it is supposed to receive 9V from six AA batteries, but here is receiving 12V PWM at about 210 (of 255).

void enableMotors()
{
    pinMode(EnA, OUTPUT);  
    pinMode(EnB, OUTPUT);
    
    pinMode(in1, OUTPUT);  
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);  
    pinMode(in4, OUTPUT);
}

const int minPower = 150;
const int minPowerOff = 50;

// motor - 1 (A) or 2 (b)
// power - -255...+255
void SetMotorPower(int motor, int power)
{
  power = constrain(power, -255, 255);
  
  if(power != 0)
  {
    if(abs(power) < minPowerOff)
    {
      power = 0;
    }
    else if(abs(power) < minPower)
    {
      power = power > 0 ? minPower : -minPower;
    }
  }
  
  int side1, side2, enable;  // sides of H-bridge (direction) and bridge enable pins for selected motor

  switch (motor)
  {
    case HeadPanMotor:
      side1 = in1;
      side2 = in2;
      enable = EnA;
      break;

    case BubbleFanMotor:
      side1 = in3;
      side2 = in4;
      enable = EnB;
      break;

    default:
      return;
  }
  
  if(power == 0)
  {
    digitalWrite(side1, HIGH);    // both HIGH disables the sides
    digitalWrite(side2, HIGH);  
    digitalWrite(enable, LOW);    // LOW disables the bridge
  }
  else if(power > 0)
  {
    digitalWrite(side1, HIGH);  
    digitalWrite(side2, LOW);  
    analogWrite(enable, power);    // PWM
  }
  else
  {
    digitalWrite(side1, LOW);  
    digitalWrite(side2, HIGH);  
    analogWrite(enable, -power);   // PWM
  }
}
// ========================= endregion Motors ==============================

