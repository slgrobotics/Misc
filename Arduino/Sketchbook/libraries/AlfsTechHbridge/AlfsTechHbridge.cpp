#ifndef F_CPU
#define F_CPU 16000000UL  // Standard Arduinos run at 16 MHz
#endif //!F_CPU

#include "AlfsTechHbridge.h"

// Notes about ALFS_TECH 50A motor H-Bridge
// VCC is connected to +Power and can deliver Vin to Arduino
//		EN, RPWM, LPWM are pulled up to internal 3.3V
//		10K to +3V on "DIS" stops (disables) the side.

// PWM pins must be one of 3, 5, 6, 9, 10, or 11 for Arduino Uno
// For pins 3,9,10,11 it is approximately 488 Hz. For pins 5 and 6, it is about 977 Hz

// Side A of the Bridge is Left, side B - Right

//  AlfsTechHbridge electrical interface

// *** Declarations - Arduino UNO Atmel 328PU (28-pin DIP) ***
// Arduino Data Pin      Atmel 328PU pin 

int DIS_AB = 12;  			// 18		// HIGH on DIS disables both sides. 

// H-Bridge Side A:
int EN_A   = 7;				// 13
int RPWM_A = 5;				// 11
int LPWM_A = 9;				// 15
boolean reverse_A = false;

// H-Bridge Side B:
int EN_B   = 4;				// 6
int RPWM_B = 6;				// 12
int LPWM_B = 11;			// 17
boolean reverse_B = true;

#define ATHB_MOTORS_ENABLE 			LOW
#define ATHB_MOTORS_DISABLE			HIGH


// Constructor
AlfsTechHbridge::AlfsTechHbridge(void)
{
  digitalWrite(DIS_AB, HIGH);  // disable for now; must call motorEnableAB() later
  pinMode(DIS_AB,   OUTPUT);
  
  pinMode(EN_A,   OUTPUT);
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(EN_B,   OUTPUT);
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  
  // Encoders:
  ENCODER_A_B   = 2;		// 4	  // side B (interrupt 0)
  ENCODER_B_B   = 8;		// 14	  // side B

  ENCODER_A_A   = 3;		// 5	  // side A (interrupt 1)
  ENCODER_B_A   = 10;		// 16	  // side A

  pinMode(ENCODER_A_A, INPUT); 
  pinMode(ENCODER_B_A, INPUT); 
  digitalWrite(ENCODER_A_A, HIGH);       // turn on pullup resistors
  digitalWrite(ENCODER_B_A, HIGH);

  pinMode(ENCODER_A_B, INPUT); 
  pinMode(ENCODER_B_B, INPUT); 
  digitalWrite(ENCODER_A_B, HIGH);       // turn on pullup resistors
  digitalWrite(ENCODER_B_B, HIGH);
  
  flipLeft = false;
  flipRight = false;
}

// Destructor
AlfsTechHbridge::~AlfsTechHbridge(void)
{
  motorsDisable();
}

void AlfsTechHbridge::init(void)
{
  motorsEnable();
}

// ======================== region AlfsTech H-Bridge bus operations =====================================

// enable/disable flipping of left motor
void AlfsTechHbridge::flipLeftMotor(boolean flip)
{
  flipLeft = flip;
}

// enable/disable flipping of right motor
void AlfsTechHbridge::flipRightMotor(boolean flip)
{
  flipRight = flip;
}

void AlfsTechHbridge::motorsDisable()
{
  digitalWrite(DIS_AB, HIGH);
}

void AlfsTechHbridge::motorsEnable()
{
  digitalWrite(DIS_AB, LOW);
}

void AlfsTechHbridge::motorsStop()
{
  stopLeft();
  stopRight();
}

void AlfsTechHbridge::stopLeft()
{
  // Parking but not brake: EN = 0, RPWM = 1, LPWM = 1, DIS = vacant
  // Parking and brake: EN = 1, RPWM = 1, LPWM = 1, DIS = vacant
  digitalWrite(RPWM_A, HIGH);
  digitalWrite(LPWM_A, HIGH);
  digitalWrite(EN_A, HIGH);
}

void AlfsTechHbridge::stopRight()
{
  // Parking but not brake: EN = 0, RPWM = 1, LPWM = 1, DIS = vacant
  // Parking and brake: EN = 1, RPWM = 1, LPWM = 1, DIS = vacant
  digitalWrite(RPWM_B, HIGH);
  digitalWrite(LPWM_B, HIGH);
  digitalWrite(EN_B, HIGH);
}

// set speed for both motors  -255...0...+255 - will be cut to this range
void AlfsTechHbridge::setSpeeds(int leftSpeed, int rightSpeed)
{
  setLeftSpeed(leftSpeed);
  setRightSpeed(rightSpeed);
}

void AlfsTechHbridge::setLeftSpeed(int power)    // power -255...0...+255 - will be cut to this range
{
  if(power == 0)
  {
    stopLeft();
    return;
  }
  
  if(reverse_A)
  {
    power = -power;
  }
  
  if(flipLeft)
  {
    power = -power;
  }
  
  if(power < -255)
  {
    power = -255;
  }
  
  if(power > 255)
  {
    power = 255;
  }
  
  if(power > 0)
  {
    // Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant
    // Arduino generates 488Hz PWM propportional to analogWrite value:
    analogWrite( RPWM_A, (byte)(255 - power));  // 0 is max speed; 255 - min speed
    digitalWrite(LPWM_A, HIGH);
  }
  else
  {
    // Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant
    analogWrite( LPWM_A, (byte)(255 + power));
    digitalWrite(RPWM_A, HIGH);
  }
  digitalWrite(EN_A, HIGH);
}

void AlfsTechHbridge::setRightSpeed(int power)    // power -255...0...+255 - will be cut to this range
{
  if(power == 0)
  {
    stopRight();
    return;
  }
  
  if(reverse_B)
  {
    power = -power;
  }
  
  if(flipRight)
  {
    power = -power;
  }
  
  if(power < -255)
  {
    power = -255;
  }
  
  if(power > 255)
  {
    power = 255;
  }
  
  if(power > 0)
  {
    // Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant
    // Arduino generates 488Hz PWM propportional to analogWrite value:
    analogWrite( RPWM_B, (byte)(255 - power));  // 0 is max speed; 255 - min speed
    digitalWrite(LPWM_B, HIGH);
  }
  else
  {
    // Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant
    analogWrite( LPWM_B, (byte)(255 + power));
    digitalWrite(RPWM_B, HIGH);
  }
  digitalWrite(EN_B, HIGH);
}

