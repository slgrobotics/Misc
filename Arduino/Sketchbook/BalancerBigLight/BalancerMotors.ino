
// PWM pins must be one of 3, 5, 6, 9, 10, or 11 for Arduino Uno
// For pins 3,9,10,11 it is approximately 488 Hz. For pins 5 and 6, it is about 977 Hz

// Side A of the Bridge is Left, side B - Right

/* see C:\Projects\Arduino\arduino-1.0.2\libraries\AlfsTechHbridge\AlfsTechHbridge.cpp
// *** Declarations - Arduino UNO Atmel 328PU (28-pin DIP) ***
//     Arduino Data Pin                Atmel 328PU pin 

int DIS_AB = 12;  			// 18		// HIGH on DIS disables both sides. 

// H-Bridge Side A (left):
int EN_A   = 7;				// 13
int RPWM_A = 5;				// 11
int LPWM_A = 9;				// 15

// H-Bridge Side B (right):
int EN_B   = 4;				// 6
int RPWM_B = 6;				// 12
int LPWM_B = 11;			// 17
*/

#define DEADZONE 20

// ******************************************************************************************
//
//   Set motor power for both motors. Positive is forward.
//
// ******************************************************************************************
void set_motor()
{
  if(pwm_R > 255)      // Maximum / Minimum Limitations
    pwm_R = 255;
    
  if(pwm_R < -255)
    pwm_R = -255;

  if(pwm_L > 255)
    pwm_L = 255;
    
  if(pwm_L < -255)
    pwm_L = -255;
  
  // Set Right wheel's direction and speed
  if(pwm_R == 0)
  {
    motors.stopRight();		// "Right" side of the bridge (Side B)
  }
  else if(pwm_R > 0)
  {
    pwm_R = map(pwm_R, 1, 255, DEADZONE+1, 255);
    motors.setRightSpeed(pwm_R);     // direction forward
  }
  else // pwm_R < 0
  {
    pwm_R = map(pwm_R, -1, -255, -DEADZONE-1, -255);
    motors.setRightSpeed(pwm_R);     // direction backwards
  }
  
  // Set Left wheel's direction and speed:
  if(pwm_L == 0)
  {
    motors.stopLeft();		// "Left" side of the bridge (Side A)
  }
  else if(pwm_L > 0)
  {
    pwm_L = map(pwm_L, 1, 255, DEADZONE+1, 255);
    motors.setLeftSpeed(pwm_L);     // direction forward
  }
  else // pwm_L < 0
  {
    pwm_L = map(pwm_L, -1, -255, -DEADZONE-1, -255);
    motors.setLeftSpeed(pwm_L);    // direction backwards
  }
}


