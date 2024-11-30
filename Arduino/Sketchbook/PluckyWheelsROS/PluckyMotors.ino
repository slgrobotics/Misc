
// Arduino Mega 2560 PWM pins: 2 to 13 and 44 to 46
// see https://www.arduino.cc/en/Main/ArduinoBoardMega2560    https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
const int LDIR = 42;
const int LPWM = 44;
const int RDIR = 43;
const int RPWM = 45;

void MotorsInit()
{
  pinMode(RDIR, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(LDIR, OUTPUT);
  pinMode(LPWM, OUTPUT);

  pwm_R = 0;
  pwm_L = 0;

  set_motors();
}

void constrainPwm()
{
  // Maximum / Minimum Limitations:
  pwm_R = constrain(pwm_R, -255, 255);
  pwm_L = constrain(pwm_L, -255, 255);
}


#define DEADZONE 2

// ******************************************************************************************
//
//   Set motor power for both motors. Positive is forward.
//
// ******************************************************************************************
void set_motors()
{
  constrainPwm();

  // for most H-Bridges you have to keep PWM % below 100%, so that's why 250 is used below.

  // Set Right wheel's direction and speed
  if (pwm_R == 0)
  {
    digitalWriteFast(RDIR, LOW);
    analogWrite(RPWM, 0);
  }
  else if (pwm_R > 0)
  {
    int myPwm_R = map(pwm_R, 1, 255, DEADZONE + 1, 250);
    digitalWriteFast(RDIR, LOW);     // direction forward
    analogWrite(RPWM, myPwm_R);
  }
  else // pwm_R < 0
  {
    int myPwm_R = map(pwm_R, -1, -255, DEADZONE + 1, 250);
    digitalWriteFast(RDIR, HIGH);     // direction backwards
    analogWrite(RPWM, myPwm_R);
  }

  // Set Left wheel's direction and speed
  if (pwm_L == 0)
  {
    digitalWriteFast(LDIR, LOW);
    analogWrite(LPWM, 0);
  }
  else if (pwm_L > 0)
  {
    int myPwm_L = map(pwm_L, 1, 255, DEADZONE + 1, 250);
    digitalWriteFast(LDIR, HIGH);     // direction forward
    analogWrite(LPWM, myPwm_L);
  }
  else // pwm_L < 0
  {
    int myPwm_L = map(pwm_L, -1, -255, DEADZONE + 1, 250);
    digitalWriteFast(LDIR, LOW);     // direction backwards
    analogWrite(LPWM, myPwm_L);
  }
}
