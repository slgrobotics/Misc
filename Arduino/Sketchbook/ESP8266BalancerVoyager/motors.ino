
// do not use deadzone on BLDC motors, may overheat
//#define DEADZONE 2

// limit power up to 255:
#define MAX_POWER 100

bool motorsStopped = true;

void initMotors()
{
  pinMode(rightPwmPin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);

  // make sure motors are stopped:
  analogWrite(rightPwmPin, 0);
  analogWrite(leftPwmPin, 0);

  shift.batchWriteBegin();
  shift.writeBit(stopBit, HIGH);     // HIGH - feathers wheels
  shift.writeBit(brakeBit, LOW);     // HIGH - applies moderate braking power
  shift.writeBit(rightDirBit, LOW);
  shift.writeBit(leftDirBit, LOW);
  shift.batchWriteEnd();
}

// ***********************************************************************
//   Set motor power for both motors. Positive is forward. Takes 0.12ms
// ***********************************************************************
void set_motors()
{
  if(motorsStopped)
  {
    shift.writeBit(stopBit, LOW);
  }

  float _pwm_R = constrain(pwm_R, -MAX_POWER, MAX_POWER);
  float _pwm_L = constrain(pwm_L, -MAX_POWER, MAX_POWER);
  
  // Set Right wheel's direction and speed
  if(_pwm_R == 0)
  {
     analogWrite(rightPwmPin, 0);
  }
  else if(_pwm_R > 0)
  {
#ifdef DEADZONE
     _pwm_R = map(_pwm_R, 1, 255, DEADZONE+1, 255);
#endif
    
     shift.writeBit(rightDirBit, HIGH);    // direction forward
     analogWrite(rightPwmPin, (int)_pwm_R);
  }
  else // _pwm_R < 0
  {
#ifdef DEADZONE
     _pwm_R = map(_pwm_R, -1, -255, -DEADZONE-1, -255);
#endif
    
     shift.writeBit(rightDirBit, LOW);     // direction backwards
     analogWrite(rightPwmPin, -(int)_pwm_R);
  }
  
  // Set Left wheel's direction and speed:
  if(_pwm_L == 0)
  {
     analogWrite(leftPwmPin, 0);
  }
  else if(_pwm_L > 0)
  {
#ifdef DEADZONE
     _pwm_L = map(_pwm_L, 1, 255, DEADZONE+1, 255);
#endif
    
     shift.writeBit(leftDirBit, LOW);     // direction forward
     analogWrite(leftPwmPin, (int)_pwm_L);
  }
  else // _pwm_L < 0
  {
#ifdef DEADZONE
     _pwm_L = map(_pwm_L, -1, -255, -DEADZONE-1, -255);
#endif
    
     shift.writeBit(leftDirBit, HIGH);    // direction backwards
     analogWrite(leftPwmPin, -(int)_pwm_L);
  }
}
