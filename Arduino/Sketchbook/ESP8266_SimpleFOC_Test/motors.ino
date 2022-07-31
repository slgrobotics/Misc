
#define DEADZONE 0

// limit power up to 255:
#define MAX_POWER 20

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
  shift.writeBit(brakeBit, LOW);    // HIGH - applies moderate braking power
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
  
  if(pwm_R > MAX_POWER)      // Maximum / Minimum Limitations
    pwm_R = MAX_POWER;
    
  if(pwm_R < -MAX_POWER)
    pwm_R = -MAX_POWER;

  if(pwm_L > MAX_POWER)
    pwm_L = MAX_POWER;
    
  if(pwm_L < -MAX_POWER)
    pwm_L = -MAX_POWER;
  
  // Set Right wheel's direction and speed
  if(pwm_R == 0)
  {
     analogWrite(rightPwmPin, 0);
  }
  else if(pwm_R > 0)
  {
//    if(pwm_R < DEADZONE)
//      pwm_R = DEADZONE;

     int pwm = (int)map(pwm_R, 1, 255, DEADZONE+1, 255);
    
     shift.writeBit(rightDirBit, HIGH);    // direction forward
     analogWrite(rightPwmPin, pwm);
  }
  else // pwm_R < 0
  {
//    if(pwm_R > -DEADZONE)
//      pwm_R = -DEADZONE;
    
     int pwm = (int)map(pwm_R, -1, -255, -DEADZONE-1, -255);
    
     shift.writeBit(rightDirBit, LOW);     // direction backwards
     analogWrite(rightPwmPin, -pwm);
  }
  
  // Set Left wheel's direction and speed:
  if(pwm_L == 0)
  {
     analogWrite(leftPwmPin, 0);
  }
  else if(pwm_L > 0)
  {
//    if(pwm_L < DEADZONE)
//      pwm_L = DEADZONE;
    
     int pwm = (int)map(pwm_L, 1, 255, DEADZONE+1, 255);
    
     shift.writeBit(leftDirBit, LOW);     // direction forward
     analogWrite(leftPwmPin, pwm);
  }
  else // pwm_L < 0
  {
//    if(pwm_L > -DEADZONE)
//      pwm_L = -DEADZONE;
    
     int pwm = (int)map(pwm_L, -1, -255, -DEADZONE-1, -255);
    
     shift.writeBit(leftDirBit, HIGH);    // direction backwards
     analogWrite(leftPwmPin, -pwm);
  }
}
