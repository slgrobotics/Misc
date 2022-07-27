
#define DEADZONE 0

void initMotors()
{
  pinMode(rightPwmPin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);

  // make sure motors are stopped:
  analogWrite(rightPwmPin, 0);
  analogWrite(leftPwmPin, 0);

  shift.batchWriteBegin();
  shift.writeBit(stopBit, LOW);     // HIGH - feathers wheels
  shift.writeBit(brakeBit, LOW);    // HIGH - applies moderate braking power
  shift.writeBit(rightDirBit, LOW);
  shift.writeBit(leftDirBit, LOW);
  shift.batchWriteEnd();
}

// ***********************************************************************
//   Set motor power for both motors. Positive is forward. Takes 0.12ms
// ***********************************************************************
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
     analogWrite(rightPwmPin, 0);
  }
  else if(pwm_R > 0)
  {
//    if(pwm_R < DEADZONE)
//      pwm_R = DEADZONE;

     pwm_R = map(pwm_R, 1, 255, DEADZONE+1, 255);
    
     shift.writeBit(rightDirBit, HIGH);    // direction forward
     analogWrite(rightPwmPin, pwm_R);
  }
  else // pwm_R < 0
  {
//    if(pwm_R > -DEADZONE)
//      pwm_R = -DEADZONE;
    
     pwm_R = map(pwm_R, -1, -255, -DEADZONE-1, -255);
    
     shift.writeBit(rightDirBit, LOW);     // direction backwards
     analogWrite(rightPwmPin, -pwm_R);
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
    
     pwm_L = map(pwm_L, 1, 255, DEADZONE+1, 255);
    
     shift.writeBit(leftDirBit, LOW);     // direction forward
     analogWrite(leftPwmPin, pwm_L);
  }
  else // pwm_L < 0
  {
//    if(pwm_L > -DEADZONE)
//      pwm_L = -DEADZONE;
    
     pwm_L = map(pwm_L, -1, -255, -DEADZONE-1, -255);
    
     shift.writeBit(leftDirBit, HIGH);    // direction backwards
     analogWrite(leftPwmPin, -pwm_L);
  }
}
