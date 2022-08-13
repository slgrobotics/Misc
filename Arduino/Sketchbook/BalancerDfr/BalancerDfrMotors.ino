
#define DEADZONE 20

void initMotors()
{
  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

  analogWrite(E1, 0);    // make sure motors are stopped
  analogWrite(E2, 0);
}

// ***********************************************************************
//   Set motor power for both motors. Positive is forward. Takes 0.12ms
// ***********************************************************************
void set_motors()
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
     analogWrite(E1, 0);
  }
  else if(pwm_R > 0)
  {
//    if(pwm_R < DEADZONE)
//      pwm_R = DEADZONE;

    pwm_R = map(pwm_R, 1, 255, DEADZONE+1, 255);
    
     digitalWrite(M1, HIGH);    // direction forward
     analogWrite(E1, pwm_R);
  }
  else // pwm_R < 0
  {
//    if(pwm_R > -DEADZONE)
//      pwm_R = -DEADZONE;
    
    pwm_R = map(pwm_R, -1, -255, -DEADZONE-1, -255);
    
     digitalWrite(M1, LOW);     // direction backwards
     analogWrite(E1, -pwm_R);
  }
  
  // Set Left wheel's direction and speed:
  if(pwm_L == 0)
  {
     analogWrite(E2, 0);
  }
  else if(pwm_L > 0)
  {
//    if(pwm_L < DEADZONE)
//      pwm_L = DEADZONE;
    
    pwm_L = map(pwm_L, 1, 255, DEADZONE+1, 255);
    
     digitalWrite(M2, LOW);     // direction forward
     analogWrite(E2, pwm_L);
  }
  else // pwm_L < 0
  {
//    if(pwm_L > -DEADZONE)
//      pwm_L = -DEADZONE;
    
    pwm_L = map(pwm_L, -1, -255, -DEADZONE-1, -255);
    
     digitalWrite(M2, HIGH);    // direction backwards
     analogWrite(E2, -pwm_L);
  }
}
