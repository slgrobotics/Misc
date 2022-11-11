
#define DEADZONE 20

// limit power up to 255:
#define MAX_POWER 255

void initMotors()
{
  pinMode(rightPwmPin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);

  // make sure motors are stopped:
  analogWrite(rightPwmPin, 0);
  analogWrite(leftPwmPin, 0);
}

float pwm_R_prev = 0.0;
float pwm_L_prev = 0.0;

// ***********************************************************************
//   Set motor power for both motors. Positive is forward. Takes 0.12ms
// ***********************************************************************
void set_motors()
{
  if(pwm_R_prev != pwm_R)
  {
    pwm_R_prev = pwm_R;

    float pwm = constrain(pwm_R, -MAX_POWER, MAX_POWER);
    
    // Set Right wheel's direction and speed
    if(abs(pwm) < 1.0/256.0)
    {
       analogWrite(rightPwmPin, 0);
    }
    else if(pwm > 0)
    {
      pwm = round(map(pwm, 1, 255, DEADZONE+1, 255));
      
      digitalWrite(rightDirPin, HIGH);    // direction forward
      analogWrite(rightPwmPin, (int)pwm);
    }
    else // pwm < 0
    {
      pwm = round(map(pwm, -1, -255, -DEADZONE-1, -255));
      
      digitalWrite(rightDirPin, LOW);     // direction backwards
      analogWrite(rightPwmPin, -(int)pwm);
    }
  }
  
  if(pwm_L_prev != pwm_L)
  {
    pwm_L_prev = pwm_L;

    float pwm = constrain(pwm_L, -MAX_POWER, MAX_POWER);
    
    // Set Left wheel's direction and speed:
    if(abs(pwm) < 1.0/256.0)
    {
       analogWrite(leftPwmPin, 0);
    }
    else if(pwm > 0)
    {
      pwm = round(map(pwm, 1, 255, DEADZONE+1, 255));
      
      digitalWrite(leftDirPin, LOW);     // direction forward
      analogWrite(leftPwmPin, (int)pwm);
    }
    else // pwm_L < 0
    {
      pwm = round(map(pwm_L, -1, -255, -DEADZONE-1, -255));
      
      digitalWrite(leftDirPin, HIGH);    // direction backwards
      analogWrite(leftPwmPin, -(int)pwm);
    }
  }
}
