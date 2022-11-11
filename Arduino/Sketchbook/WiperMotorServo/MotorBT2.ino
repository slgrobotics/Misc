/*
   to provide easy interface for BT7960 based BT_2 H-Bridge controller

   see http://www.hessmer.org/blog/2013/12/28/ibt-2-h-bridge-with-arduino/
*/

// note that pins 3-5 are for MLX90316 rotation sensor, so we use PWM pins 9,10 here
#define RPWM 9
#define LPWM 10
#define L_EN 7
#define R_EN 8

//#define DEADZONE 30   // defined in main file for clarity
// if we are given pwm > FULLZONE, we just switch H-Bridge to full ON:
#define FULLZONE 250

void setupMotor()
{
  setPWMfrequency(0x02);  // timer 1 for pins 9,10, set to 3.92KHz

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
}

void setPWMfrequency(int freq) {
  TCCR1B = TCCR1B & 0b11111000 | freq ;
}

// ***********************************************************************
//   Set motor power for both motors.
//   pwr -255...255, will be constrained. Takes 0.12ms
//   returns actual PWM sent to bridge, with direction sign, -255...255 for tracing
// ***********************************************************************
int setMotorPower(int pwr)
{
  if (pwr == 0)
  {
    brakeMotor();
  }
  else if (pwr > 0)
  {
    digitalWrite(LPWM, LOW);
    pwr = toMotorPower(pwr);
    if (pwr > FULLZONE)
    {
      // in Full Zone we just switch bridge side to ON:
      digitalWrite(RPWM, HIGH);
    } 
    else 
    {
      analogWrite(RPWM, pwr);
    }
  }
  else  // pwr < 0
  {
    digitalWrite(RPWM, LOW);
    pwr = toMotorPower(-pwr);
    if (pwr > FULLZONE)
    {
      // in Full Zone we just switch bridge side to ON:
      digitalWrite(LPWM, HIGH);
    } 
    else 
    {
      analogWrite(LPWM, pwr);
    }
    pwr = -pwr;
  }
  return pwr;
}

int toMotorPower(int pwr)  // pwr expected to be 0...255, otherwise will be constrained to 0...240
{
  int PWM = constrain(pwr, 0, 255);     // Maximum / Minimum Limitations for pwm

  if (DEADZONE > 0)
    PWM = map(PWM, 1, 255, DEADZONE + 1, 255);

  return PWM;
}

void enableMotor()
{
  // EN High - enable both sides of the bridge:
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);
}

void brakeMotor()
{
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}

void featherMotor()
{
  // EN LOW - disable both sides of the bridge:
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  // disable PWM pins
  digitalWrite(RPWM, LOW);
  digitalWrite(LPWM, LOW);
}
