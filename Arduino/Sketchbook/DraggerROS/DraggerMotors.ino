
// Left Wheel IBT-2 H-Bridge:
const int L_EN   = 5; // pins 3 & 4 connected on IBT-2
const int L_RPWM = 6; // Right side of the H-Bridge 
const int L_LPWM = 7; // Left side of the H-Bridge 

// Right Wheel IBT-2 H-Bridge:
const int R_EN   = 4;
const int R_RPWM = 44;
const int R_LPWM = 45;

#define DEADZONE 0
// if we are given pwm > FULLZONE, we just switch H-Bridge to full ON:
#define FULLZONE 250

void MotorsInit()
{
  // set mode for H-bridge pins, "PWM" ones must be PWM capable:

  // Note: PWM pins cannot be controlled by Fast library. They remain in PWM mode.

  //pinModeFast(DIS_AB,   OUTPUT);
  pinModeFast(L_EN,   OUTPUT);
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinModeFast(R_EN,   OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  setTimerForPWM();

  stopMotor(true);    // make sure motors are stopped
  stopMotor(false);
}

// ***********************************************************************
//   Set motor power for both motors. Positive is forward.
// ***********************************************************************
void set_motors()
{
  int pwr;

  int ipwm_R = -(int)pwm_R;
  int ipwm_L = -(int)pwm_L;

  ipwm_R = constrain(ipwm_R, -255, 255);     // Maximum / Minimum Limitations
  ipwm_L = constrain(ipwm_L, -255, 255);
  
  // Set Right wheel's direction and speed:
  if (ipwm_R == 0)
  {
    stopMotor(false);
  }
  else if (ipwm_R > 0)
  {
    digitalWrite(R_LPWM, LOW);
    pwr = toMotorPower(ipwm_R);
    if (pwr > FULLZONE)
    {
      // in Full Zone we just switch bridge side to ON:
      digitalWrite(R_RPWM, HIGH);
    } 
    else 
    {
      // Arduino generates 488Hz PWM propportional to analogWrite value:
      analogWrite(R_RPWM, pwr);
    }
  }
  else  // ipwm_R < 0
  {
    digitalWrite(R_RPWM, LOW);
    pwr = toMotorPower(-ipwm_R);
    if (pwr > FULLZONE)
    {
      // in Full Zone we just switch bridge side to ON:
      digitalWrite(R_LPWM, HIGH);
    } 
    else 
    {
      // Arduino generates 488Hz PWM propportional to analogWrite value:
      analogWrite(R_LPWM, pwr);
    }
  }
  
  // Set Left wheel's direction and speed:
  if (ipwm_L == 0)
  {
    stopMotor(true);
  }
  else if (ipwm_L > 0)
  {
    digitalWrite(L_LPWM, LOW);
    pwr = toMotorPower(ipwm_L);
    if (pwr > FULLZONE)
    {
      // in Full Zone we just switch bridge side to ON:
      digitalWrite(L_RPWM, HIGH);
    } 
    else 
    {
      // Arduino generates 488Hz PWM propportional to analogWrite value:
      analogWrite(L_RPWM, pwr);
    }
  }
  else  // ipwm_L < 0
  {
    digitalWrite(L_RPWM, LOW);
    pwr = toMotorPower(-ipwm_L);
    if (pwr > FULLZONE)
    {
      // in Full Zone we just switch bridge side to ON:
      digitalWrite(L_LPWM, HIGH);
    } 
    else 
    {
      // Arduino generates 488Hz PWM propportional to analogWrite value:
      analogWrite(L_LPWM, pwr);
    }
  }
  
  digitalWriteFast(L_EN, HIGH);
  digitalWriteFast(R_EN, HIGH);
}

int toMotorPower(int pwr)  // pwr expected to be 0...255, otherwise will be constrained
{
  int PWM = constrain(pwr, 0, 255);     // Maximum / Minimum Limitations for pwm

  if (DEADZONE > 0)
    PWM = map(PWM, 1, 255, DEADZONE + 1, 255);

  return PWM;
}

void enableMotors(bool enable)
{
  // EN High - enable both sides of the bridge:
  digitalWrite(L_EN, enable ? HIGH : LOW);
  digitalWrite(R_EN, enable ? HIGH : LOW);
}

void stopMotor(bool is_left)  // Motor will stop and brake.
{
  // Parking and brake: EN = 1, RPWM = 0, LPWM = 0
  if(is_left) {
    digitalWrite(L_RPWM, LOW);
    digitalWrite(L_LPWM, LOW);
  } else {
    digitalWrite(R_RPWM, LOW);
    digitalWrite(R_LPWM, LOW);
  }
  enableMotors(true);
}

void featherMotors()
{
  // Parking but not brake: EN = 0, RPWM = 1, LPWM = 1
  digitalWrite(L_RPWM, LOW);
  digitalWrite(L_LPWM, LOW);
  digitalWrite(R_RPWM, LOW);
  digitalWrite(R_LPWM, LOW);
  enableMotors(false);
}

void setTimerForPWM()
{
  // http://www.letsmakerobots.com/content/changing-pwm-frequencies-arduino-controllers
  // Note: Divide the PWM frequency by 2 for an 8MHz clock, multiply by 1.25 for a 20MHz clock.

  //For Arduino Uno, Nano, Micro Magician, Mini Driver, Lilly Pad and any other board using ATmega 8, 168 or 328

  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------

  //TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  //  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz



  //For Arduino Mega1280, Mega2560, MegaADK, Spider or any other board using ATmega1280 or ATmega2560

  //---------------------------------------------- Set PWM frequency for D4 & D13 ------------------------------

  //TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  //  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
  //TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


  //---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------

  //TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------

  //TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  //  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz


  //---------------------------------------------- Set PWM frequency for D2, D3 & D5 ---------------------------

  //TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR3B = TCCR3B & B11111000 | B00000010;    // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR3B = TCCR3B & B11111000 | B00000100;    // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz


  //---------------------------------------------- Set PWM frequency for D6, D7 & D8 ---------------------------

  //TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR4B = TCCR4B & B11111000 | B00000100;    // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz


  //---------------------------------------------- Set PWM frequency for D44, D45 & D46 ------------------------

  //TCCR5B = TCCR5B & B11111000 | B00000001;    // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
  TCCR5B = TCCR5B & B11111000 | B00000010;    // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
  //  TCCR5B = TCCR5B & B11111000 | B00000011;    // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz
  //TCCR5B = TCCR5B & B11111000 | B00000100;    // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR5B = TCCR5B & B11111000 | B00000101;    // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz

}
