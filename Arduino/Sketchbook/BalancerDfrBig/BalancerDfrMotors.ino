
#define DEADZONE 0

// ***********************************************************************
//   Set motor power for both motors. Positive is forward. Takes 0.12ms
// ***********************************************************************
void set_motor_power()
{
  int pwr; 

  pwm_R = constrain(pwm_R, -255, 255);     // Maximum / Minimum Limitations
  pwm_L = constrain(pwm_L, -255, 255);
  
  // Set Right wheel's direction and speed
  if(pwm_R == 0)
  {
    digitalWrite(RPWM_B, HIGH);
    digitalWrite(LPWM_B, HIGH);
  }
  else if(pwm_R > 0)
  {
    if(DEADZONE > 0)
      pwm_R = map(pwm_R, 1, 255, DEADZONE+1, 255);
    
    pwr = toMotorPower(pwm_R);
    // Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant
    // Arduino generates 488Hz PWM propportional to analogWrite value:
    analogWrite( LPWM_B, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
    digitalWrite(RPWM_B, HIGH);
  }
  else // pwm_R < 0
  {
    if(DEADZONE > 0)
      pwm_R = map(pwm_R, -1, -255, -DEADZONE-1, -255);
    
    pwr = toMotorPower(-pwm_R);
    // Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant
    analogWrite( RPWM_B, (byte)(255 - pwr));
    digitalWrite(LPWM_B, HIGH);
  }
  
  // Set Left wheel's direction and speed:
  if(pwm_L == 0)
  {
    digitalWrite(RPWM_A, HIGH);
    digitalWrite(LPWM_A, HIGH);
  }
  else if(pwm_L > 0)
  {
    if(DEADZONE > 0)
      pwm_L = map(pwm_L, 1, 255, DEADZONE+1, 255);
    
    pwr = toMotorPower(pwm_L);
    // Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant
    // Arduino generates 488Hz PWM propportional to analogWrite value:
    analogWrite( RPWM_A, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
    digitalWrite(LPWM_A, HIGH);
  }
  else // pwm_L < 0
  {
    if(DEADZONE > 0)
      pwm_L = map(pwm_L, -1, -255, -DEADZONE-1, -255);
    
    pwr = toMotorPower(-pwm_L);
    // Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant
    analogWrite( LPWM_A, (byte)(255 - pwr));
    digitalWrite(RPWM_A, HIGH);
  }
  
  digitalWriteFast(EN_A, HIGH);
  digitalWriteFast(EN_B, HIGH);
}

int toMotorPower(int pwr)  // pwr expected to be 0...255, otherwise will be constrained to 0...240
{
  //pwr = 255 - pwr;
  pwr = constrain(pwr, 0, 240);  // PWM duty cycle cannot be less than 98% per DBH-1B H-bridge specs
  return pwr;
}

void StopMotors()  // Both Motors will stop and brake.
{
  forward_factor = 0.0;
  turn_flag = 0.0;

  // Parking but not brake: EN = 0, RPWM = 1, LPWM = 1, DIS = vacant
  // Parking and brake: EN = 1, RPWM = 1, LPWM = 1, DIS = vacant
  digitalWrite(RPWM_A, HIGH);
  digitalWrite(LPWM_A, HIGH);
  digitalWriteFast(EN_A, HIGH);
  digitalWrite(RPWM_B, HIGH);
  digitalWrite(LPWM_B, HIGH);
  digitalWriteFast(EN_B, HIGH);
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
