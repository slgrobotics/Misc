
// This code is for L298n motor H-Bridge, common motor driver board.

// H-Bridge Side A, Left Wheel:
const int EN_A   = 5;
const int RPWM_A = 4;
const int LPWM_A = 3;
//const boolean reverse_A = false;

// H-Bridge Side B, Right wheel:
const int EN_B   = 0;
const int RPWM_B = 1;
const int LPWM_B = 2;
//const boolean reverse_B = true;

#define DEADZONE 0

void MotorsInit()
{
  // set mode for H-bridge pins, "PWM" ones must be PWM capable:
  
  // Note: PWM pins cannot be controlled by Fast library. They remain in PWM mode.
  
  //pinMode(DIS_AB,   OUTPUT);
  pinMode(EN_A,   OUTPUT);
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(EN_B,   OUTPUT);
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);

  //digitalWrite(DIS_AB, HIGH);  // disable for now;
  
  setTimerForPWM();

  StopMotors();    // make sure motors are stopped
}

// ***********************************************************************
//   Set motor power for both motors. Positive is forward. Takes 0.12ms
// ***********************************************************************
void set_motors()
{
  int pwr; 

  // to reverse, put minus here:
  int ipwm_R = -(int)pwm_R;
  int ipwm_L = -(int)pwm_L;

  ipwm_R = constrain(ipwm_R, -255, 255);     // Maximum / Minimum Limitations
  ipwm_L = constrain(ipwm_L, -255, 255);
  
  // Set Right wheel's direction and speed
  if(ipwm_R == 0)
  {
    // park:
    digitalWrite(EN_B, LOW);
    digitalWrite(RPWM_B, LOW);
    digitalWrite(LPWM_B, LOW);
  }
  else if(ipwm_R > 0)
  {
    if(DEADZONE > 0)
      ipwm_R = map(ipwm_R, 1, 255, DEADZONE+1, 255);
    
    pwr = toMotorPower(ipwm_R);
    // Rotate forward: EN = PWM, RPWM = 1, LPWM = 0
    // Pico generates 500Hz PWM proportional to analogWrite value:
    analogWrite( EN_B, (byte)pwr);  // 0 is min speed; 255 - max speed
    digitalWrite(RPWM_B, HIGH);
    digitalWrite(LPWM_B, LOW);
  }
  else // ipwm_R < 0
  {
    if(DEADZONE > 0)
      ipwm_R = map(ipwm_R, -1, -255, -DEADZONE-1, -255);
    
    pwr = toMotorPower(-ipwm_R);
    // Rotate reverse: EN = PWM, RPWM = 0, LPWM = 1
    analogWrite( EN_B, (byte)pwr);
    digitalWrite(LPWM_B, HIGH);
    digitalWrite(RPWM_B, LOW);
  }
  
  // Set Left wheel's direction and speed:
  if(ipwm_L == 0)
  {
    // park:
    digitalWrite(EN_A, LOW);
    digitalWrite(RPWM_A, LOW);
    digitalWrite(LPWM_A, LOW);
  }
  else if(ipwm_L > 0)
  {
    if(DEADZONE > 0)
      ipwm_L = map(ipwm_L, 1, 255, DEADZONE+1, 255);
    
    pwr = toMotorPower(ipwm_L);
    // Rotate forward: EN = PWM, RPWM = 0, LPWM = 1
    // Pico generates 500Hz PWM proportional to analogWrite value:
    analogWrite(EN_A, (byte)pwr);  // 0 is min speed; 255 - max speed
    digitalWrite(LPWM_A, HIGH);
    digitalWrite(RPWM_A, LOW);
  }
  else // ipwm_L < 0
  {
    if(DEADZONE > 0)
      ipwm_L = map(ipwm_L, -1, -255, -DEADZONE-1, -255);
    
    pwr = toMotorPower(-ipwm_L);
    // Rotate forward: EN = PWM, RPWM = 1, LPWM = 0
    analogWrite(EN_A, (byte)pwr);
    digitalWrite(RPWM_A, HIGH);
    digitalWrite(LPWM_A, LOW);
  }
}

int toMotorPower(int pwr)  // pwr expected to be 0...255, it will be constrained to 0...240
{
  pwr = constrain(pwr, 0, 255);
  return pwr;
}

void StopMotors()  // Both Motors will stop (feather).
{
  // Parking - H-Bridge disabled:
  digitalWrite(RPWM_A, LOW);
  digitalWrite(LPWM_A, LOW);
  digitalWrite(EN_A, LOW);
  digitalWrite(RPWM_B, LOW);
  digitalWrite(LPWM_B, LOW);
  digitalWrite(EN_B, LOW);
}

void setTimerForPWM()
{
}
