
//#ifdef MTRS
// ========================= region Motors ==============================
void enableMotors()
{
    pinMode(EnA, OUTPUT);  
    pinMode(EnB, OUTPUT);
    
    pinMode(in1, OUTPUT);  
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);  
    pinMode(in4, OUTPUT);
}

const int DEADZONE = 20;
const int WEAKZONE = 80;
//const int minPower = 100;   // accounts for friction in all gears. No need to put too low PWM on a motor if it can't overcome friction. Put enough to actually turn.
//const int minPowerOff = 40; // accounts for deadzone. No need to put any power if the PID output is that small.
int ch1Pwr = -1000;
int ch2Pwr = -1000; 

// motor - 1 (A) or 2 (b)
// power - -255...+255
void SetMotorPower(int motor, int power)
{
  power = constrain(power, -255, 255);

  /*  
  if(power != 0)
  {
    if(abs(power) < minPowerOff)
    {
      power = 0;
    }
    else if(abs(power) < minPower)
    {
      power = power > 0 ? minPower : -minPower;
    }
  }
  */
  
  int side1, side2, enable;  // sides of H-bridge (direction) and bridge enable pins for selected motor

  switch (motor)
  {
    case HeadPanMotorChannel:
      if(power == ch1Pwr)
        return;
      ch1Pwr = power;
      side1 = in1;
      side2 = in2;
      enable = EnA;
      break;

    case LightHbridgeChannel:
      if(power == ch2Pwr)
        return;
      ch2Pwr = power;
      side1 = in3;
      side2 = in4;
      enable = EnB;
      break;

    default:
      return;
  }
  
  if(abs(power) < DEADZONE)
  {
    digitalWrite(side1, LOW);    // both LOW connects the sides to the ground
    digitalWrite(side2, LOW);  
    //digitalWrite(enable, LOW);  // LOW disables the bridge. The bridge will idle (feather) the motor.
    digitalWrite(enable, HIGH);  // HIGH enables the bridge. The bridge will brake the motor.
  }
  else if(power > 0)
  {
    if(WEAKZONE > 0)
      power = map(power, DEADZONE, 255, WEAKZONE+1, 255);
    
    digitalWrite(side1, HIGH);  
    digitalWrite(side2, LOW);  
    analogWrite(enable, power);    // PWM
  }
  else
  {
    if(WEAKZONE > 0)
      power = map(power, -DEADZONE, -255, -WEAKZONE-1, -255);
    
    digitalWrite(side1, LOW);  
    digitalWrite(side2, HIGH);  
    analogWrite(enable, -power);   // PWM
  }
}
// ========================= endregion Motors ==============================
//#endif //MTRS

