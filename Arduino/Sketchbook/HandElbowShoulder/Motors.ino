//
// Using SeeedStudio.com discontinued Motor Shield v 1.0 - http://www.seeedstudio.com/wiki/Motor_Shield_V1.0
// see C:\Projects\Arduino\Sketchbook\SeeedMotorShieldDemo
// there is an L298N on the shield.
//

void enableMotors()
{
    pinMode(EnA, OUTPUT);  
    pinMode(EnB, OUTPUT);
    pinMode(EnC, OUTPUT);  
    pinMode(EnD, OUTPUT);
    
    pinMode(in1A, OUTPUT);  
    pinMode(in2A, OUTPUT);
    pinMode(in3B, OUTPUT);  
    pinMode(in4B, OUTPUT);
    
    pinMode(in1C, OUTPUT);  
    pinMode(in2C, OUTPUT);
    pinMode(in3D, OUTPUT);  
    pinMode(in4D, OUTPUT);
}

const int powerMax = 255;       // 90 is low enough to not damage stopped motor. Max is 255

const int DEADZONE = 10;
const int WEAKZONE = 0;
//const int minPower = 100;   // accounts for friction in all gears. No need to put too low PWM on a motor if it can't overcome friction. Put enough to actually turn.
//const int minPowerOff = 40; // accounts for deadzone. No need to put any power if the PID output is that small.
int ch1Pwr = -1000;
int ch2Pwr = -1000; 
int ch3Pwr = -1000; 
int ch4Pwr = -1000; 

// motor - 1 (A) or 2 (B)
// power - -255...+255
void SetMotorPower(int motor, int power)
{
  power = constrain(power, -powerMax, powerMax);

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
  
  int side1, side2, enable;  // sides of H-bridge (direction) and bridge enable (power) pins for selected motor

  switch (motor)
  {
    case ElbowMotorChannel:
      if(power == ch1Pwr)
        return;
      ch1Pwr = power;
      side1 = in1A;
      side2 = in2A;
      enable = EnA;
      break;

    case ShoulderPanMotorChannel:
      if(power == ch2Pwr)
        return;
      ch2Pwr = power;
      side1 = in3B;
      side2 = in4B;
      enable = EnB;
      break;

    case ShoulderTiltMotorChannel:
      if(power == ch3Pwr)
        return;
      ch3Pwr = power;
      side1 = in1C;
      side2 = in2C;
      enable = EnC;
      break;

    case ShoulderTurnMotorChannel:
      if(power == ch4Pwr)
        return;
      ch4Pwr = power;
      side1 = in3D;
      side2 = in4D;
      enable = EnD;
      break;

    default:
      return;
  }
  
  if(abs(power) < DEADZONE)
  {
    digitalWrite(side1, LOW);     // both LOW connects the sides to the ground
    digitalWrite(side2, LOW);  
    digitalWrite(enable, LOW);    // LOW disables the bridge. The bridge will idle (feather) the motor.
    //digitalWrite(enable, HIGH); // HIGH enables the bridge. The bridge will brake the motor.
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
    power = -power;
    
    if(WEAKZONE > 0)
      power = map(power, DEADZONE, 255, WEAKZONE+1, 255);
    
    digitalWrite(side1, LOW);  
    digitalWrite(side2, HIGH);  
    analogWrite(enable, power);   // PWM
  }
}

