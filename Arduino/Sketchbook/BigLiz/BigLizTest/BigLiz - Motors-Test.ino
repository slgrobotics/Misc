
// connect arduino pins 2...5 to motor shield in1...in4 (Direction)  and pin 9,10 to EnA,EnB (PWM)
int EnA =  9;  
int EnB = 10;  
int in1 = 2;
int in2 = 3;                        
int in3 = 4;                          
int in4 = 5;                          

void setup()
{
  enableMotors();
}

void loop()
{
  int value;

  for(value = -255 ; value <= 255; value+=1)
  {
    SetMotorPower(1, value);
    SetMotorPower(2, value);
    delay(50);
  } 

  SetMotorPower(1, 0);
  SetMotorPower(2, 0);
  delay(3000);
}

void enableMotors()
{
    pinMode(EnA, OUTPUT);  
    pinMode(EnB, OUTPUT);
    
    pinMode(in1, OUTPUT);  
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);  
    pinMode(in4, OUTPUT);
}

// motor - 1 (A) or 2 (b)
// power - -255...+255
void SetMotorPower(int motor, int power)
{
  power = constrain(power, -255, 255);
  int side1, side2, enable;

  switch (motor)
  {
    case 1:
      side1 = in1;
      side2 = in2;
      enable = EnA;
      break;

    case 2:
      side1 = in3;
      side2 = in4;
      enable = EnB;
      break;

    default:
      return;
  }
  
  if(power == 0)
  {
    digitalWrite(side1, HIGH);    // both HIGH disables the sides
    digitalWrite(side2, HIGH);  
    digitalWrite(enable, LOW);    // LOW disables the bridge
  }
  else if(power > 0)
  {
    digitalWrite(side1, HIGH);  
    digitalWrite(side2, LOW);  
    analogWrite(enable, power);    // PWM
  }
  else
  {
    digitalWrite(side1, LOW);  
    digitalWrite(side2, HIGH);  
    analogWrite(enable, -power);   // PWM
  }
}

