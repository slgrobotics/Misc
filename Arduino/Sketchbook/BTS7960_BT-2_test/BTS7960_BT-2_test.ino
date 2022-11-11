/*
IBT-2 Motor Control Board driven by Arduino.
 
Speed and direction controlled by a potentiometer attached to analog input 0.
One side pin of the potentiometer (either one) to ground; the other side pin to +5V
 
Connection to the IBT-2 board:
IBT-2 pin 1 (RPWM) to Arduino pin 5(PWM)
IBT-2 pin 2 (LPWM) to Arduino pin 6(PWM)
IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
IBT-2 pin 8 (GND) to Arduino GND
IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/
 
const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to
 
int RPWM_Output = 9; // Arduino PWM output pin 9; connect to IBT-2 pin 1 (RPWM)
int LPWM_Output = 10; // Arduino PWM output pin 10; connect to IBT-2 pin 2 (LPWM)
int L_EN = 7;
int R_EN = 8;

#define DEADZONE 32
 
void setup()
{
  setPWMfrequency(0x02);  // timer 1 for pins 9,10, set to 3.92KHz

  pinMode(analogInPin, INPUT);

  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  
  Serial.begin(115200);

  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);

}
 
void loop()
{
  int sensorValue = analogRead(analogInPin);
  int reversePWM = 0;
  int forwardPWM = 0;
  
  // sensor value is in the range 0 to 1023
  // the lower half of it we use for reverse rotation; the upper half for forward rotation
  if (sensorValue < 512)
  {
    // reverse rotation
    reversePWM = -(sensorValue - 511) / 2;

    if(DEADZONE > 0)
      reversePWM = map(reversePWM, 1, 255, DEADZONE+1, 255);

    analogWrite(LPWM_Output, 0);
    analogWrite(RPWM_Output, reversePWM);
  }
  else
  {
    // forward rotation
    forwardPWM = (sensorValue - 512) / 2;

    if(DEADZONE > 0)
      forwardPWM = map(forwardPWM, 1, 255, DEADZONE+1, 255);

    analogWrite(LPWM_Output, forwardPWM);
    analogWrite(RPWM_Output, 0);
  }

  Serial.print(sensorValue);
  Serial.print("   ");
  Serial.print(forwardPWM);
  Serial.print("   ");
  Serial.println(reversePWM);
  
  delay(100);
}

void setPWMfrequency(int freq) {
  TCCR1B = TCCR1B & 0b11111000 | freq ;
}
