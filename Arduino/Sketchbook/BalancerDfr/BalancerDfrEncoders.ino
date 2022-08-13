/*-------------Encoder---------------*/

// interrupts:
// Most Arduino boards have two external interrupts: number 0 (on digital pin 2) and 1 (on digital pin 3)

// **************************
//     Init the Encoders
// **************************
void initEncoders()
{
  pinMode(2, INPUT);  // left, INT 0
  pinMode(8, INPUT);  // left

  pinMode(3, INPUT);  // right, INT 1
  pinMode(9, INPUT);  // right

  attachInterrupt(0, leftEncoderIsr, CHANGE);   // int 0, pin 2  - left encoder interrupt
  attachInterrupt(1, rightEncoderIsr, CHANGE);  // int 1, pin 3  - right encoder interrupt
}

// ************************************
//    Read distance from the encoders
// ************************************
void leftEncoderIsr()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(10, HIGH);
  
  boolean vi = (PIND & _BV(PIND2)) == 0; // read pin D2 (PD2)
  boolean vd = (PINB & _BV(PINB0)) == 0; // read pin D8 (PB0)
  
  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Ldistance--;
  } else {
    //digitalWrite(11, HIGH);
    Ldistance++;                // wheel moves forward, positive increase
  }
  //digitalWrite(10, LOW);
}

void rightEncoderIsr()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(10, HIGH);
  
  boolean vi = (PIND & _BV(PIND3)) == 0; // read pin D3 (PD3)
  boolean vd = (PINB & _BV(PINB1)) == 0; // read pin D9 (PB1)
  
  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Rdistance++;                // wheel moves forward, positive increase
  } else {
    //digitalWrite(11, HIGH);
    Rdistance--;
  }
  //digitalWrite(10, LOW);
}
