/*-------------Encoder---------------*/

// interrupts:
// Most Arduino boards have two external interrupts: number 0 (on digital pin 2) and 1 (on digital pin 3)

// **************************
//     Init the Encoders
// **************************
void EncodersInit()
{
  pinModeFast(2, INPUT);  // left, INT 0
  pinModeFast(8, INPUT);  // left

  pinModeFast(3, INPUT);  // right, INT 1
  pinModeFast(9, INPUT);  // right

  attachInterrupt(0, leftEncoder, CHANGE);   // int 0, pin D2  - left encoder interrupt
  attachInterrupt(1, rightEncoder, CHANGE);  // int 1, pin D3  - right encoder interrupt
}

// ************************************
//    Read distance from the encoders
// ************************************
void leftEncoder()
{
  // we spend around 12.5us in the interrupt, at approx 1,5-2kHz frequency at pwm=80
  //digitalWrite(10, HIGH);
  
  // for all boards we use digitalReadFast():
  boolean vi = digitalReadFast(2) == 0; //(PIND & _BV(PIND2)) == 0; // read pin D2 (PD2 on UNO)
  boolean vd = digitalReadFast(8) == 0; //(PINB & _BV(PINB0)) == 0; // read pin D8 (PB0)
  
  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Ldistance--;
    oLdistance--;
  } else {
    //digitalWrite(11, HIGH);
    Ldistance++;                // wheel moves forward, positive increase
    oLdistance++;
  }
  //digitalWrite(10, LOW);
}

void rightEncoder()
{
  // we spend around 12.5us in the interrupt, at approx 1,5-2kHz frequency at pwm=80
  //digitalWrite(10, HIGH);
  
  // for all boards we use digitalReadFast():
  boolean vi = digitalReadFast(3) == 0; //(PIND & _BV(PIND3)) == 0; // read pin D3 (PD3 on UNO)
  boolean vd = digitalReadFast(9) == 0; //(PINB & _BV(PINB1)) == 0; // read pin D9 (PB1)
  
  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Rdistance++;                // wheel moves forward, positive increase
    oRdistance++;
  } else {
    //digitalWrite(11, HIGH);
    Rdistance--;
    oRdistance--;
  }
  //digitalWrite(10, LOW);
}

