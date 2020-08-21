/*-------------Encoder---------------*/

// interrupts:
// Most Arduino boards have two external interrupts: number 0 (on digital pin 2) and 1 (on digital pin 3)

const int ENCODER_R_A = 3;    // side R (interrupt 0, Right encoder)
const int ENCODER_R_B = 9;    // side R data

const int ENCODER_L_A = 2;    // side L (interrupt 1, Left encoder)
const int ENCODER_L_B = 8;    // side L data

// **************************
//     Init the Encoders
// **************************
void EncodersInit()
{
  // Right encoder:
  pinModeFast(ENCODER_R_A, INPUT); 
  pinModeFast(ENCODER_R_B, INPUT); 
  
  // Left encoder:
  pinModeFast(ENCODER_L_A, INPUT); 
  pinModeFast(ENCODER_L_B, INPUT); 

  EncodersReset();

  attachInterrupt(0, leftEncoder, CHANGE);   // int 0, pin D2  - left encoder interrupt
  attachInterrupt(1, rightEncoder, CHANGE);  // int 1, pin D3  - right encoder interrupt
}

void EncodersReset()
{
  Rdistance = 0;
  Ldistance = 0;
  RdistancePrev = 0;
  LdistancePrev = 0;
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
