
// interrupts:
// Most Arduino boards have two external interrupts: number 0 (on digital pin 2) and 1 (on digital pin 3)

const int ENCODER_L_A = 3;    // side L (interrupt 1, Left encoder)
const int ENCODER_L_B = 9;    // side L data

const int ENCODER_R_A = 2;    // side R (interrupt 0, Right encoder)
const int ENCODER_R_B = 8;    // side R data

// we use Oak Grigsby Optical Encoders, type 96Q100-50-00355

// **************************
//     Init the Encoders
// **************************
void EncodersInit()
{
  // Left encoder:
  pinModeFast(ENCODER_L_A, INPUT); 
  pinModeFast(ENCODER_L_B, INPUT); 

  // Right encoder:
  pinModeFast(ENCODER_R_A, INPUT); 
  pinModeFast(ENCODER_R_B, INPUT); 
  
  EncodersReset();

  attachInterrupt(1, leftEncoder, CHANGE);   // int 1, pin D3  - left encoder interrupt
  attachInterrupt(0, rightEncoder, CHANGE);  // int 0, pin D2  - right encoder interrupt
}

void EncodersReset()
{
  Ldistance = 0;
  Rdistance = 0;
  LdistancePrev = 0;
  RdistancePrev = 0;
}

// ********************************************************
// Read distance from the encoders
//
// one full wheel rotation on Dragger produces 13730 ticks
// ********************************************************
void leftEncoder()
{
  // we spend around 12.5us in the interrupt, at approx 1,5-2kHz frequency at pwm=80
  //digitalWrite(10, HIGH);
  
  // for all boards we use digitalReadFast():
  boolean vi = digitalReadFast(ENCODER_L_A) == 0; //(PIND & _BV(PIND3)) == 0; // read pin D3 (PD3 on UNO)
  boolean vd = digitalReadFast(ENCODER_L_B) == 0; //(PINB & _BV(PINB1)) == 0; // read pin D9 (PB1)
  
  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Ldistance++;                // wheel moves forward, positive increase
  } else {
    //digitalWrite(11, HIGH);
    Ldistance--;
  }
  //digitalWrite(10, LOW);
}

void rightEncoder()
{
  // we spend around 12.5us in the interrupt, at approx 1,5-2kHz frequency at pwm=80
  //digitalWrite(10, HIGH);
  
  // for all boards we use digitalReadFast():
  boolean vi = digitalReadFast(ENCODER_R_A) == 0; //(PIND & _BV(PIND2)) == 0; // read pin D2 (PD2 on UNO)
  boolean vd = digitalReadFast(ENCODER_R_B) == 0; //(PINB & _BV(PINB0)) == 0; // read pin D8 (PB0)
  
  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Rdistance--;
  } else {
    //digitalWrite(11, HIGH);
    Rdistance++;                // wheel moves forward, positive increase
  }
  //digitalWrite(10, LOW);
}
