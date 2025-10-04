/*-------------Encoder---------------*/

#ifdef HAS_ENCODERS

// interrupts:
// Most Arduino boards have two external interrupts: number 0 (on digital pin 2) and 1 (on digital pin 3)

const int ENCODER_L_A = 3;    // side L (interrupt 1, Left encoder)
const int ENCODER_L_B = 10;   // side L data

const int ENCODER_R_A = 2;    // side R (interrupt 0, Right encoder)
const int ENCODER_R_B = 8;    // side R data

// we use Oak Grigsby Optical Encoders, type 96Q100-50-00355

// **************************
//     Init the Encoders
// **************************
void EncodersInit()
{
  // Left encoder:
  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);

  // Right encoder:
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

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
// one full wheel rotation on Plucky produces 2506 ticks
// ********************************************************
void leftEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80

  //boolean vi = (PIND & _BV(PIND3)) == 0; // read pin D3 (PD3)   - works for UNO

  boolean vi = digitalRead(ENCODER_L_A) == LOW;
  boolean vd = digitalRead(ENCODER_L_B) == HIGH;

  if (vi == vd)
  {
    Ldistance--;
  } else {
    Ldistance++;                // wheel moves forward, positive increase
  }
}

void rightEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80

  //boolean vi = (PIND & _BV(PIND2)) == 0; // read pin D2 (PD2)   - works for UNO

  boolean vi = digitalRead(ENCODER_R_A) == LOW;
  boolean vd = digitalRead(ENCODER_R_B) == HIGH;

  if (vi == vd)
  {
    Rdistance--;
  } else {
    Rdistance++;                // wheel moves forward, positive increase
  }
}

#endif // HAS_ENCODERS
