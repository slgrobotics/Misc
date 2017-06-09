/*-------------Encoder---------------*/

// interrupts:
// Most Arduino boards have two external interrupts: number 0 (on digital pin 2) and 1 (on digital pin 3)

/*
  see C:\Projects\Arduino\arduino-1.0.2\libraries\AlfsTechHbridge\AlfsTechHbridge.cpp
  // Encoders:
  // *** Declarations - Arduino UNO Atmel 328PU (28-pin DIP) ***
  //     Arduino Data Pin       Atmel 328PU pin 
  ENCODER_A_B   = 2;		// 4	        // side B (interrupt 0) - right
  ENCODER_B_B   = 8;		// 14	        // side B

  ENCODER_A_A   = 3;		// 5	        // side A (interrupt 1) - left
  ENCODER_B_A   = 10;		// 16	        // side A
*/

// **************************
//     Init the Encoders
// **************************
void EncodersInit()
{
  EncodersReset();

  attachInterrupt(0, rightEncoder, CHANGE);   // int 0, pin 2  - left encoder interrupt, side B
  attachInterrupt(1, leftEncoder, CHANGE);    // int 1, pin 3  - right encoder interrupt, side A
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
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  
  //boolean vi = (PIND & _BV(PIND3)) == 0; // read pin D3 (PD3)   - works for UNO 
#ifdef ALFS_HBRIDGE
  boolean vi = digitalReadFast(motors.ENCODER_A_A) == LOW;
  boolean vd = digitalReadFast(motors.ENCODER_B_A) == LOW;
#endif // ALFS_HBRIDGE

#ifdef DIRPWM_HBRIDGE
  boolean vi = digitalReadFast(ENCODER_A_A) == LOW;
  boolean vd = digitalReadFast(ENCODER_B_A) == HIGH;
#endif // DIRPWM_HBRIDGE
  
  if(vi == vd)
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
  
#ifdef ALFS_HBRIDGE
  boolean vi = digitalReadFast(motors.ENCODER_A_B) == LOW;
  boolean vd = digitalReadFast(motors.ENCODER_B_B) == HIGH;
#endif // ALFS_HBRIDGE

#ifdef DIRPWM_HBRIDGE
  boolean vi = digitalReadFast(ENCODER_A_B) == LOW;
  boolean vd = digitalReadFast(ENCODER_B_B) == HIGH;
#endif // DIRPWM_HBRIDGE
  
  if(vi == vd)
  {
    Rdistance--;
  } else {
    Rdistance++;                // wheel moves forward, positive increase
  }
}

