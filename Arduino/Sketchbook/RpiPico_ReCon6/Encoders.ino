/*-------------Encoder---------------*/

// SmartLab ReCon 6.0 has very basic encoders, just speed via IR interrupter, no direction

const int ENCODER_L_PIN = 6;    // side L (interrupt 0, Left encoder)
const int ENCODER_R_PIN = 7;    // side R (interrupt 1, Right encoder)

// **************************
//     Init the Encoders
// **************************
void encodersInit()
{
  // Left encoder:
  pinMode(ENCODER_L_PIN, INPUT); 

  // Right encoder:
  pinMode(ENCODER_R_PIN, INPUT); 
  
  encodersReset();

  attachInterrupt(ENCODER_L_PIN, leftEncoder, CHANGE);   // left encoder interrupt
  attachInterrupt(ENCODER_R_PIN, rightEncoder, CHANGE);  // right encoder interrupt
}

void encodersReset()
{
  Ldistance = 0;
  Rdistance = 0;
}

// ************************************
//    Read distance from the encoders
// ************************************
void rightEncoder()
{
  // we spend around 12.5us in the interrupt, at approx 1,5-2kHz frequency at pwm=80
  //digitalWrite(DBG1_PIN, HIGH);
  
  //boolean vi = digitalRead(ENCODER_R_PIN);

  // as the direction is not known from encoder alone, we borrow that from motors:
  if((int)pwm_R < 0)
  {
    //digitalWrite(DBG2_PIN, LOW);
    Rdistance--;
    oRdistance--;
  } else if((int)pwm_R > 0) {
    //digitalWrite(DBG2_PIN, HIGH);
    Rdistance++;                // wheel moves forward, positive increase
    oRdistance++;
  }
  //digitalWrite(DBG1_PIN, LOW);
}

void leftEncoder()
{
  // we spend around 12.5us in the interrupt, at approx 1,5-2kHz frequency at pwm=80
  //digitalWrite(DBG1_PIN, HIGH);
  
  //boolean vi = digitalRead(ENCODER_L_PIN);

  // as the direction is not known from encoder alone, we borrow that from motors:
  if((int)pwm_L < 0)
  {
    //digitalWrite(DBG2_PIN, LOW);
    Ldistance--;
    oLdistance--;
  } else if((int)pwm_L > 0) {
    //digitalWrite(DBG2_PIN, HIGH);
    Ldistance++;                // wheel moves forward, positive increase
    oLdistance++;
  }
  //digitalWrite(DBG1_PIN, LOW);
}
