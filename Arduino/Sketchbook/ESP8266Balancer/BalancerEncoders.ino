/*-------------Encoder---------------*/

// interrupts:
// must have to avoid crashing on ESP8266:
void ICACHE_RAM_ATTR leftEncoder();
void ICACHE_RAM_ATTR rightEncoder();

// **************************
//     Init the Encoders
// **************************
void initEncoders()
{
  pinMode(D4, INPUT_PULLUP);  // left
  pinMode(D5, INPUT_PULLUP);  // right

  attachInterrupt(digitalPinToInterrupt(D4), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(D5), rightEncoder, CHANGE);
}

// ************************************
//    Read distance from the encoders
// ************************************
void leftEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(debugPin1, HIGH);
  
  if(pwm_L < 0)
  {
    //digitalWrite(debugPin2, LOW);
    Ldistance--;
  } else if(pwm_L > 0) {
    //digitalWrite(debugPin2, HIGH);
    Ldistance++;                // wheel moves forward, positive increase
  }
  //digitalWrite(debugPin1, LOW);
}

void rightEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(debugPin1, HIGH);
  
  if(pwm_R > 0)
  {
    //digitalWrite(debugPin2, LOW);
    Rdistance++;                // wheel moves forward, positive increase
  } else if(pwm_R < 0) {
    //digitalWrite(debugPin2, HIGH);
    Rdistance--;
  }
  //digitalWrite(debugPin1, LOW);
}
