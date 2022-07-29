/*-------------Encoder---------------*/

// interrupts:
// must have to avoid crashing on ESP8266:
void ICACHE_RAM_ATTR leftEncoderIsr();
void ICACHE_RAM_ATTR rightEncoderIsr();

// **************************
//     Init the Encoders
// **************************
void initEncoders()
{
  pinMode(leftEncoderPin, INPUT_PULLUP);
  pinMode(rightEncoderPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), leftEncoderIsr, FALLING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), rightEncoderIsr, FALLING);
}

// ************************************
//    Read distance from the encoders
// ************************************
void leftEncoderIsr()
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

void rightEncoderIsr()
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
