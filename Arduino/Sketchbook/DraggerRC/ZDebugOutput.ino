
void InitLeds()
{
// debugging LEDs:
  pinModeFast(redLedPin, OUTPUT);
  pinModeFast(blueLedPin, OUTPUT);
  pinModeFast(greenLedPin, OUTPUT);
}

#ifdef TRACE
void printAll()
{
  // takes 34ms and completely stops the 5ms cycle
  int Ldur = Ldistance;
  int Rdur = Rdistance;

  Serial.println("--------------------");

  Serial.print("K1=");
  Serial.print(k1);
  Serial.print("    K2=");
  Serial.print(k2);
  Serial.print("    K3=");
  Serial.print(k3);
  Serial.print("    K4=");
  Serial.println(k4);

  // display the raw RC Channel PWM Inputs
  Serial.print("R/C:   Right: ");
  Serial.print(PW_RIGHT);
  Serial.print("       Left: ");
  Serial.println(PW_LEFT);

  Serial.print("Desired speed %:   Right: ");
  Serial.print(setpointSpeedR);
  Serial.print("       Left: ");
  Serial.println(setpointSpeedL);

  Serial.print("Encoders:  Right Rdistance: ");
  Serial.print(Rdur);
  Serial.print("       Left Ldistance: ");
  Serial.println(Ldur);

  Serial.print("Measured speed %:   Right: ");
  Serial.print(speedMeasured_R);
  Serial.print("       Left: ");
  Serial.println(speedMeasured_L);

  Serial.print("PID:   Right dpwm_R: ");
  Serial.print(dpwm_R);
  Serial.print("       Left dpwm_L: ");
  Serial.println(dpwm_L);

  Serial.print("Motors:   Right pwm_R: ");
  Serial.print(pwm_R);
  Serial.print("       Left pwm_L: ");
  Serial.println(pwm_L);
}
#endif

void shortBuzz()
{
  // TODO: use http://playground.arduino.cc/Code/Timer1
  digitalWrite(buzzerPin,HIGH);
  delay(100);  
  digitalWrite(buzzerPin,LOW);
}

// debugging LEDs:
void DbgRed(bool on)
{
  digitalWriteFast(redLedPin, on);
}

void DbgBlue(bool on)
{
  digitalWriteFast(blueLedPin, on);
}

void DbgGreen(bool on)
{
  digitalWriteFast(greenLedPin, on);
}
