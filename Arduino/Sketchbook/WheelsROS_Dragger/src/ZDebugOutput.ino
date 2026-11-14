
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
  double Ldur = (double)Ldistance;
  double Rdur = (double)Rdistance;

  Serial.println("--------------------");

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

  Serial.print("Joystick:   in control: ");
  Serial.print(isControlByJoystick());
  Serial.print("  pressed: ");
  Serial.print(isJoystickPressed());
  Serial.print(" X: ");
  Serial.print(joystickX());
  Serial.print(" Y: ");
  Serial.print(joystickY());
  Serial.print(" joystick speed L: ");
  Serial.print(joystickSpeedL);
  Serial.print(" speed R: ");
  Serial.println(joystickSpeedR);

  Serial.print("Sonar: FR: ");
  Serial.print(rangeFRcm);
  Serial.print(" FL: ");
  Serial.print(rangeFLcm);
  Serial.print(" BR: ");
  Serial.print(rangeBRcm);
  Serial.print(" BL: ");
  Serial.println(rangeBLcm);
}
#endif

void shortBuzz()
{
  // TODO: use http://playground.arduino.cc/Code/Timer1
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
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
