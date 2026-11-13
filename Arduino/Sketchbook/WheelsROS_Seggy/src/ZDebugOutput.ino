/* For debugging purposes */

void InitLeds()
{
  pinMode (mainLedPin, OUTPUT);  // Status LED

#ifdef HAS_LEDS
  // diagnostic LEDs:
  pinMode (redLedPin, OUTPUT);
  pinMode (blueLedPin, OUTPUT);
  pinMode (greenLedPin, OUTPUT);
  pinMode (whiteLedPin, OUTPUT);
#endif // HAS_LEDS

  blinkLED(10, 50);

  digitalWrite(mainLedPin, HIGH);

  /*
    digitalWrite(redLedPin, HIGH);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(blueLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(whiteLedPin, HIGH);
  */
}

void buzz(int nTimes, int buzzDurationMs, int silenceDurationMs)
{
  for (int i = 0; i < nTimes; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(buzzDurationMs);
    digitalWrite(BUZZER_PIN, LOW);
    delay(silenceDurationMs);
  }
}

void blinkLED(int nTimes, int halfPeriodMs)
{
  for (int i = 0; i < nTimes; i++)
  {
    digitalWrite(mainLedPin, HIGH);
    delay(halfPeriodMs);
    digitalWrite(mainLedPin, LOW);
    delay(halfPeriodMs);
  }
}

#ifdef TRACE

#define PRINT_INTERVAL_MS 5000

unsigned long lastPrintMillis = 0;

// warning: this takes significant time and disrupts control loop a bit
void printAll()
{
  if (millis() - lastPrintMillis > PRINT_INTERVAL_MS)
  {
    lastPrintMillis = millis();

    double Ldur = (double)Ldistance;
    double Rdur = (double)Rdistance;

    Serial.print("-------------------- ");
    Serial.print((loopCnt - lastLoopCnt) / (PRINT_INTERVAL_MS / 1000));
    Serial.println(" loops/sec");

    Serial.print("Battery: ADC: ");
    Serial.print(analogRead(batteryVoltageInPin));
    Serial.print("\tmV: ");
    Serial.print(battery_voltage_mv);
    Serial.print("\tmA: ");
    Serial.println(battery_current_ma);

    Serial.print("Left  DC: ");
    Serial.print(current_dc_L);
    Serial.print(" A\tPhases:\t");
    Serial.print(current_ph_L.a);  // milli Amps
    Serial.print("\t");
    Serial.print(current_ph_L.b);
    Serial.print("\t");
    Serial.println(current_ph_L.c);

    Serial.print("Right DC: ");
    Serial.print(current_dc_R);
    Serial.print(" A\tPhases:\t");
    Serial.print(current_ph_R.a);  // milli Amps
    Serial.print("\t");
    Serial.print(current_ph_R.b);
    Serial.print("\t");
    Serial.println(current_ph_R.c);

    Serial.print("Speed setpoints %: Left: ");
    Serial.print(speedSetpointL);
    Serial.print("\tRight: ");
    Serial.println(speedSetpointR);

    Serial.print("Speed measured:    Left: ");
    Serial.print(motorL.shaft_velocity);
    Serial.print("\tRight: ");
    Serial.println(motorR.shaft_velocity);

    Serial.print("Encoders:  Right Rdistance: ");
    Serial.print(Rdur);
    Serial.print("       Left Ldistance: ");
    Serial.println(Ldur);

    Serial.print("Measured speed %:   Right: ");
    Serial.print(speedMeasured_R);
    Serial.print("       Left: ");
    Serial.println(speedMeasured_L);

    Serial.print("Joystick:   active: ");
    Serial.print(isJoystickActive());
    Serial.print("  pressed: ");
    Serial.print(isJoystickPressed());
    Serial.print(" X: ");
    Serial.print(joystickX());
    Serial.print(" Y: ");
    Serial.print(joystickY());
    Serial.print(" joystick speed L: ");
    Serial.print(joystickSpeedL);
    Serial.print(" R: ");
    Serial.println(joystickSpeedR);

    lastLoopCnt = loopCnt;
  }
}

// limits degrees "a" to 0...360
double to360(double a)
{
  if (a > 0.0)
  {
    while (a >= 360.0)
    {
      a -= 360.0;
    }
  }
  else if (a < 0.0)
  {
    while (a < 0.0)
    {
      a += 360.0;
    }
  }

  return a;
}
#endif
