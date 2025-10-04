/* For debugging purposes */

void InitLeds()
{
  pinMode (ledPin, OUTPUT);  // Status LED

  // diagnostic LEDs:
  pinMode (redLedPin, OUTPUT);
  pinMode (yellowLedPin, OUTPUT);
  pinMode (blueLedPin, OUTPUT);
  pinMode (greenLedPin, OUTPUT);
  pinMode (whiteLedPin, OUTPUT);

  blinkLED(10, 50);

  digitalWrite(ledPin, HIGH);

  /*
    digitalWrite(redLedPin, HIGH);
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(blueLedPin, HIGH);
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(whiteLedPin, HIGH);
  */
}

void blinkLED(int nTimes, int halfPeriodMs)
{
  for (int i = 0; i < nTimes; i++)
  {
    digitalWrite(ledPin, HIGH);
    delay(halfPeriodMs);
    digitalWrite(ledPin, LOW);
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

#ifdef HAS_ENCODERS
    double Ldur = (double)Ldistance;
    double Rdur = (double)Rdistance;
#endif // HAS_ENCODERS

    Serial.print("-------------------- ");
    Serial.print((loopCnt - lastLoopCnt) / (PRINT_INTERVAL_MS / 1000));
    Serial.println(" loops/sec");

    Serial.print("Speed setpoints %:   Right: ");
    Serial.print(setpointSpeedR);
    Serial.print("       Left: ");
    Serial.println(setpointSpeedL);

#ifdef HAS_ENCODERS
    Serial.print("Encoders:  Right Rdistance: ");
    Serial.print(Rdur);
    Serial.print("       Left Ldistance: ");
    Serial.println(Ldur);

    Serial.print("Measured speed %:   Right: ");
    Serial.print(speedMeasured_R);
    Serial.print("       Left: ");
    Serial.println(speedMeasured_L);
#endif // HAS_ENCODERS

#ifdef USE_PIDS
    Serial.print("PID:   Right dpwm_R: ");
    Serial.print(dpwm_R);
    Serial.print("       Left dpwm_L: ");
    Serial.println(dpwm_L);
#endif // USE_PIDS

    Serial.print("Motors:   pwm_R: ");
    Serial.print(pwm_R);
    Serial.print("  pwm_L: ");
    Serial.print(pwm_L);
    Serial.print("       angles: steer: ");
    Serial.print(angle_steer);
    Serial.print("  throttle: ");
    Serial.println(angle_throttle);

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
    Serial.print(" speed R: ");
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
