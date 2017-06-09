/* For debugging purposes */

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

#define PRINT_INTERVAL_MS 5000

unsigned long lastPrintMillis = 0;

// warning: this takes significant time and disrupts control loop a bit
void printAll()
{
  if (millis() - lastPrintMillis > PRINT_INTERVAL_MS)
  {
    lastPrintMillis = millis();

    if (doTRACE)
    {
      long Ldur = Ldistance;
      long Rdur = Rdistance;

      Serial.print("-------------------- ");
      Serial.print((loopCnt - lastLoopCnt) / (PRINT_INTERVAL_MS / 1000));
      Serial.println(" loops/sec");

      Serial.print("Motors:   Right pwm_R: ");
      Serial.print(pwm_R);
      Serial.print("       Left pwm_L: ");
      Serial.println(pwm_L);

      Serial.print("Speed:   Right: ");
      Serial.print(speedMeasured_R);
      Serial.print("       Left: ");
      Serial.println(speedMeasured_L);

      Serial.print("PID_L:   speedMeasured_L: ");
      Serial.print(speedMeasured_L);
      Serial.print("       dpwm_L: ");
      Serial.print(dpwm_L);
      Serial.print("       distL: ");
      Serial.print(distL);
      Serial.print("       desiredSpeedL: ");
      Serial.println(desiredSpeedL);

      Serial.print("Encoders:  Right Rdistance: ");
      Serial.print(Rdur);
      Serial.print("       Left Ldistance: ");
      Serial.println(Ldur);

      Serial.print("Odometry: X=");
      Serial.print(X);
      Serial.print("    Y=");
      Serial.print(Y);
      Serial.print(" meters    Theta=");
      Serial.print(Theta);
      Serial.print("  (");
      Serial.print(to360(Theta * 57.295));
      Serial.println(" degrees)");

      lastLoopCnt = loopCnt;
    }
  }
}

// limits degrees "a" to 0...360
double to360(double a)
{
  if(a > 0.0)
  {
    while (a >= 360.0)
    {
      a -= 360.0;
    }
  }
  else if(a < 0.0)
  {
    while (a < 0.0)
    {
      a += 360.0;
    }
  }
  
  return a;
}


