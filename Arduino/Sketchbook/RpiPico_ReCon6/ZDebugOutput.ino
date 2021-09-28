
void ledsInit()
{
// debugging LEDs:
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
}

#ifdef TRACE
void printAll()
{
  // takes 34ms and completely stops the 5ms cycle
  int Ldur = Ldistance;
  int Rdur = Rdistance;

  Serial.println("--------------------");

//  Serial.print("K1=");
//  Serial.print(k1);
//  Serial.print("    K2=");
//  Serial.print(k2);
//  Serial.print("    K3=");
//  Serial.print(k3);
//  Serial.print("    K4=");
//  Serial.println(k4);

  Serial.print("Sonars:   Left: ");
  Serial.print(sonarLF);
  Serial.print("   Front: ");
  Serial.print(sonarF);
  Serial.print("   Right: ");
  Serial.println(sonarRF);

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
  
  Serial.print("Odometry Encoders:  R: ");
  Serial.print(oRdistance);
  Serial.print("  L: ");
  Serial.print(oLdistance);

  Serial.print("     Pose: X=");
  Serial.print(X);
  Serial.print("  Y=");
  Serial.print(Y);
  Serial.print(" meters    Theta=");
  Serial.print(Theta);
  Serial.print("  (");
  Serial.print(to360(Theta * 57.295));
  Serial.println(" degrees)");

  printYawPitchRoll();
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

#endif // TRACE

void shortBuzz()
{
  // TODO: use http://playground.arduino.cc/Code/Timer1
  for(int i=0; i < 1000 ;i++)
  {
    digitalWrite(buzzerPin,HIGH);
    delay(1);  
    digitalWrite(buzzerPin,LOW);
    delay(1);  
  }
}

// debugging LEDs:
void DbgRed(bool on)
{
  digitalWrite(redLedPin, on);
}

void DbgBlue(bool on)
{
  digitalWrite(blueLedPin, on);
}

void DbgGreen(bool on)
{
  digitalWrite(greenLedPin, on);
}
