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

  Serial.print("Gyro:   X=");
  Serial.print(GyroX);
  Serial.print("   Y=");
  Serial.print(GyroY);
  Serial.print("   Z=");
  Serial.print(GyroZ);
  Serial.print("   Temp(C)=");
  Serial.println(GyroTemp);

  Serial.print("Angle calculation:   acceleration=");
  Serial.print(acceleration);
  Serial.print("   theta=");
  Serial.print(theta);
  Serial.print("       After Kalman:   angle=");
  Serial.print(angle);
  Serial.print("   angle_dot=");
  Serial.println(angle_dot);

  Serial.print("pwm=");
  Serial.print(pwm);
  Serial.print("   direction_error_all=");
  Serial.println(direction_error_all);

  Serial.print("Motors:   Right pwm_R: ");
  Serial.print(pwm_R);
  Serial.print("       Left pwm_L: ");
  Serial.println(pwm_L);

  Serial.print("Encoders:  Right Rdistance: ");
  Serial.print(oRdistance);
  Serial.print("  ");
  Serial.print(Rdur);
  Serial.print("       Left Ldistance: ");
  Serial.print(oLdistance);
  Serial.print("  ");
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
}
#endif

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

void shortBuzz()
{
  // TODO: use http://playground.arduino.cc/Code/Timer1
  digitalWrite(buzzer,HIGH);
  delay(100);  
  digitalWrite(buzzer,LOW);
}

// debugging LEDs:
void DbgRed(bool on)
{
  digitalWriteFast(RED_LED_PIN, on);
}

void DbgBlue(bool on)
{
  digitalWriteFast(BLUE_LED_PIN, on);
}

void DbgGreen(bool on)
{
  digitalWriteFast(GREEN_LED_PIN, on);
}


