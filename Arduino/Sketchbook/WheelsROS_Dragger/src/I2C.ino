
#define VERY_CLOSE_RANGE_CM 20

void InitializeI2c()
{
  Wire.begin(); // Start I2C Bus as Master, clock at 100kHz by default
  //Wire.setClock( 400000UL); // clock at 400 kHz
 #if defined(WIRE_HAS_TIMEOUT)
  // see https://docs.arduino.cc/language-reference/en/functions/communication/wire/setWireTimeout/
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
 #endif

#ifdef TRACE
  Serial.println("I2C Master ready...");
#endif // TRACE
}

void receiveI2cSonarPacket()
{
  // see C:\Projects\Arduino\Sketchbook\ParkingSensorI2C

  int checksum = 0;

#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif

  byte len = Wire.requestFrom(SONAR_I2C_ADDRESS, 6);    // request 6 bytes from slave device

  if (len != 6) {
    
#ifdef TRACE
    Serial.print("I2C Error occured when reading sonar, len=");
    Serial.println(len);
  #if defined(WIRE_HAS_TIMEOUT)
    if (Wire.getWireTimeoutFlag())
    {
      Serial.println("It was a timeout");
    }
  #endif
#endif // TRACE

    if(millis() - lastSonarMs > 2000)  // no ping for more than 2 seconds
    {
      rangeFRcm = rangeFLcm = rangeBRcm = rangeBLcm = VERY_CLOSE_RANGE_CM; // set to minimum
    }
    return;
  }
  int i = 1;
  while (Wire.available())  // slave may send less than requested
  {
    int b = Wire.read();    // receive a byte, cast to int for shifting
    //Serial.println(b);
    switch (i)
    {
      case 1:
        rangeFRcm = b;
        break;

      case 2:
        rangeFLcm = b;
        break;

      case 3:
        rangeBRcm = b;
        break;

      case 4:
        rangeBLcm = b;
        break;

      case 5:
        checksum = b;
        break;

      case 6:
        checksum += (b << 8);
        break;
    }
    i++;
  }

  if (i == 7 && checksum + rangeFRcm + rangeFLcm + rangeBRcm + rangeBLcm == 0)
  {
    //    Serial.print(rangeFRcm);
    //    Serial.print(" ");
    //    Serial.print(rangeFLcm);
    //    Serial.print(" ");
    //    Serial.print(rangeBRcm);
    //    Serial.print(" ");
    //    Serial.print(rangeBLcm);
    //    Serial.print(" ");
    //    Serial.println(checksum);
    lastSonarMs = millis();
  }
  else if(millis() - lastSonarMs > 2000)  // no ping for more than 2 seconds
  {
    //    Serial.println("Error: bad transmission from I2C Slave");
    rangeFRcm = rangeFLcm = rangeBRcm = rangeBLcm = VERY_CLOSE_RANGE_CM; // set to minimum
  }
}
