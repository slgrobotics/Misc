void InitializeI2c()
{
  Wire.begin(); // Start I2C Bus as Master
  //Fastwire::setup(400, 0);
}

double lastCompassYaw = 0.0;

void receiveI2cCompassPacket()
{
  // see C:\Projects\Arduino\Sketchbook\MotionPlug

  compassYaw = mympu.ypr[0];  // -180.0,180.0

  if(lastCompassYaw != compassYaw)
  {
    lastCompassYaw = compassYaw;
    lastImu = millis();
  }
}

void receiveI2cSonarPacket()
{
  // see C:\Projects\Arduino\Sketchbook\ParkingSensorI2C
  
  int checksum = 0;

  Wire.requestFrom(SONAR_I2C_ADDRESS, 6);    // request 6 bytes from slave device

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
    lastSonar = millis();
  }
  else
  {
//    Serial.println("Error: bad transmission from Slave");
        rangeFRcm = -1;
        rangeFLcm = -1;
        rangeBRcm = -1;
        rangeBLcm = -1;
  }
}

