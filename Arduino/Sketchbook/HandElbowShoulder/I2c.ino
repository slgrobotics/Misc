
#define SLAVE_I2C_ADDRESS 9

void InitializeI2c()
{
  Wire.begin(); // Start I2C Bus as Master
}

void sendI2cPacket(int channel, int cmd, int cmdValue, int checksum)
{
  Wire.beginTransmission(SLAVE_I2C_ADDRESS); // transmit to device #SLAVE_I2C_ADDRESS
  Wire.write(channel); 
  Wire.write(cmd);
  Wire.write(cmdValue & 0xFF);
  Wire.write((cmdValue >> 8) & 0xFF);
  Wire.write(checksum & 0xFF);
  Wire.write((checksum >> 8) & 0xFF);
  Wire.endTransmission();    // stop transmitting
}

void receiveI2cPacket()
{
  int _palmSensorValue = -1;
  int _servoCurrentValue = -1;
  int checksum = 0;

  Wire.requestFrom(SLAVE_I2C_ADDRESS, 6);    // request 6 bytes from slave device

  int i = 1;
  while (Wire.available())  // slave may send less than requested
  {
    unsigned int b = Wire.read();    // receive a byte, cast to int for shifting
    switch (i)
    {
      case 1:
        _palmSensorValue = b;
        break;
      
      case 2:
        _palmSensorValue += (b << 8);
        break;
      
      case 3:
        _servoCurrentValue = b;
        break;
      
      case 4:
        _servoCurrentValue += (b << 8);
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

  if (i == 7 && checksum + _palmSensorValue + _servoCurrentValue == 0)
  {
    palmSensorValue = _palmSensorValue;
    servoCurrentValue = _servoCurrentValue;
//    Serial.print(palmSensorValue);
//    Serial.print(" ");
//    Serial.println(servoCurrentValue);
  }
//  else
//  {
//    Serial.println("bad transmission");
//  }
}

