
// https://www.arduino.cc/en/Reference/Wire
//
// see C:\Projects\Arduino\Sketchbook\ParkingSensorI2CTest for a test program

void InitializeI2c()
{
  Wire.begin(SLAVE_I2C_ADDRESS); // Start I2C Bus as a Slave (Device Number SLAVE_I2C_ADDRESS)
  Wire.onRequest(requestEvent);  // register request event - asking slave for info
}

// function that executes whenever data is requested by master
// this function is registered as an event, see InitializeI2c()
void requestEvent() 
{
  //Serial.println("requestEvent()");

  // Just report current values as they are.
  // 6 bytes as expected by master:
  byte toSend[6];
  int i = 0;

  toSend[i++] = rangeFRcm & 0xFF;
  toSend[i++] = rangeFLcm & 0xFF;

  toSend[i++] = rangeBRcm & 0xFF;
  toSend[i++] = rangeBLcm & 0xFF;

  int checksum = -(toSend[0] + toSend[1] + toSend[2] + toSend[3]);

  toSend[i++] = checksum & 0xFF;
  toSend[i++] = (checksum >> 8) & 0xFF;
  
  // all 6 bytes must be sent in one write()
  Wire.write(toSend, 6);
}

