
#define SLAVE_I2C_ADDRESS 9

void InitializeI2c()
{
  Wire.begin(SLAVE_I2C_ADDRESS); // Start I2C Bus as a Slave (Device Number SLAVE_I2C_ADDRESS)
  Wire.onReceive(receiveEvent);  // register receive event - command to slave
  Wire.onRequest(requestEvent);  // register request event - asking slave for info
}

// function that executes whenever action is requested by master
// this function is registered as an event, see InitializeI2c()
void receiveEvent(int howMany)
{
  Serial.print("receiveEvent: ");

  // HandElbowShoulder retransmits channel 5-9 packages from Serial to I2C.
  // We receive exact copies here.
  
  int channel = Wire.read();    // receive byte as an integer
  int cmd = Wire.read();
  int xx = Wire.read();
  int cmdValue = xx + (Wire.read() << 8);
  xx = Wire.read();
  int checksum = xx + (Wire.read() << 8);

  if(channel + cmd + cmdValue + checksum != 0)
  {
    // bad transmission
    //x = 1;    // indicates trouble
    Serial.println("Error: I2C checksum");
  }
  /*
  else
  {
    //x = 0; // checksum OK
    Serial.print("ch=");
    Serial.print(channel);
    Serial.print(" cmd=");
    Serial.print(cmd);
    Serial.print(" val=");
    Serial.println(cmdValue);
  }
  */

  switch (channel)
  {
    case 5:
      thumbAngle = myMap((double)cmdValue, 800, 2200, 120, 160);
      break;
    case 6:
      indexAngle = myMap((double)cmdValue, 800, 2200, 150, 90);
      break;
    case 7:
      middleAngle = myMap((double)cmdValue, 800, 2200, 70, 140);
      break;
    case 8:
      pinkyAngle = myMap((double)cmdValue, 800, 2200, 100, 30);
      break;
    case 9:
      wristAngle = myMap((double)cmdValue, 800, 2200, 0, 180);
      break;
  }
}

// function that executes whenever data is requested by master
// this function is registered as an event, see InitializeI2c()
void requestEvent() 
{
  // 6 bytes as expected by master:
  //Serial.println("requestEvent()");

  byte toSend[6];
  int i = 0;

  toSend[i++] = palmSensorValue & 0xFF;
  toSend[i++] = (palmSensorValue >> 8) & 0xFF;

  toSend[i++] = servoCurrentValue & 0xFF;
  toSend[i++] = (servoCurrentValue >> 8) & 0xFF;

  int checksum = -(palmSensorValue + servoCurrentValue);

  toSend[i++] = checksum & 0xFF;
  toSend[i++] = (checksum >> 8) & 0xFF;

  // all 6 bytes must be sent in one write()
  Wire.write(toSend, 6);
}

