
// see I2CTestMaster

#include <Wire.h>

#define led 13
#define SLAVE_I2C_ADDRESS 9

byte x = 0;

// received from Slave, readings in centimeters:
volatile int rangeFRcm;
volatile int rangeFLcm;
volatile int rangeBRcm;
volatile int rangeBLcm;

void setup()
{
  Serial.begin(115200);

  InitializeI2c();
  
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  
  Serial.println("Master ready...");
}

void loop()
{
  receiveI2cPacket();

  delay(450);
}

void InitializeI2c()
{
  Wire.begin(); // Start I2C Bus as Master
}

void receiveI2cPacket()
{
  int checksum = 0;

  Wire.requestFrom(SLAVE_I2C_ADDRESS, 6);    // request 6 bytes from slave device

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
    Serial.print(rangeFRcm);
    Serial.print(" ");
    Serial.print(rangeFLcm);
    Serial.print(" ");
    Serial.print(rangeBRcm);
    Serial.print(" ");
    Serial.print(rangeBLcm);
    Serial.print(" ");
    Serial.println(checksum);
  }
  else
  {
    Serial.println("Error: bad transmission from Slave");
  }
}

