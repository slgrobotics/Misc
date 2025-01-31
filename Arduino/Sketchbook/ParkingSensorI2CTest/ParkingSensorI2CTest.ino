
// see I2CTestMaster

#include <Wire.h>

#define led 13
#define SLAVE_I2C_ADDRESS 9

// looks like AVR MCs don't have this defined, but it works:
#define WIRE_HAS_TIMEOUT

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
  
  Serial.println("I2C Master ready...");
}

void loop()
{
  receiveI2cPacket();

  delay(450);
}

void InitializeI2c()
{
  Wire.begin(); // Start I2C Bus as Master, clock at 100kHz by default
  //Wire.setClock( 400000UL); // clock at 400 kHz
 #if defined(WIRE_HAS_TIMEOUT)
  // see https://docs.arduino.cc/language-reference/en/functions/communication/wire/setWireTimeout/
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
 #endif
}

void receiveI2cPacket()
{
  int checksum = 0;

#if defined(WIRE_HAS_TIMEOUT)
  Wire.clearWireTimeoutFlag();
#endif

  byte len = Wire.requestFrom(SLAVE_I2C_ADDRESS, 6);    // request 6 bytes from slave device

  if (len != 6) {
    Serial.print("I2C Error occured when reading sonar, len=");
    Serial.println(len);
#if defined(WIRE_HAS_TIMEOUT)
    if (Wire.getWireTimeoutFlag())
    {
      Serial.println("It was a timeout");
    }
#endif

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
    Serial.println("Error: bad transmission from I2C Slave");
  }
}
