/*
  Reading a serial ASCII-encoded string, modify and sending it back.
 */

int led = 13;

void setup()
{
  // initialize serial:
  Serial.begin(19200);
  pinMode(led, OUTPUT);     
  setLow();
}

void loop()
{
  // if there's any serial available, read it:
  while (Serial.available() > 0)
  {
    String str = Serial.readStringUntil('\r');
    if(str == "reset")
    {
        Serial.println("?????? firmware 3.0.0 Copyright 2006-2010, RoboticsConnection.com\r\n\r\n");
        //Serial.println("read: '" + str + "'");
        setHigh();
        delay(1000);  // ms
        setLow();
        Serial.println("?????? firmware 3.0.0 Copyright 2006-2010, RoboticsConnection.com\r\n\r\n");
    }
    else
    {
      //Serial.println("read: '" + str + "'");
      for(int i=0; i < 10; i++)
      {
        setHigh();
        delay(100);  // ms
        setLow();
        delay(100);  // ms
      }
    }
  }
}

void setHigh()
{
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void setLow()
{
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
}

