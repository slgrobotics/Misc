/*
  Reading a serial ASCII-encoded string from Serial1, send it to Serial (Mega, Leonardo).
 */

int led = 13;

void setup()
{
  // initialize serial:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only - Leonardo
  }
  Serial1.begin(57600); // connect device to pins 19 (RX) and 18 (TX)

  pinMode(led, OUTPUT);     
  setLow();
}

void loop()
{
  // if there's any serial available, read it:
  while (Serial1.available() > 0)
  {
    String str = Serial1.readStringUntil('\n');
    Serial.println("read: '" + str + "'");

    /*
    for(int i=0; i < 10; i++)
    {
      setHigh();
      delay(100);  // ms
      setLow();
      delay(100);  // ms
    }
    */
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

