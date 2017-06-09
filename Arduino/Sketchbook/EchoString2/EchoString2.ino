/*
  Reading a serial ASCII-encoded string, modify and sending it back.
 */

int led = 13;

int j = 0; // sensor "randomizer" for debugging

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
  if (Serial.available() > 0)
  {
    String str = Serial.readStringUntil('\r');
    if(str == "reset")
    {
        Serial.print("?????? firmware 3.0.0 Copyright 2006-2010, RoboticsConnection.com\r\n>");
        setHigh();
    }
    else
    {
      setLow();
      //Serial.println("read: '" + str + "'");
      char chars[100];
      char* tokens[5];
      int channels[5];
      int pwms[5];
      str.toCharArray(chars, 100);
      int i=0; 

      // strtok replaces " " with '\0'
      const char sep[2] = " ";
      tokens[i] = strtok(chars, sep);
      if(tokens[i] == NULL)
      {
        return;
      }
      
      while (i < 5 && tokens[i] != NULL)
      {
        //Serial.print("token: ");
        //Serial.println(tokens[i]);
        tokens[++i] = strtok(NULL, sep);
      }

      Serial.print(i);

      //Serial.print("\r\n");

      if(!strcmp(tokens[0], "cfg"))
      {
        //Serial.println("OK: cfg");
        Serial.print("\r\nACK\r\n>");
      }
      else if(!strcmp(tokens[0], "sensor"))
      {
        setHigh();
        //Serial.println("OK: sensor");
        Serial.print("\r\n"); Serial.print(83 + j); Serial.print(" \r\n>");
      }
      else if(!strcmp(tokens[0], "srf04"))
      {
        setLow();
        //Serial.println("OK: srf04");
        Serial.print("\r\n"); Serial.print(200 + j); Serial.print("\r\n>");
      }
      else if(!strcmp(tokens[0], "getenc"))
      {
        //Serial.println("OK: getenc");
        switch(atoi(tokens[1]))
        {
          case '1':
            Serial.print("\r\n");
            Serial.print(Ldistance);
            Serial.print("\r\n>");
            break;
          case '2':
            Serial.print("\r\n");
            Serial.print(Rdistance);
            Serial.print("\r\n>");
            break;
          default:
            Serial.print("\r\n0\r\n>");
            break;            
        }
      }
      else if(!strcmp(tokens[0], "pwm"))
      {
        // 'pwm 1:35 2:-65'
        //Serial.println("OK: pwm");
        for(int k=1; k < i ;k++)
        {
          channels[k-1] = atoi(tokens[k]);
          pwms[k-1] = atoi(tokens[k] + 2);

          Serial.print("   ch=");
          Serial.print(channels[k-1]);
          Serial.print("   pwm=");
          Serial.println(pwms[k-1]);

        }
        Serial.print("\r\nACK\r\n>");
      }
      else if(!strcmp(tokens[0], "vel"))
      {
        //Serial.println("OK: vel");
        Serial.print("\r\n0 0 \r\n>");
      }
      else
      {
        Serial.println("****************** unknown read: '" + str + "'");
      }
    }
    //Serial.print("\r\n>");
  }
  if(++j > 50)
    j = 0;
}

void setHigh()
{
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void setLow()
{
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
}

