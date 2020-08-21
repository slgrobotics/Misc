// **************************
// Remote control code
// **************************

//#define COMM_BALANCER
//#define COMM_SIMPLE
#define COMM_ELEMENT

#define COMM_SERIAL Serial3

void InitSerial()
{
  Serial.begin(115200);   // start serial for USB
  COMM_SERIAL.begin(115200);  // start serial for Funduino Bluetooth XBee
}

#ifdef COMM_ELEMENT

int j = 0; // sensor "randomizer" for srf04 sonar debugging/faking

void readCommCommand()
{
  while (COMM_SERIAL.available())
  {
    String str = COMM_SERIAL.readStringUntil('\r');
    if (str == "reset")
    {
      COMM_SERIAL.print("Arduino firmware Dragger - COMM_ELEMENT\r\n>");  // code phrase, checked by C# side
      desiredSpeedR = 0;
      desiredSpeedL = 0;
      resetEma(RightMotorChannel);
      resetEma(LeftMotorChannel);
      EncodersReset();
      lastComm = millis();
    }
    else
    {
      //Serial.println("read: '" + str + "'");
      char chars[100];
      char* tokens[5];
      int channels[5];
      int pwms[5];
      str.toCharArray(chars, 100);
      int i = 0;

      // strtok replaces " " with '\0'
      const char sep[2] = " ";
      tokens[i] = strtok(chars, sep);
      if (tokens[i] == NULL)
      {
        return;
      }

      while (i < 5 && tokens[i] != NULL)
      {
        //Serial.print("token: ");
        //Serial.println(tokens[i]);
        tokens[++i] = strtok(NULL, sep);
      }

      //Serial.print(i);

      if (!strcmp(tokens[0], "cfg"))
      {
        //Serial.println("OK: cfg");
        COMM_SERIAL.print("\r\nACK\r\n>");
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "compass"))
      {
        //receiveI2cCompassPacket();
        COMM_SERIAL.print("\r\n");
        COMM_SERIAL.print(compassYaw);
        COMM_SERIAL.print("\r\n>");
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "psonar"))
      {
        //receiveI2cSonarPacket();
        COMM_SERIAL.print("\r\n");
        COMM_SERIAL.print(rangeFRcm);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(rangeFLcm);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(rangeBRcm);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(rangeBLcm);
        COMM_SERIAL.print("\r\n>");
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "sensor"))
      {
        //Serial.println("OK: sensor");
        int pin = atoi(tokens[1]);
        switch (pin)
        {
          case 5: // battery
            COMM_SERIAL.print("\r\n");
            //Serial.print(800);    // "800" here relates to battery voltage 11.72V = 3.91V per cell
            //Serial.print(analogRead(batteryInPin));
            COMM_SERIAL.print(800);
            COMM_SERIAL.print("\r\n>");
            break;

          default:
            COMM_SERIAL.print("\r\n");
            COMM_SERIAL.print(analogRead(pin));
            COMM_SERIAL.print("\r\n>");
            break;
        }
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "odomreset"))
      {
        odometry->Reset();
        X = Y = Theta = 0.0;
        Ldistance = Rdistance = 0;
        COMM_SERIAL.print("\r\nACK\r\n>");
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "odom"))
      {
        COMM_SERIAL.print("\r\n");
        COMM_SERIAL.print(Ldistance);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(Rdistance);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(X);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(Y);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(Theta);
        COMM_SERIAL.print("\r\n>");
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "gps"))
      {
        COMM_SERIAL.print("\r\n");
        if ((millis() - lastGpsData) > 2000 || gpsFix == 0)
        {
          COMM_SERIAL.print("0\r\n>");
        }
        else
        {
          COMM_SERIAL.print(gpsFix);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(gpsSat);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(millis() - lastGpsData);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(gpsHdop);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(longlat);
          COMM_SERIAL.print("\r\n>");
        }
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "srf04"))
      {
        //Serial.println("OK: srf04");
        COMM_SERIAL.print("\r\n"); 
        COMM_SERIAL.print(200 + j); 
        COMM_SERIAL.print("\r\n>");
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "getenc"))
      {
        //Serial.println("OK: getenc");
        switch (atoi(tokens[1]))
        {
          case 1:
            COMM_SERIAL.print("\r\n");
            COMM_SERIAL.print(Rdistance);
            COMM_SERIAL.print("\r\n>");
            break;
          case 2:
            COMM_SERIAL.print("\r\n");
            COMM_SERIAL.print(Ldistance);
            COMM_SERIAL.print("\r\n>");
            break;
          default:
            COMM_SERIAL.print("\r\n0\r\n>");
            break;
        }
        lastComm = millis();
      }
      else if (!strcmp(tokens[0], "pwm"))
      {
        // we don't want any errors on the wheels - so we protect this command with a checksum.
        // 'pwm 1:35 2:-65 c1234'
        //Serial.println("OK: pwm");
        int checksum = 0;
        for (int k = 1; k < i ; k++)
        {
          if (tokens[k][0] == 'c')
          {
            checksum += atoi(tokens[k] + 1);
          }
          else
          {
            channels[k - 1] = atoi(tokens[k]);
            checksum += channels[k - 1];
            pwms[k - 1] = atoi(tokens[k] + 2);
            checksum += pwms[k - 1];

            //          Serial.print("   ch=");
            //          Serial.print(channels[k-1]);
            //          Serial.print("   pwm=");
            //          Serial.println(pwms[k-1]);
          }
        }

        if (checksum == 0)
        {
          COMM_SERIAL.print("\r\nACK\r\n>");
          for (int k = 0; k < i - 1 ; k++)
          {
            // "pwm" comes in the range -100...100 - it has a meaning of "percent of max speed"
            switch (channels[k])
            {
              case 1:
                desiredSpeedR = pwms[k];
                break;
              case 2:
                desiredSpeedL = pwms[k];
                break;
            }
          }
          lastComm = millis();
          lastPwm = lastComm;
        }
        else
        {
          COMM_SERIAL.print("\r\nNAK\r\n>");
        }
      }
      else if (!strcmp(tokens[0], "vel"))
      {
        //Serial.println("OK: vel");
        COMM_SERIAL.print("\r\n0 0 \r\n>");
        lastComm = millis();
      }
      //else
      //{
      //  Serial.println("****************** unknown read: '" + str + "'");
      //}
    }
  }
  if (++j > 50)  // sensor "randomizer" for srf04 debugging
    j = 0;
}
#endif // COMM_ELEMENT

#ifdef COMM_SIMPLE
void control()
{
  if (COMM_SERIAL.available())
  {
    int val;
    val = COMM_SERIAL.read();
    switch (val)
    {
      case 'a':   // forward
        if (error_a < 25)
        {
          error_a += 3;
        }
        break;
      case 'b':   // back
        if (error_a > -25)
        {
          error_a -= 3;
        }
        break;
      case 's':   // stop
        error_a = 0;
        turn_flag = 0;
        break;
      case 'r':  // right, CW (increase Theta)
        turn_flag = 1;
        break;
      case 'l':  // left, CCW (decrease Theta)
        turn_flag = -1;
        break;
      case 'x':  // reset odometry
        Ldistance = 0;
        Rdistance = 0;
        odometry->Reset();
        break;
      case 'z':  // reset position
        X = 0.0;
        Y = 0.0;
        Theta = 0.0;
        break;

      default:
        break;

    }
  }
}
#endif // COMM_SIMPLE

#ifdef COMM_BALANCER

#define CHANNEL_IDENTIFY         0    // send back device ident string
#define CHANNEL_DRIVE            1    // drive control

#define CMD_IDENTIFY             127
#define CMD_SET_VALUES           1    // any channel

#define CMD_DRIVE_STOP           10
#define CMD_DRIVE_MOVE           11

#define USE_FORWARD_FACTOR

void readCommCommand()
{
  if(COMM_SERIAL.available())  // takes 8us if false
  {
    int val;
    val=COMM_SERIAL.read();
    switch (val)
    {
    case 'a':             // forward
        //shortBuzz();
#ifdef USE_FORWARD_FACTOR        
        if(forward_factor < 40.0)
        {
          forward_factor += 4.0;
        }
#else // USE_FORWARD_FACTOR        
        if(error_a > -8)         
        { 
          error_a -= 1.0;
        }
#endif // USE_FORWARD_FACTOR        
        break;
          
    case 'b':             // backwards
#ifdef USE_FORWARD_FACTOR        
        if(forward_factor > -10.0)
        {
          forward_factor -= 1.0;
        }
#else // USE_FORWARD_FACTOR        
        if(error_a < 8)
        { 
          error_a += 1.0;
        }
#endif // USE_FORWARD_FACTOR        
        break;
          
    case 's':             // stop
        error_a = 0.0;
        turn_flag = 0.0;
        forward_factor = 0.0;
        break;
      
    case 'r':              // right
        turn_flag = 1.0;   // positive angle
        break;
      
    case 'l':              // left
        turn_flag = -1.0;  // negative angle
        break;
      
    case 'z':              // reset odometry
        X = 0.0;
        Y = 0.0;
        Theta = 0.0;
        oLdistance = 0;
        oRdistance = 0;
        break;

    case 'o':              // read odometry
        COMM_SERIAL.print("o");
        COMM_SERIAL.print(X);
        COMM_SERIAL.print(",");
        COMM_SERIAL.print(Y);
        COMM_SERIAL.print(",");
        COMM_SERIAL.print(Theta);
        break;
      
    case '*':              // ToArduino packet
        {
          //shortBuzz();

          // look for the next 3+ valid integers in the incoming serial stream:
          int checksum = 0;
          int channel = COMM_SERIAL.parseInt();      checksum += channel;
          int cmd = COMM_SERIAL.parseInt();          checksum += cmd;
          int command = cmd & 0x7F;
          int cmdValue;    // used for single-value commands to save cmdValues allocation
          
          // WARNING: the size of cmdValues can cause RAM overfill in Atmel 328 based Arduinos, it is only 2KB there.
          int *cmdValues = NULL;
          
          int nValues = 0;
          boolean doRepeat = (cmd & 0x80) != 0;
      
          // we can have 0 to 10 values, depending on the command.   
          switch(command)
          {
            case CMD_IDENTIFY:
              // respond to "*0 127 -127"
              break;
              
            case CMD_DRIVE_MOVE:
            case CMD_SET_VALUES:
              //shortBuzz();
              nValues = cmd >> 8;
              cmdValues = (int*)malloc((size_t)(nValues * sizeof(int)));    // heap allocation, competes with other objects for 2KB RAM
              for(int i=0; i < nValues ;i++)
              {
                int cmdValue = COMM_SERIAL.parseInt();
                cmdValues[i] = cmdValue;
                checksum += cmdValue;
              }
              break;
      
            case CMD_DRIVE_STOP:
            default:
              break;
          }
          int checksumPkg = COMM_SERIAL.parseInt();  // must meet a trailing space, otherwise 1 second delay will occur
          checksum += checksumPkg;
      
          if(checksum != 0)
          {
            // bad transmission
            //digitalWrite(led, 1);    // indicates trouble
            shortBuzz();
          }
          else
          {      
            //shortBuzz();
            //digitalWrite(led, 0);  // indicates normal operation
            controlAction(channel, command, nValues, cmdValues);
            //shortBuzz();
          }
          
          if(cmdValues != NULL)
          {
            free(cmdValues);
          }
        }
        break;
      
    default:
        break;      
    }
  }
}

void controlAction(int channel, int command, int nValues, int* cmdValues)
{
  /*
    Serial.print("controlAction: ");
    Serial.print(channel);
    Serial.print(" ");
    Serial.print(command);
    Serial.print(" ");
    Serial.print(nValues);
  */
    switch (channel)
    {
      case CHANNEL_IDENTIFY:
          COMM_SERIAL.println("*ARD COMM HEAD V1.0*");
          break;
          
      case CHANNEL_DRIVE:      // drive control
          {
            switch(command)
            {
              case CMD_DRIVE_STOP:
                error_a=0;
                turn_flag=0;
                break;
                
              case CMD_DRIVE_MOVE:
                {
                  //shortBuzz();
                  //Serial.println("*ARD COMM OK");
                  double speed = (double)constrain(cmdValues[1], -700, 700);
                  double turn = (double)constrain(cmdValues[0], -250, 250);
/*
    Serial.print(":");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(turn);
*/                  
                  //if(speed * speed + turn * turn > 65000.0f)
                  {
                    /*
                    if(error_a > -4.0 && error_a < 4.0)         
                    { 
                      error_a += speed / 30.0;
                    }
                    */
                    
                    error_a = speed / 100.0;
                    
                    turn_flag = turn / 200.0;
                  }
                }
                break;
                
              default:
                {
                  //shortBuzz();
                  Serial.print("? CHANNEL_DRIVE: bad command ");
                  Serial.println(command);
                  int j=0;
                  for(int i=0; i < nValues ;i++, j+=3)
                  {
                    int val = cmdValues[j];
                  }
                }
                break;
            }
          }
          break;

      default:            
          break;
    }
    //Serial.println();
}
#endif // COMM_BALANCER
