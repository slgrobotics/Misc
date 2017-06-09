#ifdef SERIAL_MY

void InitializeSerial()
{
}

bool CheckSerial()
{  
  // if there's any serial available, read it:
  if (Serial.available() > 0)
  {
    // look for the asterisk *. That's the beginning of your sentence:
    if (Serial.read() != '*')
    {
      return true;  // we have some garbage to read through, come back soon
    }

    // look for the next four valid integers in the incoming serial stream:
    int channel = Serial.parseInt(); 
    int cmd = Serial.parseInt(); 
    int cmdValue = Serial.parseInt(); 
    int checksum = Serial.parseInt(); 
    int mapedServoValue;

    if(channel + cmd + cmdValue + checksum != 0)
    {
      // bad transmission
      digitalWrite(led, 1);    // indicates trouble
      return true;  // we may have some garbage to read through
    }

    digitalWrite(led, 0);  // indicates normal operation

    //os.spinlock_acquire(&myLock); 
    cli();    
    switch (channel)
    {
    case 1:  // Head platform pan via H-Bridge
      HeadPanSetpoint = myMap((double)cmdValue, 800, 2200, -90, 90);
      break;
    case 2:  // Lights power on H-Bridge
      lightsPowerMks = cmdValue;
      setLightsPower();
      break;
    case 3:  // direct servo control
      GimbalPitchSetpointMks = cmdValue;
      break;
    }
    sei();
    //os.spinlock_release(&myLock);        
    return true;  // we may have some more to read through
  }
  return false;  // idling, can sleep
}

void  ReportState()
{
  // print variables for debugging and communication to PC:
  Serial.print("*");
  Serial.print(_lightsPowerMks);
  Serial.print(" ");
  Serial.print(_HeadPanSetpoint);
  Serial.print(" ");
  Serial.print(_HeadPanMeasured);
  Serial.print(" ");
  Serial.println(_HeadPanMotorChannelPower);
}

#endif // SERIAL_MY

#ifdef SERIAL_ROBOREALM

// see http://www.roborealm.com/help/Sparkfun_Arduino.php   http://www.roborealm.com/help/RoboRealm_Arduino/RoboRealm_Arduino.pde
// see C:\Projects\Arduino\Sketchbook\RoboRealm_Arduino

unsigned int g_crc;
unsigned int g_command;
unsigned int g_channel;
unsigned int g_value;
unsigned int g_valueLow;
unsigned int g_valueHigh;
unsigned int g_streamDigital;
unsigned int g_streamAnalog;
unsigned int g_lastDigital;
unsigned int g_lastAnalog[8];
unsigned int g_heartBeat=0;
unsigned int g_defaultServo[14];

#define ARDUINO_GET_ID 0
#define ARDUINO_SET_SERVO 1
#define ARDUINO_SET_DIGITAL_STREAM 2
#define ARDUINO_SET_DIGITAL_HIGH 3
#define ARDUINO_SET_DIGITAL_LOW 4
#define ARDUINO_SET_ANALOG_STREAM 5
#define ARDUINO_DIGITAL_STREAM 6
#define ARDUINO_ANALOG_STREAM 7
#define ARDUINO_SET_ANALOG 8
#define ARDUINO_SET_SERVO_DEFAULT 9
#define ARDUINO_HEARTBEAT 10

void InitializeSerial()
{
  int i;
  //for (i=2;i<14;i++) pinModes[i]=-1;
  g_streamDigital=0;
  g_streamAnalog=0;
  g_lastDigital=-1;
  for (i=0;i<8;i++) g_lastAnalog[i]=-1;
  for (i=0;i<12;i++) g_defaultServo[i]=1500;
}

bool CheckSerial()
{  
  // if there's any serial available, read it:
  if (Serial.available() > 0)
  {
    readPacket();

    g_heartBeat=0;

    switch (g_command)
    {
      // init
    case  ARDUINO_GET_ID:
      InitializeSerial();
      Serial.print("ARDU");
      break;
      // servo
    case  ARDUINO_SET_SERVO:
      if ((g_channel>=3)&&(g_channel<=11))
      {
        if (readValuePacket())
        {
//          if (pinModes[g_channel]!=1)
//          {
//            servos[g_channel].attach(g_channel);
//            pinModes[g_channel]=1;
//          }
//          servos[g_channel].writeMicroseconds(g_value);

          // ------------------------------------
          cli();  
          // servo channels are 3, 5, 6, 9, 10, 11 in RoboRealm
          switch (g_channel)
          {
          case 3:  // Head platform pan via H-Bridge
            HeadPanSetpoint = myMap((double)g_value, 800, 2200, -90, 90);
            break;
          case 5:  // direct servo control
            GimbalPitchSetpointMks = g_value;
            break;
          case 6:  // Lights power on H-Bridge
            lightsPowerMks = g_value;
            setLightsPower();
            break;
          }
          sei();
          // ------------------------------------
          
          writeValuePacket(g_value);
        }
      }
      break;
      //digital stream
    case  ARDUINO_SET_DIGITAL_STREAM:
      if (readValuePacket())
      {
        g_streamDigital = g_value;
        writeValuePacket(g_value);
        g_lastDigital=-1;
      }
      break;
      //set digital high
    case  ARDUINO_SET_DIGITAL_HIGH:
      if ((g_channel>=2)&&(g_channel<14))
      {
//        if (pinModes[g_channel]!=2)
//        {
//          if (pinModes[g_channel]==1)
//            servos[g_channel].detach();
//
//          pinMode(g_channel, OUTPUT);
//          pinModes[g_channel]=2;
//          if (g_streamDigital&(1<<g_channel))
//            g_streamDigital^=1<<g_channel;
//        }
//
//        digitalWrite(g_channel, HIGH);
        writePacket();
      }
      break;
      //set digital low
    case  ARDUINO_SET_DIGITAL_LOW:
      if ((g_channel>=2)&&(g_channel<14))
      {
//        if (pinModes[g_channel]!=2)
//        {
//          if (pinModes[g_channel]==1)
//            servos[g_channel].detach();
//
//          pinMode(g_channel, OUTPUT);
//          pinModes[g_channel]=2;
//
//          if (g_streamDigital&(1<<g_channel))
//            g_streamDigital^=1<<g_channel;
//        }
//        digitalWrite(g_channel, LOW);
        writePacket();
      }
      break;
      //analog stream
    case  ARDUINO_SET_ANALOG_STREAM:
      if (readValuePacket())
      {
        g_streamAnalog = g_value;
        writeValuePacket(g_value);
        for (g_channel=0;g_channel<8;g_channel++) g_lastAnalog[g_channel]=-1;
      }
      break;
    case  ARDUINO_SET_ANALOG:
      if (readValuePacket())
      {
        if ((g_channel>=3)&&(g_channel<=11))
        {
//          if (pinModes[g_channel]!=2)
//          {
//            if (pinModes[g_channel]==1)
//              servos[g_channel].detach();
//
//            pinMode(g_channel, OUTPUT);
//            pinModes[g_channel]=2;
//          }
//          analogWrite(g_channel, g_value);
          writeValuePacket(g_value);
        }
      }
      break;
    case  ARDUINO_SET_SERVO_DEFAULT:
      if ((g_channel>=3)&&(g_channel<=11))
      {
        if (readValuePacket())
        {
          g_defaultServo[g_channel] = g_value;
          writeValuePacket(g_value);
        }
      }
      break;
    case ARDUINO_HEARTBEAT:
      break;
    }
  }      // end if (Serial.available() > 0)

  if ((g_heartBeat++) > 25000)
  {
    // no serial commands for a while... bad sign
    
//    int i;
//    for (i=3; i < 12 ;i++)
//    {
//      if (pinModes[i]==1) 
//        servos[i].writeMicroseconds(g_defaultServo[i]);
//      else
//        if (pinModes[i]==2) 
//          analogWrite(i, 0);
//    }

    // assume safe posture:
//    HeadPanSetpoint = 0.0;
//    lightsPowerMks = 800;
//    setLightsPower();
//    GimbalPitchSetpointMks = 1500;

  }

//  for (g_channel=0;g_channel<8;g_channel++)
//  {
//    if (g_streamAnalog&(1<<g_channel))
//    {
//      g_value = analogRead(g_channel);
//      // only send g_value if it has changed
//      if (g_value!=g_lastAnalog[g_channel])
//      {
//        g_command = ARDUINO_ANALOG_STREAM;
//        writeValuePacket(g_value);
//
//        g_lastAnalog[g_channel]=g_value;
//      }
//    }
//  }

//  if (g_streamDigital)
//  {
//    g_value=0;
//    for (g_channel=2;g_channel<14;g_channel++)
//    {
//      if (g_streamDigital&(1<<g_channel))
//      {
//        if (pinModes[g_channel]!=3)
//        {
//          if (pinModes[g_channel]==1)
//            servos[g_channel].detach();
//
//          pinMode(g_channel, INPUT);
//          pinModes[g_channel]=3;
//          // pullup
//          digitalWrite(g_channel, HIGH); 
//        }
//
//        g_value |= digitalRead(g_channel)<<g_channel;
//      }
//    }
//
//    // only send g_value if it has changed
//    if (g_value!=g_lastDigital)
//    {    
//      g_command = ARDUINO_DIGITAL_STREAM;
//      writeValuePacket(g_value);
//
//      g_lastDigital=g_value;
//    }
//    return true;
//  }

  return false;
}

void writePacket()
{
  unsigned char buffer[2];  
  buffer[0]=g_command|128;
  buffer[1]=g_channel;
  Serial.write(buffer, 2);
}

void writeValuePacket(int g_value)
{
  unsigned char buffer[5];  

  buffer[0]=g_command|128;
  buffer[1]=g_channel;
  buffer[2]=g_value&127;
  buffer[3]=(g_value>>7)&127;
  buffer[4]=(buffer[0]^buffer[1]^buffer[2]^buffer[3])&127;

  Serial.write(buffer, 5);
}

void readPacket()
{
  // get header byte
  // 128 (bit 8) flag indicates a new g_command packet .. that 
  // means the g_value bytes can never have 128 set!
  // next byte is the g_command 0-8
  // next byte is the g_channel 0-16

  do
  {
    while (Serial.available() <= 0) continue; 
    g_command = Serial.read();
  }
  while ((g_command&128)==0);

  g_command^=128;

  while (Serial.available() <= 0) continue; 
  g_channel = Serial.read();
}

int readValuePacket()
{
  unsigned int g_valueLow;
  unsigned int g_valueHigh;

  // wait for g_value low byte    
  while (Serial.available() <= 0) continue; 
  g_valueLow = Serial.read();
  if (g_valueLow&128) return 0;

  // wait for g_value high byte    
  while (Serial.available() <= 0) continue; 
  g_valueHigh = Serial.read();
  if (g_valueHigh&128) return 0;

  // wait for g_crc byte    
  while (Serial.available() <= 0) continue; 
  g_crc = Serial.read();
  if (g_crc&128) return 0;

  if (g_crc!=(((128|g_command)^g_channel^g_valueLow^g_valueHigh)&127)) return 0;

  g_value = g_valueLow|(g_valueHigh<<7);

  return 1;
}

void  ReportState()
{
}

#endif // SERIAL_ROBOREALM

