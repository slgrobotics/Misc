// **************************
// Remote control code
// **************************

#define CHANNEL_IDENTIFY         0    // send back device ident string
#define CHANNEL_DRIVE            1    // drive control

#define CMD_IDENTIFY             127
#define CMD_SET_VALUES           1    // any channel

#define CMD_DRIVE_STOP           10
#define CMD_DRIVE_MOVE           11


void control()
{
  if(Serial.available())  // takes 8us if false
  {
    int val;
    val=Serial.read();
    switch (val)
    {
    case 'a':             // forward
        //shortBuzz();
        if(angle_setpoint_remote > -8)         
        { 
          angle_setpoint_remote -= 1.0;
        }
        break;
          
    case 'b':             // backwards
        if(angle_setpoint_remote < 8)
        { 
          angle_setpoint_remote += 1.0;
        }
        break;
          
    case 's':             // stop
        angle_setpoint_remote=0;
        turn_flag=0;
        break;
      
    case 'r':             // right
        turn_flag=1;      // positive angle
        break;
      
    case 'l':            // left
        turn_flag=-1;    // negative angle
        break;
      
    case '*':            // ToArduino packet
        {
          //shortBuzz();

          // look for the next 3+ valid integers in the incoming serial stream:
          int checksum = 0;
          int channel = Serial.parseInt();      checksum += channel;
          int cmd = Serial.parseInt();          checksum += cmd;
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
                int cmdValue = Serial.parseInt();
                cmdValues[i] = cmdValue;
                checksum += cmdValue;
              }
              break;
      
            case CMD_DRIVE_STOP:
            default:
              break;
          }
          int checksumPkg = Serial.parseInt();  // must meet a trailing space, otherwise 1 second delay will occur
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
          Serial.println("*ARD COMM HEAD V1.0*");
          break;
          
      case CHANNEL_DRIVE:      // drive control
          {
            switch(command)
            {
              case CMD_DRIVE_STOP:
                angle_setpoint_remote=0;
                turn_flag=0;
                break;
                
              case CMD_DRIVE_MOVE:
                {
                  //shortBuzz();
                  //Serial.println("*ARD COMM OK");
                  float speed = (float)constrain(cmdValues[1], -700, 700);
                  float turn = (float)constrain(cmdValues[0], -250, 250);
/*
    Serial.print(":");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(turn);
*/                  
                  //if(speed * speed + turn * turn > 65000.0f)
                  {
                    /*
                    if(angle_setpoint_remote > -4.0 && angle_setpoint_remote < 4.0)         
                    { 
                      angle_setpoint_remote += speed / 30.0;
                    }
                    */
                    
                    angle_setpoint_remote = speed / 100.0;
                    
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
