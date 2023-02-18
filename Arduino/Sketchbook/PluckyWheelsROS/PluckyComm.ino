// **************************
// Remote control code
// **************************

//#define COMM_SIMPLE
//#define COMM_ELEMENT
#define COMM_ARTICUBOTS

//#define ARTICUBOTS_USE_SERVOS
#define ARTICUBOTS_USE_BASE

#define COMM_SERIAL Serial
#define BAUDRATE_ARTICUBOTS 115200

void InitSerial()
{
  Serial1.begin(57600); // connect GPS Leonardo shield uplink (pins 3 and 8) to pins 19 (RX) and 18 (TX)

#ifdef COMM_ARTICUBOTS
  COMM_SERIAL.begin(BAUDRATE_ARTICUBOTS);   // start serial for USB to Raspberry Pi
#else
  //Serial.begin(115200);     // start serial for USB
  COMM_SERIAL.begin(115200);  // start serial for Funduino Bluetooth XBee
#endif  
}

#ifdef COMM_ARTICUBOTS

/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define SONAR_PING     'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'

#define LEFT            0
#define RIGHT           1

/* Variable initialization */

unsigned char moving = 0; // is the base in motion?

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

void readCommCommand()
{
  while (COMM_SERIAL.available())
  {
    // Read the next character
    chr = Serial.read();
    lastCommMs = millis();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = '\0';
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE_ARTICUBOTS);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case SONAR_PING:
    Serial.println(Ping(arg1));
    break;
#ifdef ARTICUBOTS_USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef ARTICUBOTS_USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommandMs = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    //leftPID.TargetTicksPerFrame = arg1;
    desiredSpeedL = arg1;
    //rightPID.TargetTicksPerFrame = arg2;
    desiredSpeedR = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommandMs = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != NULL) {
       pid_args[i] = atoi(str);
       i++;
    }
    /*    
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    */
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

long Ping(int pin) {
  return 100; // cm
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return Ldistance;
  else return Rdistance;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT){
    Ldistance = 0;
    LdistancePrev = 0;
    return;
  } else { 
    Rdistance = 0;
    RdistancePrev = 0;
    return;
  }
}

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

void resetPID() {
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) desiredSpeedL = spd;
  else desiredSpeedR = spd;
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}

#endif // COMM_ARTICUBOTS

#ifdef COMM_ELEMENT

int j = 0; // sensor "randomizer" for srf04 sonar debugging/faking

void readCommCommand()
{
  while (COMM_SERIAL.available())
  {
    String str = COMM_SERIAL.readStringUntil('\r');
    if (str == "reset")
    {
      COMM_SERIAL.print("Arduino firmware Plucky Wheels - COMM_ELEMENT\r\n>");  // code phrase, checked by C# side
      desiredSpeedR = 0;
      desiredSpeedL = 0;
      resetEma(RightMotorChannel);
      resetEma(LeftMotorChannel);
      EncodersReset();
      lastCommMs = millis();
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
        return ret;
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
        lastCommMs = millis();
      }
      else if (!strcmp(tokens[0], "compass"))
      {
        receiveI2cCompassPacket();
        COMM_SERIAL.print("\r\n");
        COMM_SERIAL.print(compassYaw);
        COMM_SERIAL.print("\r\n>");
        lastCommMs = millis();
      }
      else if (!strcmp(tokens[0], "psonar"))
      {
        receiveI2cSonarPacket();
        COMM_SERIAL.print("\r\n");
        COMM_SERIAL.print(rangeFRcm);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(rangeFLcm);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(rangeBRcm);
        COMM_SERIAL.print(" ");
        COMM_SERIAL.print(rangeBLcm);
        COMM_SERIAL.print("\r\n>");
        lastCommMs = millis();
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
            COMM_SERIAL.print(analogRead(batteryInPin));
            COMM_SERIAL.print("\r\n>");
            break;

          default:
            COMM_SERIAL.print("\r\n");
            COMM_SERIAL.print(analogRead(pin));
            COMM_SERIAL.print("\r\n>");
            break;
        }
        lastCommMs = millis();
      }
      else if (!strcmp(tokens[0], "odomreset"))
      {
        odometry->Reset();
        X = Y = Theta = 0.0;
        Ldistance = Rdistance = 0;
        COMM_SERIAL.print("\r\nACK\r\n>");
        lastCommMs = millis();
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
        lastCommMs = millis();
      }
      else if (!strcmp(tokens[0], "gps"))
      {
        COMM_SERIAL.print("\r\n");
        if ((millis() - lastGpsDataMs) > 2000 || gpsFix == 0)
        {
          COMM_SERIAL.print("0\r\n>");
        }
        else
        {
          COMM_SERIAL.print(gpsFix);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(gpsSat);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(millis() - lastGpsDataMs);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(gpsHdop);
          COMM_SERIAL.print(" ");
          COMM_SERIAL.print(longlat);
          COMM_SERIAL.print("\r\n>");
        }
        lastCommMs = millis();
      }
      else if (!strcmp(tokens[0], "srf04"))
      {
        //Serial.println("OK: srf04");
        COMM_SERIAL.print("\r\n"); 
        COMM_SERIAL.print(200 + j); 
        COMM_SERIAL.print("\r\n>");
        lastCommMs = millis();
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
        lastCommMs = millis();
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
          lastCommMs = millis();
          lastMotorCommandMs = lastCommMs;
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
        lastCommMs = millis();
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
void readCommCommand()
{
  if (COMM_SERIAL.available())
  {
    int val;
    val = COMM_SERIAL.read();
    lastCommMs = millis();
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

// see GPSKitchenSink.ino - it is connected to Serial1
void readGpsUplink()
{
  // if there's any serial available from GPS uplink, read it:
  if (Serial1.available() > 0)
  {
    // expect
    //  FIX 1
    //  SAT 6
    //  HDP 124
    //  LOC 33.575966833 -117.662913833

    String str = Serial1.readStringUntil('\n');
    //Serial.println("read: '" + str + "'");
    str.toCharArray(gpsChars, 100);

    if (strncmp(gpsChars, "FIX", 3) == 0)
    {
      gpsFix = (int)atol(gpsChars + 4);
      //Serial.print("gpsFix: ");
      //Serial.println(gpsFix);
    }

    if (strncmp(gpsChars, "SAT", 3) == 0)
    {
      gpsSat = (int)atol(gpsChars + 4);
      //Serial.print("gpsSat: ");
      //Serial.println(gpsSat);
    }

    if (strncmp(gpsChars, "HDP", 3) == 0)
    {
      gpsHdop = (int)atol(gpsChars + 4);
      //Serial.print("gpsHdop: ");
      //Serial.println(gpsHdop);
    }

    if (strncmp(gpsChars, "LOC", 3) == 0)
    {
      longlat = str.substring(4);
      //Serial.print("longlat: ");
      //Serial.println(longlat);
    }

    lastGpsDataMs = millis();
  }
}
