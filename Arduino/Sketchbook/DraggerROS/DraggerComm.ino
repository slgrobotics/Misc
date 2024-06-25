// **************************
// Remote control code
// **************************

//#define ARTICUBOTS_USE_SERVOS
#define ARTICUBOTS_USE_BASE

// could use Serial3 for BLE-LINK on pins 14,15 ?
#define COMM_SERIAL Serial
#define BAUDRATE_ARTICUBOTS 115200
#define DO_RESPOND_OK

// ==== To be moved to I2C.ino: =====

void receiveI2cSonarPacket() {
}

double lastCompassYaw = 0.0;

void receiveI2cCompassPacket() {
}

// ==================================

void InitSerial() {
  //Serial1.begin(57600);  // connect GPS Leonardo shield uplink (pins 3 and 8) to pins 19 (RX) and 18 (TX)

  COMM_SERIAL.begin(BAUDRATE_ARTICUBOTS);  // start serial for USB to Raspberry Pi
  //COMM_SERIAL.setTimeout(1);
}

/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#define ANALOG_READ 'a'
#define GET_BAUDRATE 'b'
#define PIN_MODE 'c'
#define DIGITAL_READ 'd'
#define READ_ENCODERS 'e'
#define READ_GPS 'g'
#define READ_HEALTH 'h'
#define MOTOR_SPEEDS 'm'
#define MOTOR_RAW_PWM 'o'
#define SONAR_PING 'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE 's'
#define SERVO_READ 't'
#define UPDATE_PID 'u'
#define DIGITAL_WRITE 'w'
#define ANALOG_WRITE 'x'

#define LEFT 0
#define RIGHT 1

/* Variable initialization */

unsigned char moving = 0;  // is the base in motion?

// Variable to hold the current single-character command
char cmd;
String cmd_in;

// The arguments converted to integers
long arg1;
long arg2;

char pid_args_str[64];
char out_buf[128];

void respond() {
  COMM_SERIAL.print(out_buf);
  COMM_SERIAL.flush();  // wait for the output stream to finish transmission
}

void respond_OK(char rq_cmd) {
#ifdef DO_RESPOND_OK
  sprintf(out_buf, "%c OK\r", rq_cmd);
  respond();
#endif
}

void respond_ERROR(String rq_cmd_in) {
  sprintf(out_buf, "? Error: '%s'\r", rq_cmd_in);
  respond();
}

void readCommCommand() {
  unsigned long now = millis();
  if (COMM_SERIAL.available()) {
    String strs[20];
    int StringCount = 0;

    cmd_in = COMM_SERIAL.readStringUntil('\r');  // Command is terminated with a CR
    cmd_in.trim();
    //Serial.println(cmd_in);

    if (cmd_in.length() == 0) {
      respond_OK('*');
      return;  // ignore empty strings
    }

    // Split the string into substrings:
    while (cmd_in.length() > 0) {
      int index = cmd_in.indexOf(' ');
      if (index == -1)  // No space found
      {
        strs[StringCount++] = cmd_in;
        break;
      } else {
        if (StringCount == 0 && index != 1) {  // expect only single character commands
          //COMM_SERIAL.println("Error: not a command");
          respond_ERROR(cmd_in);
          return;
        }
        strs[StringCount++] = cmd_in.substring(0, index);
        cmd_in = cmd_in.substring(index + 1);
      }
    }

    cmd = strs[0].charAt(0);
    out_buf[0] = cmd;
    out_buf[1] = ' ';
    out_buf[2] = '\0';

    //Serial.println(StringCount);

    switch (StringCount) {
      case 1:  // just the command, like "e"
        runCommand();
        resetCommand();
        break;

      case 2:  // command and one composite argument, possibly "u 30:20:0:100"
        strs[1].toCharArray(pid_args_str, sizeof(pid_args_str));
        runCommand();
        resetCommand();
        break;

      case 3:  // command and two arguments, like "m 100 200"
        arg1 = strs[1].toInt();
        arg2 = strs[2].toInt();
        runCommand();
        resetCommand();
        break;

      default:
        // something wrong, keep looking for a valid string
        //COMM_SERIAL.println("Error: wrong number of arguments");
        respond_ERROR(cmd_in);
        return;
    }

    /*    
    // Show the resulting substrings
    for (int i = 0; i < StringCount; i++)
    {
      Serial.print(i);
      Serial.print(": \"");
      Serial.print(strs[i]);
      Serial.println("\"");
    }
    */
  }
}

/* Clear the current command parameters */
void resetCommand() {
  cmd = '\0';
  pid_args_str[0] = '\0';
  arg1 = 0;
  arg2 = 0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {

  //Serial.print("runCommand: ");
  //Serial.println(cmd);

  lastCommMs = millis();

  switch (cmd) {
    case GET_BAUDRATE:
      sprintf(&out_buf[2], "%ld\r", BAUDRATE_ARTICUBOTS);
      respond();
      break;
    case ANALOG_READ:
      sprintf(&out_buf[2], "%d\r", analogRead(arg1));
      respond();
      break;
    case DIGITAL_READ:
      sprintf(&out_buf[2], "%d\r", digitalReadFast(arg1));
      respond();
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      respond_OK(cmd);
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      respond_OK(cmd);
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      respond_OK(cmd);
      break;
    case SONAR_PING:
      receiveI2cSonarPacket();
      // reported 255 if out of range, otherwise range in centimeters:
      sprintf(&out_buf[2], "%d %d %d %d\r", rangeFRcm, rangeFLcm, rangeBRcm, rangeBLcm);
      respond();
      break;
#ifdef ARTICUBOTS_USE_SERVOS
    case SERVO_WRITE:
      servos[arg1].setTargetPosition(arg2);
      respond_OK(cmd);
      break;
    case SERVO_READ:
      sprintf(&out_buf[2], "%d\r", servos[arg1].getServo().read());
      respond();
      break;
#endif  // ARTICUBOTS_USE_SERVOS

#ifdef ARTICUBOTS_USE_BASE
    case READ_ENCODERS:
      sprintf(&out_buf[2], "%ld %ld\r", readEncoder(LEFT), readEncoder(RIGHT));
      respond();
      break;
    case RESET_ENCODERS:
      EncodersReset();
      resetPID();
      respond_OK(cmd);
      break;
    case READ_GPS:
      receiveI2cCompassPacket();
      // Serial.print("longlat: ");
      // Serial.print(longlat);
      // Serial.print("   lastCompassYaw: ");
      // Serial.println(lastCompassYaw);
      {
        int gpsFix = 0;
        int gpsSat = 0;
        int gpsHdop = 0;
        String longlat = "";
        unsigned long lastGpsDataMs = 0;
        sprintf(&out_buf[2], "%d %d %d %ld %s %d\r", gpsFix, gpsSat, gpsHdop, (long)(millis() - lastGpsDataMs), longlat.c_str(), (int)round(lastCompassYaw));
      }
      respond();
      break;
    case READ_HEALTH:
      {
        //long bat = analogRead(batteryInPin); // "800" here relates to battery voltage 11.72V = 3.90V per cell
        //bat = bat * 39 / 8; // millivolts, returns "4031" for 4.031V per cell
        long bat = 4031;
        int current_ma = 1234;  // TODO: need to measure it
        sprintf(&out_buf[2], "%ld %d %d\r", bat, current_ma, freeRam());
        respond();
      }
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommandMs = millis();
      //Serial.print(arg1);
      //Serial.print(" --- ");
      //Serial.println(arg2);
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      } else moving = 1;

      // desiredSpeed* is in the range -100...100 - it has a meaning of "percent of max possible speed".
      // For Dragger full wheel rotation takes 2.2 seconds. So, with R_wheel=0.192m dist=1.206m max speed is:
      //   - 0.548 m/sec (1.23 mph, 1.97 km/h)
      //   - 2.86 rad/sec
      //   - 6241 encoder ticks/sec
      desiredSpeedL = arg1;
      desiredSpeedR = arg2;
      respond_OK(cmd);
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommandMs = millis();
      resetPID();
      moving = 0;  // Sneaky way to temporarily disable the PID
      setMotorSpeeds(arg1, arg2);
      respond_OK(cmd);
      break;
    case UPDATE_PID:
      {
        char *p = pid_args_str;
        char *str;
        int i = 0;
        int pid_args[4];

        while ((str = strtok_r(p, ":", &p)) != NULL) {
          pid_args[i] = atoi(str);
          i++;
        }

        /*
      Serial.print(pid_args[0]); Serial.print("|");
      Serial.print(pid_args[1]); Serial.print("|");
      Serial.print(pid_args[2]); Serial.print("|");
      Serial.println(pid_args[3]);
      /*
      Kp = pid_args[0];
      Kd = pid_args[1];
      Ki = pid_args[2];
      Ko = pid_args[3];
      */
      }
      respond_OK(cmd);
      break;
#endif  // ARTICUBOTS_USE_BASE

    default:
      respond_ERROR(cmd_in);
      break;
  }
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return Ldistance;
  else return Rdistance;
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
