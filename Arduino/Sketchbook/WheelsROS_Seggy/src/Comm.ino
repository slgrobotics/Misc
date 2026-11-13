// **************************
// Remote control code
//
// Implements simple strings-based protocol between the ROS2 diffdrive_arduino package (ros2_control interfaces) and RPi Pico
//
// See https://github.com/slgrobotics/diffdrive_arduino   - ROS2 driver for these wheels
//
// Credits: Original code by Articulated Robotics (Josh Newans):
// https://articulatedrobotics.xyz/category/build-a-mobile-robot-with-ros
// 
// **************************

//#define ARTICUBOTS_USE_SERVOS
#define ARTICUBOTS_USE_BASE

#define COMM_SERIAL Serial
#define BAUDRATE_ARTICUBOTS 115200
#define DO_RESPOND_OK

void InitSerial() {

  COMM_SERIAL.begin(BAUDRATE_ARTICUBOTS);   // start serial for USB to Raspberry Pi /dev/ttyACM0
  //COMM_SERIAL.setTimeout(1);
}

/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define READ_HEALTH    'h'
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

// Variable to hold the current single-character command
char cmd;
String cmd_in;

// The arguments converted to integers
long arg1;
long arg2;

char out_buf[128];

void respond() {
  COMM_SERIAL.print(out_buf);
  COMM_SERIAL.flush(); // wait for the output stream to finish transmission
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

  if (COMM_SERIAL.available()) {
    String strs[20];
    int StringCount = 0;

    cmd_in = COMM_SERIAL.readStringUntil('\r'); // Command is terminated with a CR
    cmd_in.trim();
    //Serial.println(cmd_in);

    if (cmd_in.length() == 0) {
      respond_OK('*');
      return;    // ignore empty strings
    }

    // Split the string into substrings:
    while (cmd_in.length() > 0) {
      int index = cmd_in.indexOf(' ');
      if (index == -1) // No space found
      {
        strs[StringCount++] = cmd_in;
        break;
      } else {
        if (StringCount == 0 && index != 1) { // expect only single character commands
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
      case 1:   // just the command, like "e"
        runCommand();
        resetCommand();
        break;

      case 2:  // command and one simple or composite argument
        arg1 = strs[1].toInt(); // for commands like "a 4"
        runCommand();
        resetCommand();
        break;

      case 3:   // command and two arguments, like "m 100 200"
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
      sprintf(&out_buf[2], "%d\r", digitalRead(arg1));
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
      //receiveI2cSonarPacket();
      // reported 255 if out of range, otherwise range in centimeters:
      sprintf(&out_buf[2], "%d %d %d %d\r", 0, 0, 0, 0);
      respond();
      break;
    case SERVO_WRITE:
#ifdef ARTICUBOTS_USE_SERVOS
      servos[arg1].setTargetPosition(arg2);
#endif // ARTICUBOTS_USE_SERVOS
      respond_OK(cmd);
      break;
    case SERVO_READ:
#ifdef ARTICUBOTS_USE_SERVOS
      sprintf(&out_buf[2], "%d\r", servos[arg1].getServo().read());
#else  // ARTICUBOTS_USE_SERVOS
      sprintf(&out_buf[2], "%d\r", 0);
#endif  // ARTICUBOTS_USE_SERVOS
      respond();
      break;

#ifdef ARTICUBOTS_USE_BASE
    case READ_ENCODERS:
      sprintf(&out_buf[2], "%ld %ld\r", readEncoder(LEFT), readEncoder(RIGHT));
      respond();
      break;
    case RESET_ENCODERS:
      EncodersReset();
      // do nothing
      respond_OK(cmd);
      break;
    case READ_HEALTH:
      {
        sprintf(&out_buf[2], "%ld %d %d\r", battery_voltage_mv, battery_current_ma, 0);
        respond();
      }
      break;
    case MOTOR_SPEEDS:
      /* Reset the auto stop timer */
      lastMotorCommandMs = millis();
      //Serial.print(arg1);
      //Serial.print(" --- ");
      //Serial.println(arg2);

      // speedSetpoint* is in the range -100...100 - it has a meaning of "percent of max possible speed".
      speedSetpointL = arg1;
      speedSetpointR = arg2;
      respond_OK(cmd);
      break;
    case MOTOR_RAW_PWM:
      /* Reset the auto stop timer */
      lastMotorCommandMs = millis();
      speedSetpointL = arg1;
      speedSetpointR = arg2;
      respond_OK(cmd);
      break;
    case UPDATE_PID:
      // do nothing
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
