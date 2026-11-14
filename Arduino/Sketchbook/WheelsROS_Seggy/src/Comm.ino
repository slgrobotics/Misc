#include <errno.h>
#include <inttypes.h>

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

/* Variable initialization */

// Variable to hold the current single-character command
char cmd;
char cmd_in[128];

// The arguments converted to integers
long arg1;
long arg2;

char out_buf[128];

bool responsePending = false;

void respond() {
  const size_t length = strlen(out_buf);
  responsePending = COMM_SERIAL.availableForWrite() < static_cast<int>(length);
  if (!responsePending) {
    COMM_SERIAL.write(reinterpret_cast<const uint8_t *>(out_buf), length);
  }
}

void respond_OK(char rq_cmd) {
#ifdef DO_RESPOND_OK
  sprintf(out_buf, "%c OK\r", rq_cmd);
  respond();
#endif
}

void respond_ERROR(const char *rq_cmd_in) {
  constexpr int maxCommandLength = sizeof(out_buf) - sizeof("? Error: ''\r");
  snprintf(out_buf, sizeof(out_buf), "? Error: '%.*s'\r", maxCommandLength, rq_cmd_in);
  respond();
}

void processCommFrame() {
  char *context = nullptr;
  char *command = strtok_r(cmd_in, " \t\n", &context);
  if (!command) {
    respond_OK('*');
    return;
  }

  char *first = strtok_r(nullptr, " \t\n", &context);
  char *second = strtok_r(nullptr, " \t\n", &context);
  char *extra = strtok_r(nullptr, " \t\n", &context);
  if (command[1] != '\0' || extra) {
    respond_ERROR("Invalid command frame");
    return;
  }

  cmd = command[0];
  arg1 = arg2 = 0;
  if (cmd == MOTOR_SPEEDS) {
    if (!first || !second) {
      respond_ERROR("Expected two wheel speeds");
      return;
    }
    char *endLeft;
    char *endRight;
    errno = 0;
    arg1 = strtol(first, &endLeft, 10);
    arg2 = strtol(second, &endRight, 10);
    if (*endLeft || *endRight || errno == ERANGE) {
      respond_ERROR("Invalid wheel speed");
      return;
    }
  } else {
    // Preserve legacy commands, including composite UPDATE_PID arguments.
    if (first) arg1 = atol(first);
    if (second) arg2 = atol(second);
  }

  out_buf[0] = cmd;
  out_buf[1] = ' ';
  out_buf[2] = '\0';
  runCommand();
}

void readCommCommand() {
  static size_t length = 0;
  static bool discardFrame = false;

  if (responsePending) {
    respond();
    if (responsePending) return;
  }

  // Bound the amount of serial work between FOC updates.
  for (int bytes = 0; bytes < 32; ++bytes) {
    const int byte = COMM_SERIAL.read();
    if (byte < 0) return;
    if (byte == '\r') {
      if (discardFrame) {
        respond_ERROR("Invalid or oversized command frame");
      } else {
        cmd_in[length] = '\0';
        processCommFrame();
      }
      length = 0;
      discardFrame = false;
      return;
    }
    if (byte == 0 || length == sizeof(cmd_in) - 1) {
      discardFrame = true;
    }
    if (!discardFrame) cmd_in[length++] = static_cast<char>(byte);
  }
}

// The ROS driver computes encoder deltas across signed 32-bit wraparound.
// Send each counter as a signed 32-bit value that wraps modulo 2^32.
// diffdrive_arduino/src/wheel.cpp:48 computes encoder deltas across that wrap.
int32_t encoderWireCount(long long count) {
  const uint32_t lowBits = static_cast<uint32_t>(count);
  const int64_t signedValue = lowBits <= INT32_MAX
      ? static_cast<int64_t>(lowBits)
      : static_cast<int64_t>(lowBits) - (int64_t{1} << 32);
  return static_cast<int32_t>(signedValue);
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {

  //Serial.print("runCommand: ");
  //Serial.println(cmd);

  switch (cmd) {
    case GET_BAUDRATE:
      sprintf(&out_buf[2], "%d\r", BAUDRATE_ARTICUBOTS);
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

    case READ_ENCODERS: {
      noInterrupts();
      const long long left = Ldistance;
      const long long right = Rdistance;
      interrupts();
      snprintf(&out_buf[2], sizeof(out_buf) - 2, "%" PRId32 " %" PRId32 "\r",
               encoderWireCount(left), encoderWireCount(right));
      respond();
      break;
    }
    case RESET_ENCODERS:
      EncodersReset();
      // do nothing
      respond_OK(cmd);
      break;
    case READ_HEALTH:
      {
        sprintf(&out_buf[2], "%ld %ld %d %d\r", battery_voltage_mv, battery_current_ma, 0,
                digitalRead(batteryChargingInPin) == LOW);
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
      respond_ERROR("Raw PWM is unsupported");
      break;
    case UPDATE_PID:
      // do nothing
      respond_OK(cmd);
      break;

    default:
      respond_ERROR(cmd_in);
      break;
  }
}
