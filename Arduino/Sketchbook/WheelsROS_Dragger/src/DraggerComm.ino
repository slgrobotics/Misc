// **************************
// Remote control code
// **************************

// #include "freeram.h" - originally from MotionPlug library
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//#define ARTICUBOTS_USE_SERVOS
#define ARTICUBOTS_USE_BASE

#define COMM_SERIAL Serial
#define BAUDRATE_ARTICUBOTS 115200
#define DO_RESPOND_OK

void InitSerial() {

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



// Variable to hold the current single-character command
char cmd;
char cmd_in[128];
bool responsePending = false;

// The arguments converted to integers
long arg1;
long arg2;

char out_buf[128];

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
  constexpr int maxCommandLength = SERIAL_TX_BUFFER_SIZE - sizeof("? Error: ''\r");
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
  if (cmd == MOTOR_SPEEDS || cmd == MOTOR_RAW_PWM) {
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
    case READ_ENCODERS: {
      long long left, right;
      snapshotEncoders(left, right);
      sprintf(&out_buf[2], "%ld %ld\r", encoderWireCount(left), encoderWireCount(right));
      respond();
      break;
    }
    case RESET_ENCODERS:
      EncodersReset();
      resetPID();
      respond_OK(cmd);
      break;
    case READ_HEALTH:
      {
        long voltage_mv = analogRead(batteryVoltageInPin); // "900" here relates to battery voltage 13.713V
        voltage_mv = voltage_mv * 13713 / 900; // millivolts, returns "12000" for 12V
        long current_ma = analogRead(batteryCurrentInPin);
        current_ma = max(39l,current_ma);
        current_ma = (current_ma - 39l) * 4530l / 148l; // milliamperes, returns 148 for 4.53A
        sprintf(&out_buf[2], "%ld %ld %d\r", voltage_mv, current_ma, freeRam());
        respond();
      }
      break;
    case MOTOR_SPEEDS:
      lastMotorCommandMs = millis();
      if (!isControlByJoystick()) {
        if (rawPwmActive || (arg1 == 0 && arg2 == 0)) stopDrive();
        desiredSpeedL = constrain(arg1, -100L, 100L);
        desiredSpeedR = constrain(arg2, -100L, 100L);
      }
      respond_OK(cmd);
      break;
    case MOTOR_RAW_PWM:
      lastMotorCommandMs = millis();
      if (!isControlByJoystick()) {
        if (!rawPwmActive) stopDrive();
        rawPwmActive = true;
        pwm_L = constrain(arg1, -255L, 255L);
        pwm_R = constrain(arg2, -255L, 255L);
        set_motors();
      }
      respond_OK(cmd);
      break;
    case UPDATE_PID:
      // Acknowledge the ROS startup command. PID gains are configured locally.
      respond_OK(cmd);
      break;
#endif  // ARTICUBOTS_USE_BASE

    default:
      respond_ERROR(cmd_in);
      break;
  }
}


void resetPID() {
  myPID_L.SetMode(MANUAL);
  myPID_R.SetMode(MANUAL);
  dpwm_L = dpwm_R = 0;
  myPID_L.SetMode(AUTOMATIC);
  myPID_R.SetMode(AUTOMATIC);
}

void stopDrive() {
  rawPwmActive = false;
  desiredSpeedL = desiredSpeedR = 0;
  setpointSpeedL = setpointSpeedR = 0;
  pwm_L = pwm_R = 0;
#ifdef USE_EMA
  clearEma();
#endif
  resetPID();
  set_motors();
}
