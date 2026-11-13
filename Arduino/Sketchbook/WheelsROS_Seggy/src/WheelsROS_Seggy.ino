//#define TRACE
//#define HAS_LEDS
//#define USE_ARCADE_DRIVE

// See https://github.com/slgrobotics/robots_bringup

//
// This code is an adaptation of Dragger and Plucky robots code, which runs on Mega 2560, to Seggy's Teensy 4.0
// The "Comm" part complies with articubot_one and can be used with ROS2 interchangibly.
//
// As of 2025-10-20 this code is deployed on Seggy robot and works with its joystick and ROS2 Jazzy
//
// See https://github.com/slgrobotics/robots_bringup/tree/main/Docs
//     https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/DraggerROS
//     https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/PluckyWheelsROS
//     all the experiments and tests in the Sketchbook, named FOC_*
//     https://github.com/slgrobotics/diffdrive_arduino   - ROS2 driver for these wheels
//
// Sergei Grichine - slg@quakemap.com
//
// Note: using Arduino IDE 2.3.6 + Teensy plugins + Teensy 4.0 on Ubunti 24.04
//

#include <SimpleFOC.h>

#define MAX_SPEED_RAD_S 3.5

#define JOYSTICK_ACTIVATE_PIN 32
#define JOYSTICK_PRESSED_PIN 33
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A9

#define JOYSTICK_TRIM_X 12
#define JOYSTICK_TRIM_Y -2

#define DEADZONE_JS 5.0

const int batteryVoltageInPin = A10;  // Analog input pin that the battery 1/3 divider is attached to. 4.7K / 1.2K does the job for 15V max.

#define BUZZER_PIN 26

const int mainLedPin = LED_BUILTIN;

#ifdef HAS_LEDS
// additional diagnostic LEDs:
const int redLedPin = 28;
const int greenLedPin = 29;
const int whiteLedPin = 30;
const int blueLedPin = 31;
#endif // HAS_LEDS

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int controlLoopFactor = 20; // factor of 20 make for 100 ms Control cycle

//-------------------------------------- Variable definitions --------------------------------------------- //

// 64-bit integers to avoid overflow
volatile long long Ldistance, Rdistance;   // encoders - distance traveled, in ticks
long long LdistancePrev = 0;   // last encoders values - distance traveled, in ticks
long long RdistancePrev = 0;

double speedMeasured_R = 0;  // percent of max speed for this drive configuration, -100..100
double speedMeasured_L = 0;

long distR; // ticks per 100ms cycle, as measured
long distL;

// desired speed can be set by joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// Setpoints (desired speed) is set by Comm or Joystick.
// comes in the range -100...100 - it has a meaning of "percent of max possible speed"
double speedSetpointR = 0.0;
double speedSetpointL = 0.0;

// Robot physical parameters:
double wheelBaseMeters = 0.46;
double wheelRadiusMeters = 0.12;
double encoderTicksPerRevolution = 2506; // one wheel rotation

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
unsigned long AUTO_STOP_INTERVAL = 2000;

long battery_voltage_mv = 0l;
long battery_current_ma = 0l;

// https://docs.simplefoc.com/inline_current_sense#standalone-current-sense
PhaseCurrent_s current_ph_L;
PhaseCurrent_s current_ph_R;
DQCurrent_s current_dq_L;
DQCurrent_s current_dq_R;

float current_dc_L = 0.0;
float current_dc_R = 0.0;

// milliseconds from last events:
unsigned long lastMotorCommandMs = 0;
unsigned long lastCommMs = 0;
unsigned long lastSonarMs = 0;

//-------------------------------------- End of variable definitions -----------------------------------------//

bool init_ok = false;

unsigned long timer = 0;     // general purpose timer, microseconds
unsigned long timer_old;

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

void setup()
{
  InitSerial(); // ArticuBots uplink, uses Serial/USB.

  InitLeds();

  pinMode (BUZZER_PIN, OUTPUT);
  buzz(2, 50, 100);

  // ======================== init motors and encoders: ===================================

  init_ok = MotorsInit();

  timer = micros();
  delay(20);
  loopCnt = 0;

  if(init_ok) {
    buzz(1, 200, 200);
  } else {
    buzz(3, 500, 500);
  }
}

// =========================== main loop timing ============================================
unsigned long STD_LOOP_TIME  = mydt * 1000;  // Fixed time loop of 5 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;  // see Kalman dt

#ifdef TRACE
const int printLoopFactor = 200;
int printLoopCnt = 0;
#endif

// =========================================================================================

void loop() //Main Loop
{
  if (!init_ok) {
    // Error: blinking LED_BUILTIN very slowly, doing nothing
    blinkLED(10, 2000);
    return;
  }

  // Test and adjust joystick: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/FOC_joystick_test_Teensy_40

  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while ((micros() - timer) < STD_LOOP_TIME)
    {
      motorLoop(); // FOC must run as often as possible
      readCommCommand();  // reads speedSetpointL/R and other commands from ROS2 driver - https://github.com/slgrobotics/diffdrive_arduino
    }
  }

  loopCnt++;
  timer_old = timer;
  timer = micros();

  motorLoop(); // FOC must run as often as possible

  boolean isControlLoop = loopCnt % controlLoopFactor == 0;

#ifdef TRACE
  boolean isPrintLoop = printLoopCnt++ >= printLoopFactor;

  if (isPrintLoop)
  {
    printLoopCnt = 0;
    /*
      Serial.print("Dist/cycle: ");
      Serial.print(distR);
      Serial.print("   Desired: ");
      Serial.print(speedSetpointR);
      Serial.print("   Measured: ");
      Serial.println(speedMeasured_R);
    */
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif

  if (millis() - lastCommMs > AUTO_STOP_INTERVAL || millis() - lastMotorCommandMs > AUTO_STOP_INTERVAL)
  {
    // failsafe - Comm signal or motor command is not detected for more than a second
    speedSetpointR = speedSetpointL = 0.0;
  }

  // test - controller should hold this speed:
  //speedSetpointR = 20;
  //speedSetpointL = 20;

  if (isJoystickActive())
  {
    // ignore Comm values and override by joystick on A2 and A9:

    computeJoystickSpeeds();

    speedSetpointL = joystickSpeedL;
    speedSetpointR = joystickSpeedR;
  }

  // At this point the speedSetpointL/R should be determined and will be used for motors

  if (isControlLoop) // we do speed calculation on a slower scale, about 10Hz
  {
    battery_voltage_mv = analogRead(batteryVoltageInPin);
    battery_voltage_mv = battery_voltage_mv * 16204l / 1000l; // millivolts, returns "12000" for 12.0V

    digitalWrite(BUZZER_PIN, battery_voltage_mv < 10000l);

    getFOCCurrents();

    /*
    //battery_current_ma = 529l + 51l;
    battery_current_ma = max(51l,battery_current_ma);
    battery_current_ma = (battery_current_ma - 51l) * 3730l / 529l; // milliamperes, returns 580 for 3.73A
    //battery_current_ma = (battery_current_ma - 51l); // direct A/D reading, after offset
    */

    // calculate speed and distance:
    speed_calculate();

    set_motors(); // pass speedSetpointR/L to FOC

#ifdef HAS_LEDS
    digitalWrite(redLedPin, millis() - lastCommMs > 1000 ? HIGH : LOW);
    digitalWrite(whiteLedPin, millis() - lastMotorCommandMs > 1000 ? HIGH : LOW);
    //digitalWrite(blueLedPin, millis() - lastSonarMs > 1000 ? HIGH : LOW);

    digitalWrite(greenLedPin, !digitalRead(greenLedPin)); // blinking at 10 Hz
#endif // HAS_LEDS

    // Normal cycle blinking: 5 Hz
    digitalWrite(mainLedPin,!digitalRead(mainLedPin));
  }

  // we are done in about 3.4ms

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - timer;

}
