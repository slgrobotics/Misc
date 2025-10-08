//#define TRACE
//#define USE_EMA
//#define USE_PIDS
//#define HAS_ENCODERS
#define USE_SERVOS

// See https://github.com/slgrobotics/robots_bringup

#ifdef USE_PIDS
#include <PID_v1.h>
#endif // USE_PIDS

//
// This code is an adaptation of Dragger and Plucky robots code, which runs on Mega 2560
// The "Comm" part complies with articubot_one and can be used with ROS2 interchangibly.
//
// As of 2025-10-08 this code is deployed on Seggy robot and works with its joystick and ROS2 Jazzy
//
// See https://github.com/slgrobotics/robots_bringup/tree/main/Docs
//     https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/DraggerROS
//     https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/PluckyWheelsROS
//
// Sergei Grichine - slg@quakemap.com
//
// Note: using Arduino IDE 1.8.19 (snap installed) + Raspberry Pi Pico on Ubunti 24.04
//
// The IDE does not automatically upload the compiled ".uf2" file to Pico.
//
// look for something like "WheelsROS_Pico.ino.uf2" in:
//             admin:///tmp/snap-private-tmp/snap.arduino/tmp/arduino_build_NNNNN
//
// drag it to "usb drive", appearing when Pico is powered up with "boot" button pressed.
//

const int ledPin = LED_BUILTIN;

#define JOYSTICK_ACTIVATE_PIN 12
#define JOYSTICK_PRESSED_PIN 13

const int AD_Xpin = A0;         // Joystick X pin
const int AD_Ypin = A1;         // Joystick Y pin

#define JOYSTICK_TRIM_X 7
#define JOYSTICK_TRIM_Y -7

#define DEADZONE_JS 0.0

#ifdef USE_SERVOS
// Servo pins:
#define PIN_STEER 14
#define PIN_THROTTLE 15
#define PIN_EXTRA 11
#endif // USE_SERVOS

// diagnostic LEDs:
const int redLedPin = 16;
const int yellowLedPin = 17;
const int blueLedPin = 18;
const int greenLedPin = 19;
const int whiteLedPin = 20;

// Pico has three accessible analog input pins (GP26=A0, GP27=A1, and GP28=A2) 
const int batteryVoltageInPin = A2;  // Analog input pin that the battery 1/3 divider is attached to. "800" = 3.90V per cell (11.72V).
//const int batteryCurrentInPin = A3;  // Analog input pin that the current sensor is attached to.

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int controlLoopFactor = 20; // factor of 20 make for 100 ms Control (PID) cycle

//-------------------------------------- Variable definitions --------------------------------------------- //

#ifdef HAS_ENCODERS
// 64-bit integers to avoid overflow
volatile long long Ldistance, Rdistance;   // encoders - distance traveled, in ticks
long long LdistancePrev = 0;   // last encoders values - distance traveled, in ticks
long long RdistancePrev = 0;

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

long distR; // ticks per 100ms cycle, as measured
long distL;
#endif // HAS_ENCODERS

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to motor (H-Bridge or servos) pins (will be constrained by set_motor())
#ifdef USE_PIDS
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above
#endif // USE_PIDS

int angle_steer{0};
int angle_throttle{0};

// desired speed can be set by joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// desired speed is set by Comm or Joystick.
// comes in the range -100...100 - it has a meaning of "percent of max possible speed"
double desiredSpeedR = 0.0;
double desiredSpeedL = 0.0;

// PID Setpoints (desired speed after ema, -100...100 ):
double setpointSpeedR = 0.0;
double setpointSpeedL = 0.0;

const int RightMotorChannel = 0;  // index to ema*[] arrays
const int LeftMotorChannel = 1;

// EMA period to smooth wheels movement. 100 is smooth but fast, 300 is slow.
const int EmaPeriod = 20;

// Robot physical parameters:
double wheelBaseMeters = 0.600;
double wheelRadiusMeters = 0.192;
double encoderTicksPerRevolution = 2506; // one wheel rotation

#ifdef USE_PIDS
// higher Ki causes residual rotation, higher Kd - jerking movement
PID myPID_R(&speedMeasured_R, &dpwm_R, &setpointSpeedR, 1.0, 0, 0.05, DIRECT);    // in, out, setpoint, double Kp, Ki, Kd, DIRECT or REVERSE
PID myPID_L(&speedMeasured_L, &dpwm_L, &setpointSpeedL, 1.0, 0, 0.05, DIRECT);
#endif // USE_PIDS

bool testDir = false;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
unsigned long AUTO_STOP_INTERVAL = 2000;

// milliseconds from last events:
unsigned long lastMotorCommandMs = 0;
unsigned long lastCommMs = 0;
unsigned long lastSonarMs = 0;

//-------------------------------------- End of variable definitions -----------------------------------------//

unsigned long timer = 0;     // general purpose timer, microseconds
unsigned long timer_old;

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

void setup()
{
  InitSerial(); // ArticuBots uplink, uses Serial/USB.

  InitLeds();

#ifdef USE_PIDS
  int PID_SAMPLE_TIME = mydt * controlLoopFactor;  // milliseconds.

  // turn the PID on and set its parameters:
  myPID_R.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. PID output will be added to PWM on each cycle.
  myPID_R.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_R.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_L.SetOutputLimits(-250.0, 250.0);
  myPID_L.SetSampleTime(PID_SAMPLE_TIME);
  myPID_L.SetMode(AUTOMATIC);
#endif // USE_PIDS

  // ======================== init motors and encoders: ===================================

  MotorsInit();

#ifdef HAS_ENCODERS
  EncodersInit();    // Initialize the encoders - attach interrupts
#endif // HAS_ENCODERS

  setEmaPeriod(RightMotorChannel, EmaPeriod);
  setEmaPeriod(LeftMotorChannel, EmaPeriod);

  timer = micros();
  delay(20);
  loopCnt = 0;
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
  /*
  // Test and adjust joystick:
  int xx = analogRead(AD_Xpin) + JOYSTICK_TRIM_X;
  int yy = analogRead(AD_Ypin) + JOYSTICK_TRIM_Y;
  double jx = joystickX();
  double jy = joystickY();
  Serial.print(" X: ");
  Serial.print(xx);
  Serial.print(" / ");
  Serial.print(jx);
  Serial.print("      Y: ");
  Serial.print(yy);
  Serial.print(" / ");
  Serial.print(jy);
  Serial.print("      active: ");
  Serial.print(isJoystickActive());
  Serial.print("  pressed: ");
  Serial.println(isJoystickPressed());

  computeJoystickSpeeds();

  // Test servos:
  pwm_R = map(joystickSpeedR, -100, 100, -255, 255);
  pwm_L = map(joystickSpeedL, -100, 100, -255, 255);

  Serial.print("PWM: R: ");
  Serial.print(pwm_R);
  Serial.print("   L: ");
  Serial.println(pwm_L);

  //constrainPwm();

  //pwmToAngles();

  set_motors();

  delay(100);
  return;
  */
  
  //delay(1); // 1 ms

  // whole loop takes slightly over 3ms, with a bit less than 2ms left to idle

  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while ((micros() - timer) < STD_LOOP_TIME)
    {
      readCommCommand();  // reads desiredSpeed
    }
  }

  loopCnt++;
  timer_old = timer;
  timer = micros();

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
      Serial.print(desiredSpeedR);
      Serial.print("   Measured: ");
      Serial.println(speedMeasured_R);
    */
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif

  if (millis() - lastCommMs > AUTO_STOP_INTERVAL || millis() - lastMotorCommandMs > AUTO_STOP_INTERVAL)
  {
    // failsafe - Comm signal or motor command is not detected for more than a second
    desiredSpeedR = desiredSpeedL = 0.0;
  }

  // test - controller should hold this speed:
  //desiredSpeedR = 20;
  //desiredSpeedL = 20;

  if (isJoystickActive())
  {
    // ignore Comm values and override by joystick on A0 and A1:

    computeJoystickSpeeds();

    desiredSpeedL = joystickSpeedL;
    desiredSpeedR = joystickSpeedR;

    //  myPID_L.SetMode(MANUAL);  // Disables PID, input goes straight to PWM
    //  myPID_R.SetMode(MANUAL);
    //} else {
    //  myPID_L.SetMode(AUTOMATIC);
    //  myPID_R.SetMode(AUTOMATIC);
  }

#ifdef USE_EMA
  // smooth movement by using ema: take desiredSpeed and produce setpointSpeed
  ema(RightMotorChannel);
  ema(LeftMotorChannel);
#else // USE_EMA
  setpointSpeedR = desiredSpeedR; // no ema
  setpointSpeedL = desiredSpeedL;
#endif // USE_EMA

#ifdef USE_PIDS
  // compute control inputs - increments to current PWM
  myPID_R.Compute();
  myPID_L.Compute();
#endif // USE_PIDS

  if (isControlLoop) // we do speed PID calculation on a slower scale, about 10Hz
  {
#ifdef HAS_ENCODERS
    // based on PWM increments, calculate speed:
    speed_calculate();
#endif // HAS_ENCODERS

#ifdef USE_PIDS
    // Calculate the pwm, given the desired speed (pick up values computed by PIDs):
    pwm_calculate();
#else // USE_PIDS
    pwm_R = constrain(map(setpointSpeedR, -100, 100, -255, 255), -255.0, 255.0);
    pwm_L = constrain(map(setpointSpeedL, -100, 100, -255, 255), -255.0, 255.0);
#endif // USE_PIDS

    // if(abs(pwm_R) > 250 || abs(desiredSpeedR - speedMeasured_R) > 3) {
    //   Serial.print(speedMeasured_R);
    //   Serial.print("  ");
    //   Serial.println(pwm_R);
    // }

    // test: At both motors set to +80 expect Ldistance and Rdistance to increase
    //pwm_R = 80.0;
    //pwm_L = 80.0;
    //if(!isJoystickActive()) {
    //  pwm_L = 255.0;  // measure full speed distR and distL. See SpeedControl tab.
    //  pwm_R = 255.0;
    //}

    set_motors();

    digitalWrite(redLedPin, millis() - lastCommMs > 1000 ? HIGH : LOW);
    digitalWrite(whiteLedPin, millis() - lastMotorCommandMs > 1000 ? HIGH : LOW);
    //digitalWrite(blueLedPin, millis() - lastSonarMs > 1000 ? HIGH : LOW);

    digitalWrite(greenLedPin, !digitalRead(greenLedPin)); // blinking at 10 Hz
    digitalWrite(ledPin,!digitalRead(ledPin));
  }

  // we are done in about 3.4ms

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - timer;

}
