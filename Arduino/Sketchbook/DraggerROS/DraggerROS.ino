//#define TRACE
//#define USE_EMA

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
// see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
// and http://code.google.com/p/digitalwritefast/
// and http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino/

// See https://github.com/slgrobotics/robots_bringup

#include <Wire.h>

// looks like AVR MCs don't have this defined, but it works:
#define WIRE_HAS_TIMEOUT

#include <PID_v1.h>
//#include <PID_v1_bc.h>

//
// this code is written for Arduino Mega 2560 and heavily relies on timers 4 and 5, PWM pins 44,45
//                 - see setTimerForPWM() in Motors section.
// Serial3 (pins 14,15) is connected to BLE-LINK module.
// DFrobot.com is the source of original code and their "6 DOF IMU Shield" is used here.
//
// As of 2024-06-24 this code is deployed on Dragger robot and works with its joystick and ROS
//
// See https://github.com/slgrobotics/robots_bringup/tree/main/Docs/Dragger
//
// Sergei Grichine - slg@quakemap.com
//


//const int ledPin = LED_BUILTIN;    // Arduino UNO/Mega Yellow built-in LED (pin 13)
const int buzzerPin   = 13;   // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

// diagnostic LEDs:
const int redLedPin   = 33;
//const int yellowLedPin = 51; TBD
const int greenLedPin = 35;
const int blueLedPin  = 31;
//const int whiteLedPin = 52; TBD

const int batteryInPin = A4;  // Analog input pin that the battery 1/3 divider is attached to. "900" = 13.713V

#define SONAR_I2C_ADDRESS 9   // parking sonar sensor, driven by Arduino Pro Mini (ParkingSensorI2C.ino)

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int pidLoopFactor = 20; // factor of 20 make for 100ms PID cycle

//-------------------------------------- Variable definitions --------------------------------------------- //

// 64-bit integers to avoid overflow
volatile long long Ldistance, Rdistance;   // encoders - distance traveled, in ticks
long long LdistancePrev = 0;   // last encoders values - distance traveled, in ticks
long long RdistancePrev = 0;

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to H-Bridge pins (will be constrained by set_motor())
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

long distR; // ticks per 100ms cycle, as measured
long distL;

// desired speed can be set by joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// desired speed is set by Comm or Joystick.
// comes in the range -100...100 - it has a meaning of "percent of max possible speed":
// For Dragger full wheel rotation takes 2.2 seconds. So, with R_wheel=0.192m dist=1.206m max speed is:
//   - 0.548 m/sec (1.23 mph, 1.97 km/h)
//   - 2.86 rad/sec
//   - 6241 encoder ticks/sec
double desiredSpeedR = 0.0;
double desiredSpeedL = 0.0;

// PID Setpoints (desired speed after ema):
double setpointSpeedR = 0.0;
double setpointSpeedL = 0.0;

const int RightMotorChannel = 0;  // index to ema*[] arrays
const int LeftMotorChannel = 1;

// EMA period to smooth wheels movement. 100 is smooth but fast, 300 is slow.
const int EmaPeriod = 20;

// Dragger robot physical parameters:
double wheelBaseMeters = 0.580;
double wheelRadiusMeters = 0.192;
double encoderTicksPerRevolution = 13730; // one wheel rotation

// higher Ki causes residual rotation, higher Kd - jerking movement
PID myPID_R(&speedMeasured_R, &dpwm_R, &setpointSpeedR, 1.0, 0.0, 0.08, DIRECT);    // in, out, setpoint, double Kp, Ki, Kd, DIRECT or REVERSE
PID myPID_L(&speedMeasured_L, &dpwm_L, &setpointSpeedL, 1.0, 0.0, 0.08, DIRECT);

// received from parking sonar sensor I2C Slave, readings in centimeters (receiveI2cSonarPacket() must be called):
volatile int rangeFRcm;
volatile int rangeFLcm;
volatile int rangeBRcm;
volatile int rangeBLcm;

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

  int PID_SAMPLE_TIME = mydt * pidLoopFactor;  // milliseconds.

  // turn the PID on and set its parameters:
  myPID_R.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. PID output will be added to PWM on each cycle.
  myPID_R.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_R.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_L.SetOutputLimits(-250.0, 250.0);
  myPID_L.SetSampleTime(PID_SAMPLE_TIME);
  myPID_L.SetMode(AUTOMATIC);

  // DEBUG pins:
  //pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);

  InitializeI2c();

  MotorsInit();

  EncodersInit();    // Initialize the encoders - attach interrupts

  setEmaPeriod(RightMotorChannel, EmaPeriod);
  setEmaPeriod(LeftMotorChannel, EmaPeriod);

  pinMode(buzzerPin, OUTPUT);
  shortBuzz();

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

  boolean isPidLoop = loopCnt % pidLoopFactor == 0;
  
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
  //desiredSpeedR = 10;
  //desiredSpeedL = 10;

  if (isControlByJoystick())
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

  // compute control inputs - increments to current PWM
  myPID_R.Compute();
  myPID_L.Compute();

  if (isPidLoop) // we do speed PID and calculation on a slower scale, about 20Hz
  {
    // based on PWM increments, calculate speed:
    speed_calculate();

    // Calculate the pwm, given the desired speed (pick up values computed by PIDs):
    pwm_calculate();

    // if(abs(pwm_R) > 250 || abs(desiredSpeedR - speedMeasured_R) > 3) {
    //   Serial.print(speedMeasured_R);
    //   Serial.print("  ");
    //   Serial.println(pwm_R);
    // }

    // test: At both motors set to +80 expect Ldistance and Rdistance to increase
    //pwm_R = 80.0;
    //pwm_L = 80.0;
    //if(!isControlByJoystick()) {
    //  pwm_L = 255.0;  // measure full speed distR and distL. See SpeedControl tab.
    //  pwm_R = 255.0;
    //}

    set_motors();

    digitalWriteFast(redLedPin, millis() - lastCommMs > 1000 ? HIGH : LOW);
    //digitalWriteFast(whiteLedPin, millis() - lastMotorCommandMs > 1000 ? HIGH : LOW);
    //digitalWriteFast(blueLedPin, millis() - lastSonarMs > 1000 ? HIGH : LOW);

    digitalWriteFast(greenLedPin, !digitalReadFast(greenLedPin)); // blinking at 10 Hz
    //digitalWriteFast(ledPin,!digitalReadFast(ledPin));
  }

  // we are done in about 3.4ms

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - timer;

  /*
    if (lastLoopUsefulTime > STD_LOOP_TIME + STD_LOOP_TIME)
    {
    DbgRed(true);
    }

    if(angle_prev != angle)
    {
    angle_prev = angle;
    DbgGreen(true);
    }
    else
    {
    DbgGreen(false);
    }
  */

  // let operating environment work (maybe taking time) and reenter the loop()
}
