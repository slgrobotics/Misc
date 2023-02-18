//#define TRACE
//#define USE_EMA

#include <digitalWriteFast.h>
#include <Wire.h>
//#include "I2Cdev.h"

#include <Odometry.h>
#include <PID_v1.h>
#include "mpu.h"        // MotionPlug

const int ledPin = 13;    // Arduino UNO Yellow LED

// diagnostic LEDs:
const int redLedPin = 49;
const int yellowLedPin = 51;
const int blueLedPin = 50;
const int greenLedPin = 53;
const int whiteLedPin = 52;

const int batteryInPin = A3;  // Analog input pin that the battery 1/3 divider is attached to

#define SONAR_I2C_ADDRESS 9   // parking sonar sensor, driven by Arduino Pro Mini

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int pidLoopFactor = 20; // factor of 20 make for 100ms PID cycle

//-------------------------------------- Variable definitions --------------------------------------------- //

volatile long Ldistance, Rdistance;   // encoders - distance traveled, in ticks
long LdistancePrev = 0;   // last encoders values - distance traveled, in ticks
long RdistancePrev = 0;

int pwm_R, pwm_L;       // pwm -255..255 sent to H-Bridge pins (will be constrained by set_motor()) 
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

// desired speed can be set joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// desired speed is set by Comm:
double desiredSpeedR = 0.0;
double desiredSpeedL = 0.0;

// PID Setpoints (desired speed after ema):
double setpointSpeedR = 0.0;
double setpointSpeedL = 0.0;

const int RightMotorChannel = 0;  // index to ema*[] arrays
const int LeftMotorChannel = 1;

// EMA period to smooth wheels movement. 100 is smooth but fast, 300 is slow.
const int EmaPeriod = 20;

// Plucky robot physical parameters:
double wheelBaseMeters = 0.600;
double wheelRadiusMeters = 0.192;
double encoderTicksPerRevolution = 2506; // now has 16T sprocket. Prior art - 47T sprocket: 853;  // one wheel rotation

// current robot pose, updated by odometry:
double X;      // meters
double Y;      // meters
double Theta;  // radians, positive clockwise

DifferentialDriveOdometry *odometry;

// higher Ki causes residual rotation, higher Kd - jerking movement
PID myPID_R(&speedMeasured_R, &dpwm_R, &setpointSpeedR, 1.0, 0, 0.05, DIRECT);    // in, out, setpoint, double Kp, Ki, Kd, DIRECT or REVERSE
PID myPID_L(&speedMeasured_L, &dpwm_L, &setpointSpeedL, 1.0, 0, 0.05, DIRECT);

// received from parking sonar sensor Slave, readings in centimeters:
volatile int rangeFRcm;
volatile int rangeFLcm;
volatile int rangeBRcm;
volatile int rangeBLcm;

// delivered by MotionPlug via I2C:
double compassYaw = 0.0;

int gpsFix = 0;
int gpsSat = 0;
int gpsHdop = 0;
String longlat = "";
unsigned long lastGpsDataMs = 0; 

char gpsChars[100];

bool testDir = false;

/* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
unsigned long AUTO_STOP_INTERVAL = 2000;

// milliseconds from last events:
unsigned long lastImuMs = 0;
unsigned long lastMotorCommandMs = 0;
unsigned long lastCommMs = 0;
unsigned long lastSonarMs = 0;

// ------------------------------------------------------------------------------------------------------ //

unsigned long timer = 0;     // general purpose timer, microseconds
unsigned long timer_old;

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

void setup()
{
  InitSerial(); 

  InitLeds();
  
  int PID_SAMPLE_TIME = mydt * pidLoopFactor;  // milliseconds.

  // turn the PID on and set its parameters:
  myPID_R.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. PID output will be added to PWM on each cycle.
  myPID_R.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_R.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_L.SetOutputLimits(-250.0, 250.0);
  myPID_L.SetSampleTime(PID_SAMPLE_TIME);
  myPID_L.SetMode(AUTOMATIC);

  InitializeI2c();

  mympu_open(200);   // see C:\Projects\Arduino\Sketchbook\MotionPlug    rate: Desired fifo rate (Hz), max 200

  // ======================== init motors and encoders: ===================================

  MotorsInit();

  EncodersInit();    // attach interrupts

  odometry = new DifferentialDriveOdometry();
  odometry->Init(wheelBaseMeters, wheelRadiusMeters, encoderTicksPerRevolution);

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
  // test - direct motor PWM control:

  digitalWriteFast(RDIR, testDir ? LOW : HIGH);
  analogWrite(RPWM, 200);

  digitalWriteFast(LDIR, testDir ? HIGH : LOW);
  analogWrite(LPWM, 200);

  // test - motor PWM control via set_motors():
//  pwm_R = 255;
//  pwm_L = 255;
//  set_motors();

  printAll();
  delay(3000);
  testDir = !testDir;
  return;
  /**/
  
  //delay(1); // 1 ms

  // whole loop takes slightly over 3ms, with a bit less than 2ms left to idle
  
  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while((micros() - timer) < STD_LOOP_TIME)
    {
      mympu_update();   // Main loop runs at 200Hz (mydt=5), to allow frequent sampling of AHRS

      readGpsUplink();

      readCommCommand();  // reads desiredSpeed and reports odometry
    }
  }
  
  loopCnt++;
  timer_old = timer;
  timer = micros(); 
  
  boolean isPidLoop = loopCnt % pidLoopFactor == 0;
#ifdef TRACE
  boolean isPrintLoop = printLoopCnt++ >= printLoopFactor;
#endif

  //digitalWrite(11, HIGH);

#ifdef TRACE
  if(isPrintLoop)
  {
    printLoopCnt = 0;
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif
  
  if(millis() - lastCommMs > AUTO_STOP_INTERVAL || millis() - lastMotorCommandMs > AUTO_STOP_INTERVAL)
  {
    // failsafe - Comm signal or motor command is not detected for more than a second
    desiredSpeedR = desiredSpeedL = 0.0;
  }

  // test - controller should hold this speed:
  //desiredSpeedR = 20;
  //desiredSpeedL = 20;
  
  if(isControlByJoystick())
  {
    // ignore Comm values and override by joystick on A0 and A1:

    computeJoystickSpeeds();
  
    desiredSpeedL = joystickSpeedL;
    desiredSpeedR = joystickSpeedR;
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

  if(isPidLoop)  // we do speed PID and odometry calculation on a slower scale, about 20Hz
  {
    // based on PWM increments, calculate speed:
    speed_calculate();
    
    // Calculate the pwm, given the desired speed (pick up values computed by PIDs):
    pwm_calculate();

    // test: At both motors set to +80 expect Ldistance and Rdistance to increase
    //pwm_R = 80;
    //pwm_L = 80;
    //pwm_L = 255;  // measure full speed distR and distL. See SpeedControl tab.

    set_motors();

    // odometry calculation takes 28us
    //digitalWrite(10, HIGH);
    // process encoders readings into X, Y, Theta using odometry library:
    odometry->wheelEncoderLeftTicks = Ldistance;
    odometry->wheelEncoderRightTicks = Rdistance;
  
    odometry->Process();
  
    if (odometry->displacement.halfPhi != 0.0 || odometry->displacement.dCenter != 0.0)
    {
      double theta = Theta + odometry->displacement.halfPhi;   // radians
  
      // calculate displacement in the middle of the turn:
      double dX = odometry->displacement.dCenter * cos(theta);      // meters
      double dY = odometry->displacement.dCenter * sin(theta);      // meters
  
      X += dX;
      Y += dY;
      
      Theta += odometry->displacement.halfPhi * 2.0;
    }
    //digitalWrite(10, LOW);

    digitalWriteFast(redLedPin, millis() - lastCommMs > 1000 ? HIGH : LOW);
    digitalWriteFast(yellowLedPin, millis() - lastImuMs > 1000 ? HIGH : LOW);
    digitalWriteFast(whiteLedPin, millis() - lastMotorCommandMs > 1000 ? HIGH : LOW);
    digitalWriteFast(blueLedPin, millis() - lastSonarMs > 1000 ? HIGH : LOW);

    digitalWriteFast(greenLedPin,!digitalReadFast(greenLedPin));  // blinking at 10 Hz
    //digitalWriteFast(ledPin,!digitalReadFast(ledPin));
  }

  // we are done in about 3.4ms
  //digitalWrite(11, LOW);

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - timer;

}
