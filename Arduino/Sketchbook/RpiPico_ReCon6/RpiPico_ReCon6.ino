#define TRACE
//#define USE_EMA

#include <PID_v1.h>

#include "hardware/pwm.h"
#include "hardware/clocks.h"

//
// this code is written for Raspberry Pi Pico.
//
// Sergei Grichine -  slg@quakemap.com   September 2021
//

//-------------------------------------- Variable definitions ---------------------------------------------//

// debugging LEDs:
const int redLedPin   = 16;
const int greenLedPin = 17;
const int blueLedPin  = 18;

#define DBG1_PIN 14
#define DBG2_PIN 15

const int buzzerPin   = 19;   // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int pidLoopFactor = 20; // factor of 20 make for 100ms PID cycle

double k1, k2, k3, k4;   // Adjustable Parameters - use as you wish

volatile int Ldistance, Rdistance; // encoders - distance traveled, used by balance calculator for increments and zeroed often

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to H-Bridge pins (will be constrained by set_motor()) 
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

// desired speed can be set joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// desired speed is set by R/C
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double desiredSpeedR = 0.0;
double desiredSpeedL = 0.0;

// R/C pulse width measurements, microseconds:
volatile int PW_LEFT;
volatile int PW_RIGHT;

// PID Setpoints (desired speed after ema):
double setpointSpeedR = 0.0;
double setpointSpeedL = 0.0;

const int RightMotorChannel = 0;  // index to ema*[] arrays
const int LeftMotorChannel = 1;

// EMA period to smooth wheels movement. 100 is smooth but fast, 300 is slow.
const int EmaPeriod = 20;

// higher Ki causes residual rotation, higher Kd - jerking movement
PID myPID_R(&speedMeasured_R, &dpwm_R, &setpointSpeedR, 1.0, 0.0, 0.08, DIRECT);    // in, out, setpoint, double Kp, Ki, Kd, DIRECT or REVERSE
PID myPID_L(&speedMeasured_L, &dpwm_L, &setpointSpeedL, 1.0, 0.0, 0.08, DIRECT);

// milliseconds from last events:
long lastComm = 0;
long lastPwm = 0;

//-------------------------------------- End of variable definitions -----------------------------------------//

unsigned long timer = 0;     // general purpose timer
unsigned long timer_old;
unsigned long timer_rc_avail;

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

void setup()
{
  Serial.begin(115200);   // start serial for USB

  InitLeds();

  //InitRcInput();
  
  int PID_SAMPLE_TIME = mydt * pidLoopFactor;  // milliseconds.

  // turn the PID on and set its parameters:
  myPID_R.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. PID output will be added to PWM on each cycle.
  myPID_R.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_R.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_L.SetOutputLimits(-250.0, 250.0);
  myPID_L.SetSampleTime(PID_SAMPLE_TIME);
  myPID_L.SetMode(AUTOMATIC);

  // DEBUG pins:
  pinMode(DBG1_PIN, OUTPUT);
  pinMode(DBG2_PIN, OUTPUT);

  k1 = 0;
  k2 = 0;
  k3 = 0;
  k4 = 0;

  MotorsInit();
  
  EncodersInit();	// Initialize the encoders
  
  setEmaPeriod(RightMotorChannel, EmaPeriod);
  setEmaPeriod(LeftMotorChannel, EmaPeriod);
  
  pinMode(buzzerPin, OUTPUT);
  shortBuzz();
  
  timer = micros(); 
}

// =========================== main loop timing ============================================
unsigned long STD_LOOP_TIME  = mydt * 1000;  // Fixed time loop of 5 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;  // see Kalman dt

const int slowLoopFactor = 40;
int slowLoopCnt = slowLoopFactor;    // should fire on first loop to read k* values

#ifdef TRACE
const int printLoopFactor = 200;
int printLoopCnt = 0;
#endif

// =========================================================================================

void loop()
{
  // whole loop takes slightly over 3ms, with a bit less than 2ms left to idle
  
  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while((micros() - timer) < STD_LOOP_TIME)
    {
      ;
    }
  }
  
  //digitalWrite(DBG1_PIN, HIGH);
  loopCnt++;
  timer_old = timer;
  timer = micros(); 
  
  boolean isSlowLoop = slowLoopCnt++ >= slowLoopFactor;
  boolean isPidLoop = loopCnt % pidLoopFactor == 0;
#ifdef TRACE
  boolean isPrintLoop = printLoopCnt++ >= printLoopFactor;
#endif

  //digitalWrite(DBG2_PIN, HIGH);

  if(isSlowLoop)
  {
    slowLoopCnt = 0;

    // multipliers are set to the experimental values, tuned to POTs in neutral position:
    //k3 = analogRead(AD2pin) * 1.0;
    //k4 = analogRead(AD3pin) * 1.0;
  }
  
#ifdef TRACE
  if(isPrintLoop)
  {
    printLoopCnt = 0;
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif

/*
  if(RC_avail_RIGHT())
  {
    desiredSpeedR = map(constrain(PW_RIGHT, 1000, 2000), 1000, 2000, -100, 100);
    timer_rc_avail = timer;
    digitalWrite(redLedPin, LOW);
    digitalWrite(blueLedPin, HIGH);
  }

  if(RC_avail_LEFT())
  {
    desiredSpeedL = map(constrain(PW_LEFT, 1000, 2000), 1000, 2000, -100, 100);
    timer_rc_avail = timer;
    digitalWrite(redLedPin, LOW);
    digitalWrite(blueLedPin, HIGH);
  }

  if(timer - timer_rc_avail > 1000000)
  {
    // failsafe - R/C signal is not detected for more than a second
    desiredSpeedR = desiredSpeedL = 0.0;
    PW_RIGHT = PW_LEFT = 1500;
    digitalWrite(redLedPin, HIGH);
    digitalWrite(blueLedPin, LOW);
  }
*/

  // test - controller should hold this speed:
  //desiredSpeedR = 10;
  //desiredSpeedL = 10;

  if(isControlByJoystick())
  {
    // ignore R/C values and override by joystick on A0 and A1:

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

  if(isPidLoop)  // we do odometry calculation on a slower scale, about 20Hz
  {
    bool isDirectLaw = true;

    if(isDirectLaw)
    {
      // no PID or EMA, just direct power inputs:
      pwm_R = desiredSpeedR * 255.0 / 100.0;
      pwm_L = desiredSpeedL * 255.0 / 100.0;

      // do not allow float values to grow beyond motor maximums:
      pwm_R = constrain(pwm_R, -255.0, 255.0);     // Maximum / Minimum Limitations
      pwm_L = constrain(pwm_L, -255.0, 255.0);
    }
    else
    {
      // based on encoder distance increments, calculate speed:
      speed_calculate();
      // Calculate the pwm, given the desired speed (pick up values computed by PIDs):
      pwm_calculate();
    }
  
    // test: At both motors set to +80 expect Ldistance and Rdistance to increase
    //pwm_R = 80.0;
    //pwm_L = 80.0;
    //pwm_L = 255.0;  // measure full speed distR and distL. See SpeedControl tab.

    //pwm_R = 100.0;
    //pwm_L = 100.0;

    set_motors();
  
    //digitalWrite(DBG1_PIN, LOW);
    
    digitalWrite(greenLedPin,!digitalRead(greenLedPin));  // blinking at 10 Hz
  }

  // we are done in about 3.4ms
  //digitalWrite(DBG2_PIN, LOW);

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
  //digitalWrite(DBG1_PIN, LOW);
}
