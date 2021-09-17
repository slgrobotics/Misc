//#define TRACE
//#define USE_EMA

#include <PID_v1.h>
#include <Odometry.h>

#include "hardware/pwm.h"
#include "hardware/clocks.h"

//
// this code is written for Raspberry Pi Pico.
//
// Sergei Grichine -  slg@quakemap.com   September 2021
//

//-------------------------------------- Variable definitions ---------------------------------------------//

// debugging LEDs:
const int redLedPin   = 16; // GP16
const int greenLedPin = 17;
const int blueLedPin  = 18;

#define DBG1_PIN 14
#define DBG2_PIN 15

const int buzzerPin   = 19;   // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int pidLoopFactor = 20; // factor of 20 make for 100ms (10Hz) PID cycle

double k1, k2, k3, k4;   // Adjustable Parameters - use as you wish

volatile int Ldistance, Rdistance;    // encoders - distance traveled, used by balance calculator for increments and zeroed often
volatile int oLdistance, oRdistance;  // encoders - distance traveled, used by odometry calculator

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to H-Bridge pins (will be constrained by set_motor()) 
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

// desired speed can be set joystick:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double joystickSpeedR = 0.0;
double joystickSpeedL = 0.0;

// desired speed is set by joystick or Controllers:
// comes in the range -100...100 - it has a meaning of "percent of max speed":
double desiredSpeedR = 0.0;
double desiredSpeedL = 0.0;

// used as input to steering controller:
double steeringControlEffort = 0.0;

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

// Pixy camera variables:
int pixyBlocksCount = 0;
class PixyBlock {
  public:
    int x, y, width, height, signature;
};
PixyBlock pixyBlock;

// Odometry parameters and variables:

// robot parameters:
// Note: for a tracked platform, all that matters is ticks per meter traveled
double wheelBaseMeters = 0.20;   // effective, not measured usually
double wheelRadiusMeters = 0.10;
double encoderTicksPerRevolution = 300.0;  // one wheel rotation

// current robot pose, updated by odometry:
double X;      // meters
double Y;      // meters
double Theta;  // radians, positive clockwise

DifferentialDriveOdometry *odometry;

//-------------------------------------- End of variable definitions -----------------------------------------//

unsigned long timer = 0;     // stores loop starting micros()

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

void setup()
{
  Serial.begin(115200);   // start serial for USB

  // DEBUG pins:
  pinMode(DBG1_PIN, OUTPUT);
  pinMode(DBG2_PIN, OUTPUT);

  ledsInit();

  motorsInit();    // motors will be stopped
  
  encodersInit();  // initialize the encoders
  
  pixyCameraInit();

  controllersInit();

  odometry = new DifferentialDriveOdometry();
  odometry->Init(wheelBaseMeters, wheelRadiusMeters, encoderTicksPerRevolution);
  
  setEmaPeriod(RightMotorChannel, EmaPeriod);
  setEmaPeriod(LeftMotorChannel, EmaPeriod);
  
  pinMode(buzzerPin, OUTPUT);
  shortBuzz();
  
  timer = micros(); 
}

// =========================== main loop timing ============================================
unsigned long STD_LOOP_TIME  = mydt * 1000;  // Fixed time loop of 5 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;  // see Kalman dt

#ifdef TRACE
const int printLoopFactor = 200;
int printLoopCnt = 0;
#endif

// =========================================================================================

void loop()
{
  // whole loop takes slightly over 3ms, with a bit less than 2ms left to idle

  loopCnt++;

#ifdef TRACE
  boolean isPrintLoop = printLoopCnt++ >= printLoopFactor;

  if(isPrintLoop)
  {
    printLoopCnt = 0;
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif

  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while((micros() - timer) < STD_LOOP_TIME)
    {
      idleTasks();  // check out Pixy camera, joystick...
    }
  }
  
  //digitalWrite(DBG1_PIN, HIGH); // start of 5ms cycle
  timer = micros(); 
  
  boolean isControlLoop = loopCnt % pidLoopFactor == 0;

  if(isControlLoop)  // we do speed control and odometry calculation on a slower scale, about 100ms / 10Hz
  {
    //digitalWrite(DBG2_PIN, HIGH); // start of 100ms cycle

    odometryProcess();

    if(!isControlByJoystick())
    {
      // when in autonomous mode:
      controlPosition();
    } else {
      controlSpeed(); // use desired speeds from joystick
    }

    pixyBlocksCount = 0;
    
    // test: At both motors set to +80 expect Ldistance and Rdistance to increase
    //pwm_R = 80.0;
    //pwm_L = 80.0;
    //pwm_L = 255.0;  // measure full speed distR and distL. See ControllerSpeed tab.

    set_motors();
  
    //digitalWrite(DBG1_PIN, LOW);
    
    digitalWrite(greenLedPin,!digitalRead(greenLedPin));  // blinking at 10/2 = 5 Hz
  }

  // we are done in about 3.4ms
  //digitalWrite(DBG1_PIN, LOW);
  //digitalWrite(DBG2_PIN, LOW);

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - timer;

  /*
  if (lastLoopUsefulTime > STD_LOOP_TIME + STD_LOOP_TIME)
  {
    DbgRed(true);
  }
  */
  
  // let operating environment work (maybe taking time) and reenter the loop()
}
