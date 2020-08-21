//#define TRACE

#include <Wire.h>
//#include <stdint.h> // Needed for uint8_t

#include <Odometry.h>
#include <PID_v1.h>

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
                               // and http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino/

//
// this code is written for Arduino Mega 2560 and heavily relies on its serial 3,
//      timers 4 and 5, PWM pins 44,45 - see setTimerForPWM() in Motors section.
// DFrobot.com is the source of original code and their "6 DOF IMU Shield" is used here.
// Sergei Grichine -  trackroamer.com   slg@quakemap.com
//

//-------------------------------------- Variable definitions ---------------------------------------------//

// debugging LEDs:
const int redLedPin   = 33;
const int greenLedPin = 35;
const int blueLedPin  = 31;

const int buzzerPin   = 13;   // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

const int mydt = 5;           // 5 milliseconds make for 200 Hz operating cycle
const int pidLoopFactor = 20; // factor of 20 make for 100ms PID cycle

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

double GyroX, GyroY, GyroZ, GyroTemp;  // raw variables delivered by the IMU

// variables calculated by the Kalman filter:
double angle;      // Optimal (estimated) angle from the vertical, degrees, positive when tilting backwards
double angle_dot;  // Optimal (estimated) angular velocity, positive when turning backwards

double k1, k2, k3, k4;   // Adjustable PID Parameters

// variables involved in calculating motor inputs:
double acceleration;  // Acceleration in the conversion
double theta;
double y_value;       // y value of acceleration sensor

double turn_flag=0.0;          // will be set by Remote to initiate turn
double forward_factor = 0.0;   // will be set by Remote to initiate forward or backwards movement

volatile int Ldistance, Rdistance; // encoders - distance traveled, used by balance calculator for increments and zeroed often
long LdistancePrev = 0;   // last encoders values - distance traveled, in ticks
long RdistancePrev = 0;

volatile long oLdistance, oRdistance; // encoders - distance traveled, used by odometry calculator

//int pwm;
int pwm_R, pwm_L;       // pwm -255..255 sent to H-Bridge pins (will be constrained by set_motor()) 
double dpwm_R, dpwm_L;  // correction output, calculated by PID, constrained -250..250 normally, will be added to the above

double speedMeasured_R = 0;  // percent of max speed for this drive configuration.
double speedMeasured_L = 0;

double range = 0.0;
double direction_error_all = 0.0;  // accumulates Ldistance - Rdistance 
double wheel_speed;
double last_wheel;
double error_a = 0.0;

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

// Dragger robot physical parameters:
double wheelBaseMeters = 0.570;
double wheelRadiusMeters = 0.1805;
double encoderTicksPerRevolution = 1460;  // one wheel rotation

// current robot pose, updated by odometry:
double X;      // meters
double Y;      // meters
double Theta;  // radians, positive clockwise

DifferentialDriveOdometry *odometry;

// higher Ki causes residual rotation, higher Kd - jerking movement
PID myPID_R(&speedMeasured_R, &dpwm_R, &setpointSpeedR, 0.2, 0.0, 0.02, DIRECT);    // in, out, setpoint, double Kp, Ki, Kd, DIRECT or REVERSE
PID myPID_L(&speedMeasured_L, &dpwm_L, &setpointSpeedL, 0.2, 0.0, 0.02, DIRECT);

double angle_prev;

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
long lastGpsData = 0; 

char gpsChars[100];

// milliseconds from last events:
long lastComm = 0;
long lastImu = 0;
long lastPwm = 0;
long lastSonar = 0;

//-------------------------------------- End of variable definitions -----------------------------------------//

unsigned long timer = 0;     // general purpose timer
unsigned long timer_old;

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;


void setup()
{
  Wire.begin();           // join i2c bus (address optional for master)

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

  // enable pull-up resistor on RX3 pin to help bluetooth module with signal levels:
  //pinMode(15, INPUT);  
  //digitalWrite(15, HIGH);

  initImu();

  GyroCalibrate();
  delay(100);

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  k1 = 0;
  k2 = 0;
  k3 = 0;
  k4 = 0;

  MotorsInit();
  
  EncodersInit();	// Initialize the encoders
  
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
  
  //digitalWrite(10, HIGH);
  loopCnt++;
  timer_old = timer;
  timer = micros(); 
  
  boolean isSlowLoop = slowLoopCnt++ >= slowLoopFactor;
  boolean isPidLoop = loopCnt % pidLoopFactor == 0;
#ifdef TRACE
  boolean isPrintLoop = printLoopCnt++ >= printLoopFactor;
#endif

  //digitalWrite(11, HIGH);

  if(isSlowLoop)
  {
    slowLoopCnt = 0;

    // multipliers are set to the experimental values, tuned to POTs in neutral position:
    k1 = analogRead(AD0pin) * 0.015;
    k2 = analogRead(AD1pin) * 0.0025;
    k3 = analogRead(AD2pin) * 0.0009;
    k4 = analogRead(AD3pin) * 0.0015;
  }
  
#ifdef TRACE
  if(isPrintLoop)
  {
    printLoopCnt = 0;
    printAll();    // takes 34ms and completely stops the 5ms cycle
  }
#endif

  // test - controller should hold this speed:
  //desiredSpeedR = 10;
  //desiredSpeedL = 10;
    
  // smooth movement by using ema: take desiredSpeed and produce setpointSpeed
  ema(RightMotorChannel);
  ema(LeftMotorChannel);

  // compute control inputs - increments to current PWM
  myPID_R.Compute();
  myPID_L.Compute();

  if(isPidLoop)  // we do odometry calculation on a slower scale, about 20Hz
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
    
    digitalWriteFast(redLedPin, millis() - lastComm > 1000 ? HIGH : LOW);
    //digitalWriteFast(yellowLedPin, millis() - lastImu > 1000 ? HIGH : LOW);
    //digitalWriteFast(whiteLedPin, millis() - lastPwm > 1000 ? HIGH : LOW);
    digitalWriteFast(blueLedPin, millis() - lastSonar > 1000 ? HIGH : LOW);

    digitalWriteFast(greenLedPin,!digitalReadFast(greenLedPin));  // blinking at 10 Hz
  }

  // see if a command came over Serial3 (bluetooth link)
  readCommCommand();

  // we come here in 5us if no serial is available

  readFromImu();

  // we are done in about 3.4ms
  //digitalWrite(11, LOW);

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
  //digitalWrite(10, LOW);
}
