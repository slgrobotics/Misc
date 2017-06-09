//#define TRACE

#include <Wire.h>
//#include <stdint.h> // Needed for uint8_t

#include <Odometry.h>

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
volatile long oLdistance, oRdistance; // encoders - distance traveled, used by odometry calculator

int pwm;
int pwm_R, pwm_L;  // used by set_motor_power()
double range = 0.0;
double direction_error_all = 0.0;  // accumulates Ldistance - Rdistance 
double wheel_speed;
double last_wheel;
double error_a = 0.0;

// robot parameters:
double wheelBaseMeters = 0.570;
double wheelRadiusMeters = 0.1805;
double encoderTicksPerRevolution = 1460;  // one wheel rotation

// current robot pose, updated by odometry:
double X;      // meters
double Y;      // meters
double Theta;  // radians, positive clockwise

DifferentialDriveOdometry *odometry;

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

// motor pins - PWM connected to DBH-1B H-bridge pins, all must be PWM capable:
//const int A_IN1 = 6;     // A-IN1 Speed Control  (Left Motor PWM)
//const int A_IN2 = 7;     // A-IN2 Speed Control  (Left Motor PWM)
//const int B_IN2 = 44;    // B-IN2 Speed Control  (Right Motor PWM)
//const int B_IN1 = 45;    // B-IN1 Speed Control  (Right Motor PWM)
//
//int DIS_AB = 12;  			// HIGH on DIS disables both sides. 

// H-Bridge Side A:
const int EN_A   = 5;
const int RPWM_A = 6;
const int LPWM_A = 7;
//const boolean reverse_A = false;

// H-Bridge Side B:
const int EN_B   = 4;
const int RPWM_B = 44;
const int LPWM_B = 45;
//const boolean reverse_B = true;

//#define ATHB_MOTORS_ENABLE 	LOW
//#define ATHB_MOTORS_DISABLE	HIGH

#define buzzer 13 // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

// debugging LEDs:
const int RED_LED_PIN   = 33;
const int GREEN_LED_PIN = 35;
const int BLUE_LED_PIN  = 31;

double angle_prev;

//-------------------------------------- End of variable definitions -----------------------------------------//

void setup()
{
// debugging LEDs:
  pinModeFast(RED_LED_PIN, OUTPUT);
  pinModeFast(BLUE_LED_PIN, OUTPUT);
  pinModeFast(GREEN_LED_PIN, OUTPUT);

  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);   // start serial for USB
  //delay(300);
  Serial3.begin(115200);  // start serial for BLE Link

  // enable pull-up resistor on RX3 pin to help bluetooth module with signal levels:
  //pinMode(15, INPUT);  
  //digitalWrite(15, HIGH);

  initImu();

  GyroCalibrate();
  delay(100);

  // set mode for H-bridge pins, "PWM" ones must be PWM capable:
  
  // Note: PWM pins cannot be controlled by Fast library. They remain in PWM mode.
  
  //pinModeFast(DIS_AB,   OUTPUT);
  pinModeFast(EN_A,   OUTPUT);
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinModeFast(EN_B,   OUTPUT);
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);

  //digitalWriteFast(DIS_AB, HIGH);  // disable for now;
  
  setTimerForPWM();

  StopMotors();    // make sure motors are stopped

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  //The balance PID Parameters which can read from The ADPin0-3
  //  k1 = 24.80;
  //  k2 = 9.66;
  //  k3 = 4.14;
  //  k4 = 0.99;

  // best experimental values:
  //  k1 = 16.57;
  //  k2 = 2.45;
  //  k3 = 1.82;
  //  k4 = 0.79;

  k1 = 0;
  k2 = 0;
  k3 = 0;
  k4 = 0;

  EncodersInit();	// Initialize the encoders
  
  odometry = new DifferentialDriveOdometry();
  odometry->Init(wheelBaseMeters, wheelRadiusMeters, encoderTicksPerRevolution);
  
  pinMode(buzzer, OUTPUT);
  shortBuzz();
}

// =========================== main loop timing ============================================
#define STD_LOOP_TIME 5000 // Fixed time loop of 5 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;  // see Kalman dt
unsigned long loopStartTime;
int loopCnt = 0;

const int slowLoopFactor = 40;
int slowLoopCnt = slowLoopFactor;    // should fire on first loop to read k* values

#ifdef TRACE
const int printLoopFactor = 200;
int printLoopCnt = 0;
#endif

const int pidLoopFactor = 10;
// =========================================================================================

void loop()
{
  // whole loop takes slightly over 3ms, with a bit less than 2ms left to idle
  
  // Wait here, if not time yet - we use a time fixed loop
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while((micros() - loopStartTime) < STD_LOOP_TIME)
    {
      //Usb.Task();
      ;
    }
  }
  
  //digitalWrite(10, HIGH);
  loopStartTime = micros(); 
  
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

  if(isPidLoop)  // we do odometry calculation on a slower scale, about 20Hz
  {  
    // odometry calculation takes 28us
    //digitalWrite(10, HIGH);
    // process encoders readings into X, Y, Theta using odometry library:
    odometry->wheelEncoderLeftTicks = oLdistance;
    odometry->wheelEncoderRightTicks = oRdistance;
  
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
  }

  // see if a command came over Serial3 (bluetooth link)
  RemoteControl();

  // we come here in 5us if no serial is available

  readFromImu();

  if(Angle_calculate())  // must be on the fast (5ms) loop to keep track of accel/gyro all the time. We spend 470-480us here
  {
    // we come here at 3ms

    if(isPidLoop)  // we do PID->pwm calculation on a slower scale, about 20Hz
    {  
      pwm_calculate(); // takes 0.14ms

      // test: At both motors set to +80 expect Ldistance and Rdistance to show 6...7 at 2ms delay below
      //pwm_R = 80;
      //pwm_L = 80;

      set_motor_power();  // takes 0.12ms
    }
  }
  else
  {
    // angle outside limits - stop
    pwm_R = 0;
    pwm_L = 0;

    set_motor_power();  // takes 0.12ms
    shortBuzz();
  }

  // we are done in about 3.4ms
  //digitalWrite(11, LOW);

  // mark how much time we spent:
  lastLoopUsefulTime = micros() - loopStartTime;
  loopCnt++;
  
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
  
  // let operating environment work (maybe taking time) and reenter the loop()
  //digitalWrite(10, LOW);
}


