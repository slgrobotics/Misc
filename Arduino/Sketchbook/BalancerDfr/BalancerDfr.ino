
/*
 * 
 * This is an adaptation of the original 2013 DFRobot balancer sample code from
 * 
 *  https://www.dfrobot.com/index.php?route=product/product&filter_name=KIT0040&product_id=1043#.Uq1axWeA25P
 * 
 * Works on Arduino Uno with DFRobot motor/IMU shield (small Balancer)
 * 
 * repository at: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/BalancerDfr
 * 
 * Author: Sergei Grichine, August 2022
 */
#include <Wire.h>
#include <SoftTimer.h>

//-------------------------------------- Variable definitions ---------------------------------------------//

// raw variables delivered by the IMU:
float GyroX, GyroY, GyroZ, GyroTemp;
float AccelX, AccelY, AccelZ;

float k1, k2, k3, k4;   // Adjustable PID Parameters

// variables involved in calculating motor inputs:
float acceleration;  // current acceleration along Y-axis from IMU, 0.0 when vertical, <0 when tilted forward 
float pitchByAccel;  // degrees, 0.0 when vertical, <0 when tilted forward
const float pitchAdjust = 0.0;  // to offset pitch measurement passed to Kalman filter, degrees
float target_pitch; // calculated by velocity PID, radians

// variables calculated by the Kalman filter:
float pitch;      // Optimal (estimated) angle from the vertical, degrees, positive when tilting backwards
float pitch_dot;  // Optimal (estimated) angular velocity, positive when turning backwards

// comes from Android app - from serial via Bluetooth BLE-LINK:
float throttle = 0.0;
float steering=0.0;

volatile int Ldistance, Rdistance; // encoders - distance traveled

int pwm;
int pwm_R, pwm_L;
float range;
float direction_error_all;
float distance_per_cycle;
float wheel_speed;
float distance_setpoint_remote = 0.0;

#define TiltChannel          0   // for Ema
#define SpeedChannel         1

float tiltEma = 0.0;
float speedEma = 0.0;

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

// motor pins - PWM and Direction:
const int E1 = 5;    // M1 Speed Control      (RightMotorPWM, 975Hz, T0)
const int E2 = 6;    // M2 Speed Control      (LeftMotorPWM,  975Hz, T0)
const int M1 = 4;    // M1 Direction Control  (RightMotorEN)
const int M2 = 7;    // M2 Direction Control  (LeftMotorEN)

#define buzzer 13 // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

#define FAST_LOOP_TIME   5  // Fixed time loop of 5 milliseconds, see Kalman filter

// SoftTimer service routines and tasks:
void taskReadAnalog(Task* me);
void taskMainLoop(Task* me);
void taskPrintTrace(Task* me);

Task t1(100, taskReadAnalog);
Task t2(FAST_LOOP_TIME, taskMainLoop);
Task t3(1000, taskPrintTrace);

const boolean doTRACE = false;

//-------------------------------------- End of variable definitions -----------------------------------------//

void setup()
{
  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);   // start serial 

  initImu();

  initMotors();

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

  setEmaPeriod(TiltChannel, 100);
  setEmaPeriod(SpeedChannel, 100);

  initEncoders();	// Initialize the encoders

  initRemote();
  
  pinMode(buzzer, OUTPUT);
  shortBuzz();
  
  SoftTimer.add(&t1);
  SoftTimer.add(&t2);
  if(doTRACE)
  {
    SoftTimer.add(&t3);
  }
}

int loopCnt = 0;
const int pidLoopFactor = 4;

void taskMainLoop(Task* me)
{
  //digitalWrite(11, HIGH);

  remote();

  // we come here in 5us if no serial is available

  readFromImu();

  acceleration = AccelY / 267;     // current acceleration along Y-axis, in G's

  if(calculatePitch())  // must be on the fast loop to keep track of accel/gyro all the time
  {
    // we come here at 3ms

    if(loopCnt % pidLoopFactor == 0)  // we do PID->pwm calculation on a slower scale
    {  
      pwm_calculate(); // takes 0.14ms

      // test: At both motors set to +80 expect Ldistance and Rdistance to show 6...7 at 2ms delay below
      //pwm_R = 80;
      //pwm_L = 80;

      set_motors();  // takes 0.12ms
    }
  }
  else
  {
    // pitch outside limits - stop
    pwm_R = 0;
    pwm_L = 0;

    set_motors();  // takes 0.12ms
  }

  // we are done in about 3.4ms
  //digitalWrite(11, LOW);

  loopCnt++;
}

void taskReadAnalog(Task* me)
{
  // multipliers to have the experimental values when POTs are in neutral position:
  k1 = analogRead(AD0pin) * 0.03;
  k2 = analogRead(AD1pin) * 0.005;
  k3 = analogRead(AD2pin) * 0.0009; // * 0.0003;
  k4 = analogRead(AD3pin) * 0.0015; //* 0.0002;
}

void taskPrintTrace(Task* me)
{
  int Ldur = Ldistance;
  int Rdur = Rdistance;

  Serial.println("--------------------");

//  Serial.print("K1=");
//  Serial.print(k1);
//  Serial.print("    K2=");
//  Serial.print(k2);
//  Serial.print("    K3=");
//  Serial.print(k3);
//  Serial.print("    K4=");
//  Serial.println(k4);
//
//  Serial.print("Gyro:   X=");
//  Serial.print(GyroX);
//  Serial.print("   Y=");
//  Serial.print(GyroY);
//  Serial.print("   Z=");
//  Serial.print(GyroZ);
//  Serial.print("   Temp=");
//  Serial.println(GyroTemp);
//
//  Serial.print("Angle calculation:   acceleration=");
//  Serial.print(acceleration);
//  Serial.print("   pitchByAccel=");
//  Serial.print(pitchByAccel);
//  Serial.print("       After Kalman:   pitch=");
//  Serial.print(pitch);
//  Serial.print("   pitch_dot=");
//  Serial.println(pitch_dot);
//
//  Serial.print("pwm=");
//  Serial.print(pwm);
//  Serial.print("   direction_error_all=");
//  Serial.println(direction_error_all);
//
//  Serial.print("Motors:   Right pwm_R: ");
//  Serial.print(pwm_R);
//  Serial.print("       Left pwm_L: ");
//  Serial.println(pwm_L);
//
//  Serial.print("Encoders:  Right Rdistance: ");
//  Serial.print(Rdur);
//  Serial.print("       Left Ldistance: ");
//  Serial.println(Ldur);
//
  Serial.print("Remote:  throttle: ");
  Serial.print(throttle);
  Serial.print("     steering: ");
  Serial.println(steering);

  Serial.print("Ema:  tilt: ");
  Serial.print(tiltEma);
  Serial.print("       speed: ");
  Serial.print(speedEma);
  Serial.print("       pitch: ");
  Serial.println(pitch);
}

void shortBuzz()
{
  digitalWrite(buzzer,HIGH);
  delay(100);  
  digitalWrite(buzzer,LOW);
}
