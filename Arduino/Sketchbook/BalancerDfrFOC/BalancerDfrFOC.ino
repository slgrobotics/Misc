
/*
 * 
 * This is a quick compilation of the code from https://github.com/simplefoc/Arduino-FOC-balancer
 * and PID and Filter code from core SimpleFOC library.
 * 
 * adapted 08/2022 for Arduino Uno with DFRobot motor/IMU shield (small Balancer) - Sergei Grichine
 * 
 * repository at: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook
 * 
 * copy rights no more restrictive than the code cited above as follows:
 * 
    MIT License
    
    Copyright (c) 2020 SimpleFOCproject
    
    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:
    
    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.
    
    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 * 
 */

#include "lowpass_filter.h"
#include "pid.h"

#include <Wire.h>
#include <SoftTimer.h>

//-------------------------------------- Variable definitions ---------------------------------------------//

// raw variables delivered by the IMU:
float GyroX, GyroY, GyroZ, GyroTemp;
float AccelX, AccelY, AccelZ;

// variables involved in calculating motor inputs:
float acceleration;  // current acceleration along Y-axis from IMU, 0.0 when vertical, <0 when tilted forward 
float pitchByAccel;  // degrees, 0.0 when vertical, <0 when tilted forward
const float pitchAdjust = -5.0;  // to offset pitch measurement passed to Kalman filter, degrees
float target_pitch; // calculated by velocity PID, radians

// variables calculated by the Kalman filter:
float pitch;      // Optimal (estimated) angle from the vertical, degrees, positive when tilting backwards
float pitch_dot;  // Optimal (estimated) angular velocity, positive when turning backwards

// control algorithm parameters
// stabilization PID:
//PIDController pid_stb{.P = 30, .I = 100, .D = 1, .ramp = 100000, .limit = 7}; // original
//PIDController pid_stb{.P = 25, .I = 50, .D = 0.9, .ramp = 100000, .limit = 7};
//PIDController pid_stb{.P = 20, .I = 50, .D = 0.8, .ramp = 0, .limit = 7};
PIDController pid_stb{.P = 20, .I = 10, .D = 0.8, .ramp = 0, .limit = 7};

// velocity PID:
PIDController pid_vel{.P = 0.01, .I = 0.03, .D = 0, .ramp = 10000, .limit = M_PI / 10}; // original
//PIDController pid_vel{.P = 0.1, .I = 0.3, .D = 0, .ramp = 10000, .limit = M_PI / 10};

// velocity control filtering:
LowPassFilter lpf_pitch_cmd{.Tf = 0.07};

// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf = 0.5};
LowPassFilter lpf_steering{.Tf = 0.1};

// Bluetooth app variables
float steering = 0.0; // -1, 0 or +1
float throttle = 0.0; // -8..8

// measured by encoders:
float motorR_shaft_velocity = 0.0;
float motorL_shaft_velocity = 0.0;

// Wheels torque, expressed by PWM -255..255:
float pwm_R = 0.0;
float pwm_L = 0.0;
const float pwmFactor = 30.0; // amplify PID output -7..7 to turn it to PWM -255..255

// Analog pins connected to potentiometers to adjust k1...k4 :
const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

// motor pins - PWM and Direction:
const int rightPwmPin = 5;    // M1 Speed Control      (RightMotorPWM, 975Hz, T0)
const int leftPwmPin = 6;     // M2 Speed Control      (LeftMotorPWM,  975Hz, T0)
const int rightDirPin = 4;    // M1 Direction Control  (RightMotorEN)
const int leftDirPin = 7;     // M2 Direction Control  (LeftMotorEN)

#define buzzer 13 // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

// SoftTimer service routines and tasks:
void taskMeasureSpeed(Task* me);
void taskMainLoop(Task* me);
void taskPrintTrace(Task* me);

Task t1(20, taskMeasureSpeed);
Task t2(5, taskMainLoop);
Task t3(1000, taskPrintTrace);

int state = 1; // 1 on / 0 off
bool ledState = true;
const boolean doTRACE = false;

void setup() 
{
  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  initImu();

  GyroCalibrate();
  delay(100);

  initMotors();

  initEncoders();

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

void taskMainLoop(Task* me)
{
  //digitalWrite(11, HIGH);

  remote();  // see if throttle and steering came from bluetooth (BLE-LINK) serial

  readFromImu();

  if(calculatePitch())  // must be on the fast loop to keep track of accel/gyro all the time
  {
    // we come here at 3ms with pitch and pitch_dot calculated by Kalman filter
    state = 1;

    // calculate the target pitch for throttle control - set it to 0 when tuning stabilizing PID:
    target_pitch = lpf_pitch_cmd(pid_vel((motorR_shaft_velocity + motorL_shaft_velocity) / 2.0 - lpf_throttle(throttle/10.0)));
    
    // calculate the target voltage
    float voltage_control = pid_stb(target_pitch - pitch / 57.3); // back to radians here
    
    // filter steering
    float steering_adj = lpf_steering(steering);
    
    // set the target voltage (torque) values:
    pwm_R = (voltage_control + steering_adj) * pwmFactor;
    pwm_L = (voltage_control - steering_adj) * pwmFactor;

    // test: At both motors set to +80 expect Ldistance and Rdistance to show 6...7 at 2ms delay below
    //pwm_R = 80;
    //pwm_L = 80;
  }
  else
  {
    // pitch outside limits - stop
    state = 0;
    pwm_R = 0;
    pwm_L = 0;
  }

  set_motors();  // takes 0.12ms

  //digitalWrite(11, LOW);
}

// Wheels velocity filtering:
LowPassFilter lpf_vel_R{.Tf = 0.040};
LowPassFilter lpf_vel_L{.Tf = 0.040};

const float clicksPerTurn = 1400.0;  // encoder clicks per one wheel rotation
long lastMeasureRun = 0;

// encoders - distance traveled:
volatile int Rdistance = 0;
volatile int Ldistance = 0; 

int Rdistance_prev = 0;
int Ldistance_prev = 0; 

void taskMeasureSpeed(Task* me)
{
  long t_current = millis();

  if(lastMeasureRun == 0)
  {
    lastMeasureRun = t_current;
    return;
  }

  float t_elapsed = (t_current - lastMeasureRun) / 1000.0;  // seconds
  lastMeasureRun = t_current;
  
  // scale encoder clicks to rad/sec. 1400 clicks per 1 turn, task runs at 20ms
  float factor = M_PI / clicksPerTurn / t_elapsed;
  
  motorR_shaft_velocity = lpf_vel_R(Rdistance - Rdistance_prev) * factor; // radians per second
  motorL_shaft_velocity = lpf_vel_L(Ldistance - Ldistance_prev) * factor;

  Rdistance_prev = Rdistance = 0;   // avoid int overfill
  Ldistance_prev = Ldistance = 0;

  // For PID adjustments, set velocities to 0:
  //motorR_shaft_velocity = 0.0;
  //motorL_shaft_velocity = 0.0;
}

void taskPrintTrace(Task* me)
{
    Serial.print("acceleration: ");
    Serial.print(acceleration);
    Serial.print("    pitchByAccel: ");
    Serial.println(pitchByAccel);

    Serial.print("Pitch: measured: ");
    Serial.print(pitch);
    Serial.print(" target: ");
    Serial.print(target_pitch * 57.3);
    Serial.print("     pwm  R: ");
    Serial.print(pwm_R);
    Serial.print("   L: ");
    Serial.print(pwm_L);
    Serial.print("     Encoders:  R: ");
    Serial.print(Rdistance);
    Serial.print("   L: ");
    Serial.print(Ldistance);
    Serial.print("     Velocity:  R: ");
    Serial.print(motorR_shaft_velocity);
    Serial.print("   L: ");
    Serial.print(motorL_shaft_velocity);
    Serial.print("   throttle: ");
    Serial.print(throttle);
    Serial.print("   steering: ");
    Serial.println(steering);
}

void shortBuzz()
{
  digitalWrite(buzzer,HIGH);
  delay(100);  
  digitalWrite(buzzer,LOW);
}
