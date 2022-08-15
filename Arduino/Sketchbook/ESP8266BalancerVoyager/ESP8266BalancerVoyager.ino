
/*
 * 
 * This is a quick compilation of the code from https://github.com/simplefoc/Arduino-FOC-balancer
 * and PID and Filter code from core SimpleFOC library.
 * 
 * adapted 07/2022 for ESP8266 NodeMCU - Sergei Grichine
 * tested on a hoverboard with RioRand 400W BLDC controllers
 * 
 * repository at: https://github.com/slgrobotics/Misc/tree/master/Arduino/Sketchbook/ESP8266BalancerVoyager
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
 * Author: Sergei Grichine, August 2022
 */

#include "imu_helpers.h"
#include "lowpass_filter.h"
#include "pid.h"

// see https://github.com/nodemcu/nodemcu-devkit-v1.0
//     https://github.com/simplefoc/Arduino-FOC-balancer

// NodeMCU has weird pin mapping.
// Pin numbers written on the board itself do not correspond to ESP8266 GPIO pin numbers.

#define D0 16 // Connected to LED. GPIO16 does support PWM.
#define D1 5  // I2C Bus SCL (clock)
#define D2 4  // I2C Bus SDA (data)
#define D3 0
#define D4 2  // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO 
#define D7 13 // SPI Bus MOSI / RXD2
#define D8 15 // SPI Bus SS (CS) / TXD2
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

#define SD1 8   // SDD1
#define SD0 7   // SDD0, MISO
#define SD2 9   // SDD2 - do not use!
#define SD3 10  // SDD3

#define SDCLK 6  // SDCLK, CLK
#define SDCMD 11 // SDCMD, CMD

// See https://www.electronicwings.com/nodemcu/nodemcu-spi-with-arduino-ide
// Note: D8/HCS - boot fails if it is pulled high, see https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
//If you want to use NodeMCU pin 5, use D5 for pin number, and it will be translated to 'real' GPIO pin 14.

// Built in LED:
const int pinLED = D0;  // HiLetgo, NodeMCU D0
//const int pinLED = 0;   // Adafruit Feather HUZZAH 

#include <Shifty.h>

// uses Shifty Arduino library
//      https://www.arduino.cc/reference/en/libraries/shifty/
//      https://github.com/johnnyb/Shifty
//

// Declare the shift register (74HC595)
Shifty shift; 

const int latchPin = D0;  // Latch pin 12 of 74HC595
const int clockPin = D3;  // Clock pin 11 of 74HC595
const int dataPin = SD3;  // Data pin 14 of 74HC595

float pitch;      // Optimal (estimated) angle from the vertical, degrees, positive when tilting backwards

// control algorithm parameters
// stabilization PID:
PIDController pid_stb{.P = 30, .I = 100, .D = 1, .ramp = 100000, .limit = 7};
// velocity PID:
PIDController pid_vel{.P = 0.01, .I = 0.03, .D = 0, .ramp = 10000, .limit = M_PI / 10};

// velocity control filtering:
LowPassFilter lpf_pitch_cmd{.Tf = 0.07};

// low pass filters for user commands - throttle and steering
LowPassFilter lpf_throttle{.Tf = 0.5};
LowPassFilter lpf_steering{.Tf = 0.1};

// Bluetooth app variables
float steering = 0.0; // -100..100
float throttle = 0.0; // -100..100
const float max_throttle = 20; // 20 rad/s
const float max_steering = 1;  // 1 V
int state = 1; // 1 on / 0 off

// measured by encoders:
float motorR_shaft_velocity = 0.0;
float motorL_shaft_velocity = 0.0;

// Wheels torque, expressed by PWM -255..255:
float pwm_R = 0.0;
float pwm_L = 0.0;
const float pwmFactor = 30.0; // amplify PID output -7..7 to turn it to PWM -255..255

// motor pins - PWM and Direction:
// we command PWM directly, and dir/stop/brake via shift register
const int rightPwmPin = D8;    // Speed Control      (RightMotorPWM, 1kHz)
const int leftPwmPin = D4;     // Speed Control      (LeftMotorPWM,  1kHz)

const int testBit = 0;        // bit 0 connected to yellow LED for testing
const int rightDirBit = 1;    // direction Control  (Right Motor)
const int leftDirBit = 2;     // direction Control  (Left Motor)
const int stopBit = 3;        // feathers both wheels
const int brakeBit = 4;       // applies moderate braking force to both motors
const int rightLedBit = 5;    // built-in LED in the hoverboard
const int leftLedBit = 6;     // built-in LED in the hoverboard
const int buzzerBit = 7;      // small 5V buzzer

// Encoder interrupt pins:
const int rightEncoderPin = D6;
const int leftEncoderPin = D5;

float batteryVoltage = 0.0;   // measured by ADC

const boolean doTRACE = true;

void setup() 
{
  Serial.begin(115200);
  delay(1000);

  // imu init and configure
  if ( !initIMU() ) {
    Serial.println(F("IMU connection problem... Disabling!"));
    //bluetooth.println(F("IMU connection problem... Disabling!"));
    return;
  }
  delay(1000);

  // Set the number of bits you have (multiples of 8)
  shift.setBitCount(8);

  // Set the data, clock, and latch pins you are using
  // This also sets the pinMode for these pins
  shift.setPins(dataPin, clockPin, latchPin); 

  shift.batchWriteBegin();
  shift.writeBit(rightLedBit, LOW);
  shift.writeBit(leftLedBit, LOW);
  shift.writeBit(buzzerBit, LOW);
  shift.batchWriteEnd();

  initMotors();

  initEncoders();

  initRemote();
}

unsigned long lastMeasuredSpeed = 0;
const int measureSpeedIntervalMs = 20;

unsigned long lastPrintTime = 0;
const int printIntervalMs = 1000;

bool ledState = true;

void loop() 
{
  remote();  // see if throttle and steering came from bluetooth (BLE-LINK) serial

  unsigned long _now = millis();

  if(hasDataIMU()) // when IMU has received the package
  {
    // read pitch from the IMU, radians:
    pitch = getPitchIMU();

    if(abs(pitch) > 0.6)
    {
      // tilted too much, stop
      pwm_R = pwm_L = 0;
      set_motors();
      return;
    }

    // calculate the target angle for throttle control
    float target_pitch = lpf_pitch_cmd(pid_vel((motorR_shaft_velocity + motorL_shaft_velocity) / 2 - lpf_throttle(throttle/50.0)));
    
    // calculate the target voltage
    float voltage_control = pid_stb(target_pitch - pitch);
    
    // filter steering
    float steering_adj = lpf_steering(-steering/300.0);
    
    // set the target voltage value
    pwm_R = (voltage_control + steering_adj) * pwmFactor;
    pwm_L = (voltage_control - steering_adj) * pwmFactor;

    set_motors();
  }

  if(_now - lastMeasuredSpeed >= measureSpeedIntervalMs)
  {
    lastMeasuredSpeed = _now;
    measureSpeed();
  }

  if(_now - lastPrintTime >= printIntervalMs)
  {
    lastPrintTime = _now;

    ledState = !ledState;
    shift.writeBit(testBit, ledState ? HIGH : LOW);

    batteryVoltage = getBatteryVoltage();

    if(batteryVoltage > 1.0)  //  // Battery main switch could be off, and we are on USB power
    {
      if(batteryVoltage < 37.0)
      {
        shortBuzz();
        Serial.print("Error: Battery voltage too low: ");
        Serial.println(batteryVoltage);
      }
    }

    if(doTRACE)
    {
      printAll();
    }
  }
}

// Wheels velocity filtering:
LowPassFilter lpf_vel_R{.Tf = 0.040};
LowPassFilter lpf_vel_L{.Tf = 0.040};

const float clicksPerTurn = 96.0;  // encoder clicks per one wheel rotation
long lastMeasureRun = 0;

// encoders - distance traveled:
volatile int Rdistance = 0;
volatile int Ldistance = 0; 

int Rdistance_prev = 0;
int Ldistance_prev = 0; 

void measureSpeed()
{
  long t_current = millis();

  if(lastMeasureRun == 0)
  {
    lastMeasureRun = t_current;
    return;
  }

  float t_elapsed = (t_current - lastMeasureRun) / 1000.0;  // seconds
  lastMeasureRun = t_current;
  
  // scale encoder clicks to rad/sec. 96 clicks per 1 turn, task runs at 20ms
  float factor = M_PI / clicksPerTurn / t_elapsed;
  
  motorR_shaft_velocity = lpf_vel_R(Rdistance - Rdistance_prev) * factor; // radians per second
  motorL_shaft_velocity = lpf_vel_L(Ldistance - Ldistance_prev) * factor;

  Rdistance_prev = Rdistance = 0;   // avoid int overfill
  Ldistance_prev = Ldistance = 0;

  // For PID adjustments, set velocities to 0:
  //motorR_shaft_velocity = 0.0;
  //motorL_shaft_velocity = 0.0;
}

void printAll()
{
    Serial.print("Pitch: ");
    Serial.print(pitch);
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
    Serial.print("   Voltage: ");
    Serial.print(batteryVoltage);
    Serial.print("   throttle: ");
    Serial.print(throttle);
    Serial.print("   steering: ");
    Serial.println(steering);
}

float getBatteryVoltage()
{
  // a voltage divider - 82K and 6.8K cuts 42V into a 3.3V range
  int analogVal = analogRead(A0) - 8; // reads 8 when grounded
  //Serial.print("   A0 read: ");
  //Serial.println(analogVal);
  return 41.6 * ((float)analogVal) / 926.0; // reads 926 when voltage is 41.6V
}

void shortBuzz()
{
  shift.writeBit(buzzerBit, HIGH);
  delay(100);  
  shift.writeBit(buzzerBit, LOW);
}
