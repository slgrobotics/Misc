/* ------------- Motion Plug Demo Sketch -------------

Code for the Modern Device Motion Plug, which uses the MPU 9250 9-axis accelerometer/gyro/compass chip from Invensense.

Lightly modified version of AvrCopter.ino from rpicopter: https://github.com/rpicopter/ArduinoMotionSensorExample

Note: This is a 3.3v board. If you are using a 5 volt Arduino (or compatible),
plug the + line into 3.3v and use series resistors on SDA and SCL for voltage signal level shifting.
The standard value 10k resistors won't work with high speed I2C, but we found that 4.7k and 3.3k work fine.

Features:
  - uses FastWire and I2Cdev by Jeff Rowberg
  - DMP enabled
  - calculates and displays gyro and quaternions
  - This has been tested using Arduino 1.0.5, 1.6.1,
    a few variants of Arduino Pro Mini boards (atmega328), the Modern Device Robot Board Rev E. (atmega2560),
    Arduino Mega (atmega2560), the BBLeo (atmega32u4), and the JeeNode SMD (atmega328).

- This library specifies the MPU9250.
  If you have an MPU6050, MPU6500, or MPU9150,
  change #DEFINE MPU9250 to #DEFINE MPU6050,
  #DEFINE MPU6500, or #DEFINE MPU9150, respectively in inv_mpu.h.

- Default chip configuration values, from inv_mpu.h.
  Pulled out here for reference:
    test->reg_rate_div   = 0;    // 1kHz.
    test->reg_lpf        = 1;    // 188Hz.
    test->reg_gyro_fsr   = 0;    // 250dps.
    test->reg_accel_fsr  = 0x18; // 16g.
*/

#include "freeram.h"
#include "mpu.h"
#include "I2Cdev.h"

const int PWM1 = 5;    // DAC via PWM, 975Hz, Timer 0. Connected: R=4.7KOhm, C=10uf (50ms RC time)
const int adjustPotPin = A1;    // select the input pin for the potentiometer - adjusts sensitivity
int ledPin = 13;      // select the pin for the LED

boolean doTRACE = false;

const int mydt = 5;   // 5 milliseconds make for 200 Hz operating cycle
long timer = 0;       // general purpose timer

unsigned int loopCnt = 0;
unsigned int lastLoopCnt = 0;

int ret;

void setup()
{
  pinMode(ledPin, OUTPUT);
  //pinMode(PWM1, OUTPUT);

  Fastwire::setup(400, 0);
  Serial.begin(115200);

  ret = mympu_open(200);
  Serial.print("MPU init: "); Serial.println(ret);
  Serial.print("Free mem: "); Serial.println(freeRam());

  timer = millis();
  loopCnt = 0;
  delay(100);
}

int adjustPotValue = 0;  // variable to store the value coming from the POT, 0..1023

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

double yaw = 0.0;
double yawPrev = 0.0;
double yawDelta = 0.0;
double yawc = 0.0;

// emulating ADXR613 - "full range" sensitivity 150 degrees per second (standard for Turtlebot), ADXR610 - 300 dps, ADXR652 - 250 dps, ADXR642 - 300 dps  
double scaleFactor = 12.0;  // for 15 degrees in 1/10 sec to become 180, so that sensitivity is 150 degrees/second as in ADXR613.
 
void loop() {

  //doTRACE = true;

  if((millis() - timer) >= mydt)  // Main loop runs at 200Hz (mydt=5), to allow frequent sampling of AHRS
  {
    loopCnt++;
    timer = millis();

    ret = mympu_update();
    errorReporting(); // turn on for debug information
  
    if (!(loopCnt % 20)) { // output only every 20 MPU/DMP reads, 10Hz
  
      // read the value from the sensor:
      adjustPotValue = analogRead(adjustPotPin);  // 0 to 1023, center = 512
  
      yaw = mympu.ypr[0]; // degrees, -180.0,180.0 range. Based on direction at start, 0.0 there.

      yawDelta = yaw - yawPrev; // degrees per 1/10 sec

      if(yawDelta > 180)
      {
        // crossing counterclockwise:  -170 => +170
        // yawDelta = 340, must be -20
        yawDelta -= 360; 
      }
      else if(yawDelta < - 180)
      {
        // crossing clockwise:  170 => -170
        // yawDelta = -340, must be +20
        yawDelta += 360; 
      }

      // TODO: figure out how to mix adjustPotValue into yawc
      yawDelta = yawDelta * scaleFactor * (double)adjustPotValue / 512.0;
      yawDelta = constrain(yawDelta,-180.0,180.0);

      yawc = map(yawDelta,-180.0,180.0,0.0,255.0);
  
      analogWrite(PWM1, (int)round(yawc));
  
      if(doTRACE)
      {
        printAll();
      }

      yawPrev = yaw;
    }
  }
}

