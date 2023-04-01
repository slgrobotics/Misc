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

const int PWM1 = 5;    // DAC via PWM, 975Hz, Timer 0

int ret;

void setup()
{
  //pinMode(PWM1, OUTPUT);

  Fastwire::setup(400, 0);
  Serial.begin(115200);
  ret = mympu_open(200);
  Serial.print("MPU init: "); Serial.println(ret);
  Serial.print("Free mem: "); Serial.println(freeRam());
  delay(100);

}

unsigned int c = 0; //cumulative number of successful MPU/DMP reads
unsigned int np = 0; //cumulative number of MPU/DMP reads that brought no packet back
unsigned int err_c = 0; //cumulative number of MPU/DMP reads that brought corrupted packet
unsigned int err_o = 0; //cumulative number of MPU/DMP reads that had overflow bit set

void loop() {
  ret = mympu_update();
  errorReporting(); // turn on for debug information

  if (!(c % 25)) { // output only every 25 MPU/DMP reads

    double yaw = mympu.ypr[0];

    double yawc = map(yaw,-180.0,180.0,0.0,255.0);

    //analogWrite(PWM1, (int)round(yawc));

    Serial.print("Yaw: "); Serial.print(yaw);
    //Serial.print(" PWM: "); Serial.print(yawc);
    Serial.print(" Pitch: "); Serial.print(mympu.ypr[1]);
    Serial.print(" Roll: "); Serial.print(mympu.ypr[2]);
    //	    Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
    //	    Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
    //	    Serial.print(" gr: "); Serial.print(mympu.gyro[2]);
    Serial.println();
    delay(100);

  }
}

void errorReporting() {
  if (ret != 0) {
    switch (ret) {
      case 0: c++; break;
      case 1: np++; return;
      case 2: err_o++; return;
      case 3: err_c++; return;
      default:
        Serial.print("READ ERROR!  ");
        Serial.println(ret);
    }
    Serial.print(np);
    Serial.print("  ");
    Serial.print(err_c);
    Serial.print(" ");
    Serial.print(err_o);
    Serial.print(" ");
  }
}
