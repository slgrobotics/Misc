
//#define TRACE

// BMI160 IMU - gyro and accelerometer
// https://github.com/EmotiBit/EmotiBit_BMI160
#include <BMI160Gen.h>

// converted values from BMI160:
float ax, ay, az;
float gx, gy, gz;
float temp;


void setup() 
{
  Serial.begin(115200);   // start serial for USB

  bmi160Init();

}

void loop() 
{
  processBmi160();

  printBmi160();

  delay(1000);
}
