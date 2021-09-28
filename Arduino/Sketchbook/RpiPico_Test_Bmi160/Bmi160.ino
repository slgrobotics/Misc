
/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read gyroscope data
*/

const int fullGyroRange = 125;  // degrees/seconds max gyro scale

// raw values are 16-bit integers, range -32767..32768
// they are scaled to chosen range in G's and degrees per second in setup
  
int axRaw, ayRaw, azRaw;        // raw accelerometer values
int gxRaw, gyRaw, gzRaw;        // raw gyro values
int tempRaw;                    // raw temperature

void bmi160Init()
{
  // initialize device
#ifdef TRACE
  Serial.println("IP: Initializing IMU device...");
#endif // TRACE
  //if(!BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10))
  if(!BMI160.begin(BMI160GenClass::I2C_MODE, Wire, /* I2C address */ 0x69, /* INT1 to pin */ 7))
  {
#ifdef TRACE
    Serial.println("Error: bmi160Init() BMI160.begin() failed");
#endif // TRACE
    //while(true)
    //  ;
    return;
  }
  uint8_t dev_id = BMI160.getDeviceID();
#ifdef TRACE
  Serial.print("BMI160 DEVICE ID: ");
  Serial.println(dev_id, HEX);
#endif // TRACE

  // data sampling rates:
  BMI160.setAccelerometerRate(200);
  BMI160.setGyroRate(800);

  // Set the accelerometer range to 2G - most sensitive
  BMI160.setAccelerometerRange(2);
  // Set the gyro range to 125 degrees/second - most sensitive
  BMI160.setGyroRange(fullGyroRange);

  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 65536 / 4);
  BMI160.autoCalibrateGyroOffset();
  
#ifdef TRACE
  Serial.println("OK: Done initializing BMI160 IMU device.");
#endif // TRACE
}

void processBmi160()
{
  // read temperature:
  tempRaw = BMI160.readTemperature();  

  // read all values:
  BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  // only accelerometer:
  //BMI160.readAccelerometer(axRaw, ayRaw, azRaw);

  // read raw gyro measurements from device:
  //BMI160.readGyro(gxRaw, gyRaw, gzRaw);

  temp = convertRawTemperature(tempRaw);

  // convert the raw accelerometer data to G's:
  ax = convertRawAccelerometer(axRaw);
  ay = convertRawAccelerometer(ayRaw);
  az = convertRawAccelerometer(azRaw);

  // convert the raw gyro data to degrees/second
  gx = convertRawGyro(gxRaw);
  gy = convertRawGyro(gyRaw);
  gz = convertRawGyro(gzRaw);
}

void printBmi160()
{
#ifdef TRACE
  if(Serial) {
    Serial.print("Temp: ");
    Serial.println(temp);
  
    // display tab-separated accelerometer x/y/z values
    Serial.print("accel:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.println();
  
    // display tab-separated gyro x/y/z values
    Serial.print("gyro:\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);
    Serial.println();
  }
#endif // TRACE

// Note: can't use sprintf for floats. print() works though.
//       https://www.e-tinkers.com/2020/01/do-you-know-arduino-sprintf-and-floating-point/

  //char buf[128];
  //sprintf(buf, "&%f\n", gz);   
  //sprintf(buf, "&%f %f %f\n", gx, gy, gz);   
  //sprintf(buf, "&%f %f %f %f %f %f\n", gx, gy, gz, ax, ay, az);   
#ifdef TRACE
  //if(Serial) {
  //  Serial.print(buf);
  //}
#endif // TRACE
  //Serial1.print(buf); 

  Serial1.print("&");
  Serial1.print(gx);
  Serial1.print(" ");
  Serial1.print(gy);
  Serial1.print(" ");
  Serial1.print(gz);
  Serial1.println();
}

float convertRawTemperature(int tempRaw)
{
  // temperature register is a 16 bits sign signal
  // 0x0000 means 23 degree C
  // each LSB is about 0.00195 degrees
  return 23.0 + (float)tempRaw * 0.00195;
}

float convertRawAccelerometer(int aRaw) 
{
  // since we are using 2G range
  // -2G maps to a raw value of -32768
  // +2G maps to a raw value of 32767

  float a = (float)aRaw / 16384.0;

  return a;
}

float convertRawGyro(int gRaw) 
{
  // since we are using 250 degrees/seconds range
  // -fullGyroRange maps to a raw value of -32768
  // +fullGyroRange maps to a raw value of 32767

  float g = ((float)gRaw * (float)fullGyroRange) / 32768.0;

  return g;
}

/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.
   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.
   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/
