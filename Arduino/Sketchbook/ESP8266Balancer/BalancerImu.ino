
// BMI160 Inertial Measurement Unit consists of a Gyro and Accelerometer

// use https://github.com/hanyazou/BMI160-Arduino - install manually! EmotiBit_BMI160 does not work!
// Works on UNO and ESP8266 in SPI or I2C mode.

#include <BMI160Gen.h>

const int BMI160_i2c_addr = 0x69;  // see I2cdetect library example scan

int axRaw, ayRaw, azRaw;         // raw accelerometer values
int gxRaw, gyRaw, gzRaw;         // raw gyro values

// results of gyro calibration:
float  g_offx = 0;
float  g_offy = 0;
float  g_offz = 0;

void initImu()
{
  Serial.println("IP: initializing BMI160 IMU device");

  BMI160.begin(BMI160GenClass::I2C_MODE, BMI160_i2c_addr);
  Serial.print("BMI160.begin finished");

  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // Set the accelerometer range to 250 degrees/second
  BMI160.setAccelerometerRange(2);  // supported values: 2, 4, 8, 16 (G)

  // Set the gyro range to 250 degrees/second
  BMI160.setGyroRange(250);        // supported values: 125, 250, 500, 1000, 2000 (degrees/second)
  Serial.println("OK: finished initializing IMU device");
}

//
// read Accelerometer and Gyro
//
void readFromImu()
{
  // read raw measurements from device
  BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  // convert the raw accelerometer data to G's
  AccelX = convertRawAccelerometer(axRaw);
  AccelY = convertRawAccelerometer(ayRaw);
  AccelZ = convertRawAccelerometer(azRaw);

  // convert the raw gyro data to degrees/second
  GyroX = convertRawGyro(gxRaw) - g_offx;
  GyroY = convertRawGyro(gyRaw) - g_offy;
  GyroZ = convertRawGyro(gyRaw) - g_offz;
}

void GyroCalibrate()
{
  float  tmpx = 0;
  float  tmpy = 0;
  float  tmpz = 0; 

  g_offx = 0;
  g_offy = 0;
  g_offz = 0;
 
  for (char i = 0; i < 10 ;i++)
  {
    delay(10);
  
    GyroRead();

    tmpx += GyroX;
    tmpy += GyroY;
    tmpz += GyroZ; 
  }

  g_offx = tmpx/10;
  g_offy = tmpy/10;
  g_offz = tmpz/10;
}

void GyroRead()
{
  BMI160.readGyro(gxRaw, gyRaw, gzRaw);
    
  GyroX = convertRawGyro(gxRaw) - g_offx;
  GyroY = convertRawGyro(gyRaw) - g_offy;
  GyroZ = convertRawGyro(gyRaw) - g_offz;
}

float convertRawAccelerometer(int aRaw) 
{
  // since we are using 2 G's range
  // -2 maps to a raw value of -32768
  // +2 maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

float convertRawGyro(int gRaw) 
{
  // since we are using 125 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}
