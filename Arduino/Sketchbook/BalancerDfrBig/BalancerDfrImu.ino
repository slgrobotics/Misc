
// Inertial Measurement Unit consists of a Gyro and Accelerometer

/*----------- Gyro - ITG3200 ----------------*/

int g_offx = 0;
int g_offy = 0;
int g_offz = 0;

#define ITG3200_Address 0x68

/*----------- Accelerometer - ADXL345 ----------------*/

#define DEVICE (0x53)      // ADXL345 device address
#define TO_READ (6)        // num of bytes we are going to read each time (two bytes for each axis)

byte buff[TO_READ] ;       // 6 bytes buffer for saving data read from the device
int regAddress = 0x32;     // first axis-acceleration-data register on the ADXL345


void initImu()
{
  initAccel();
  delay(100);
  initGyro();
  delay(100);
}

//
// read Accelerometer and Gyro
//
void readFromImu()
{
  AccelRead();
  
  GyroRead();
}

void initAccel()
{
  // Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
}

// reads raw data, not adjusted for calibration offset
void AccelRead()
{
  // we spend 1.14ms here
  //digitalWrite(10, HIGH);
  readFrom(DEVICE, regAddress, TO_READ, buff);   //read the acceleration data from the ADXL345
  
  //x_value = (((int)buff[1]) << 8) | buff[0]; 
  y_value = (((int)buff[3]) << 8) | buff[2]; 
  //z_value = (((int)buff[5]) << 8) | buff[4]; 
  acceleration = y_value / 267;     // current acceleration along Y-axis
  //digitalWrite(10, LOW);
}

// **************************
// I2C Gyroscope ITG3200 
// **************************
void initGyro()
{
  // Fast Mode I2C (400kHz) serial interface with adress at 0x69 or 0x68
  
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x3E);  
   Wire.write(0x00);   
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x15);  
   Wire.write(0x07);   
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x16);  
   Wire.write(0x1E);   // +/- 2000 dgrs/sec, 1KHz, 1E, 19
   Wire.endTransmission(); 
   
   Wire.beginTransmission(ITG3200_Address); 
   Wire.write(0x17);  
   Wire.write(0x00);   
   Wire.endTransmission(); 
}

void GyroCalibrate()
{
  int tmpx = 0;
  int tmpy = 0;
  int tmpz = 0; 

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
  // this method is called at 200Hz and takes 1.3ms to complete
  // Fast Mode I2C (400kHz) serial interface with adress at 0x69 or 0x68
  
  //digitalWrite(10, HIGH);
  Wire.beginTransmission(ITG3200_Address); 
  Wire.write(0x1B);       
  Wire.endTransmission(); 
  
  Wire.beginTransmission(ITG3200_Address); 
  Wire.requestFrom(ITG3200_Address, 8);    // request 8 bytes from ITG3200
  
  int i = 0;
  byte buff[8];
  while(Wire.available())    
  { 
    buff[i] = Wire.read(); 
    i++;
  }
  
  Wire.endTransmission(); 
    
  GyroX = ((buff[4] << 8) | buff[5]) - g_offx;
  GyroY = ((buff[2] << 8) | buff[3]) - g_offy;
  GyroZ = ((buff[6] << 8) | buff[7]) - g_offz;
  //digitalWrite(10, LOW);
  
  //digitalWrite(11, HIGH);
  // temperature calculation takes 52us:
  int temperature = ((int) buff[0] << 8) | ((int) buff[1]);
  
  // see http://developer.mbed.org/users/aberk/code/ITG3200/docs/b098d99dd81e/ITG3200_8cpp_source.html  ITG3200::getTemperature(void)
  // Offset = -35 degrees, 13200 counts. 280 counts/degrees C.
  GyroTemp = 35.0 + (temperature + 13200)/280.0;
  //digitalWrite(11, LOW);
}

