
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
  readFrom(DEVICE, regAddress, TO_READ, buff);   //read the acceleration data from the ADXL345
  
  //x_value = (((int)buff[1]) << 8) | buff[0]; 
  y_value = (((int)buff[3]) << 8) | buff[2]; 
  //z_value = (((int)buff[5]) << 8) | buff[4]; 
  acceleration = y_value / 267;     // current acceleration along Y-axis
}

// **************************
// I2C Gyroscope ITG3200 
// **************************
void initGyro()
{
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
  GyroTemp = (buff[0] << 8) | buff[1];         // temperature 
}

