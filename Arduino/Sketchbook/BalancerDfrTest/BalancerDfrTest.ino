//===========================Device Test Code V1.0=============================================//

// The front is the USB Interface of Arduino
// Left Motor:Encoder1(6DOF_Shield),M2(Motor_driver)
// Right Motor:Encoder2(6DOF_Shield),M1(Motor_driver)
// Note:Please turn on the motor-driver's power before test!

//=============================KENT 2.26.2013==================================================//

#include <Wire.h>

//-----------------I2C BUS ------------------------------------------------------------------------------- 
#define DEVICE (0x53)      //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)
byte buff[TO_READ] ;       //6 bytes buffer for saving data read from the device
char str[512];             //string buffer to transform data before sending it to the serial port
//--------------------------------------------------------------------------------------------------------
int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345

float GyroX,GyroY,GyroZ,GyroTemp;

int g_offx = 0;
int g_offy = 0;
int g_offz = 0;

#define ITG3200_Address 0x68

float k1, k2, k3, k4; // Adjustable PID Parameters

// motor pins - PWM and Direction:
const int E1 = 5;    // M1 Speed Control      (RightMotorPWM)
const int E2 = 6;    // M2 Speed Control      (LeftMotorPWM)
const int M1 = 4;    // M1 Direction Control  (RightMotorEN)
const int M2 = 7;    // M2 Direction Control  (LeftMotorEN)

int Ldistance, Rdistance;

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

unsigned long lastPrintMillis = 0;


void setup()
{
  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);   // start serial

  initImu();

  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  
  EncodersInit();    //Initialize the encoders
}

void loop()
{
  if(millis() - lastPrintMillis > 2000)
  {
    lastPrintMillis = millis();

    GyroRead();

    // No multipliers to have raw ADC values when POTs are in neutral position:
    k1 = analogRead(AD0pin) * 1.0;
    k2 = analogRead(AD1pin) * 1.0;
    k3 = analogRead(AD2pin) * 1.0;
    k4 = analogRead(AD3pin) * 1.0;

    Serial.println("--------------------");
        
    Serial.print("K1=");
    Serial.print(k1);
    Serial.print("    K2=");
    Serial.print(k2);
    Serial.print("    K3=");
    Serial.print(k3);
    Serial.print("    K4=");
    Serial.println(k4);

    Serial.print("Gyro:   X=");
    Serial.print(GyroX);
    Serial.print("   Y=");
    Serial.print(GyroY);
    Serial.print("   Z=");
    Serial.print(GyroZ);
    Serial.print("   Temp=");
    Serial.println(GyroTemp);

    Serial.print("Encoders:  Right Rdistance: ");
    Serial.print(Rdistance);
    Serial.print("       Left Ldistance: ");
    Serial.println(Ldistance);

  }

  if(Serial.available())
  {
    char code=Serial.read();
    switch(code)
    {
    case 'R':
    case 'r':
      Rdistance = 0;
      RightMotorTest();
      delay(100);
      if(Rdistance > 10)
        Serial.print("Right Encoder OK    Rdistance: ");
      else
      {
        Serial.print("Right Encoder ERROR!    Rdistance: ");
      }
      Serial.print(Rdistance);
      Serial.println("    (expected value > 10 ticks) ****************************");
      Stop();
      break;

    case 'L':
    case 'l':
      Ldistance = 0;
      LeftMotorTest();
      delay(100);
      if(Ldistance > 10)
        Serial.print("Left Encoder OK    Ldistance: ");
      else
      {
        Serial.print("Left Encoder ERROR!    Ldistance: ");
      }
      Serial.print(Ldistance);
      Serial.println("    (expected value > 10 ticks) ****************************");
      Stop();
      break;

    case 'S':
    case 's':
      float ax, ay, az;
      readFrom(DEVICE, regAddress, TO_READ, buff); // read the acceleration data from the ADXL345
      ax = (((int)buff[1]) << 8) | buff[0];   
      ay = (((int)buff[3])<< 8) | buff[2];  
      az = (((int)buff[5]) << 8) | buff[4]; 
      GyroRead();

// Z looking down
// Y looking to the left
// X looking backwards

// robot slightly bent forward - y negative 100, z positive 200+:
// Accelerometer raw xyz: -13.00,  -106.00,  219.00,      Gyro XYZ degrees: -0.21,  -0.77,  0.14

// robot slowly rotating forward - Y negative 40:
// Accelerometer raw xyz: -6.00,  -71.00,  231.00,      Gyro XYZ degrees: 1.95,  -40.70,  -0.35

      Serial.println("--------------------------------------------");
      Serial.print("Accelerometer raw xyz: ");
      Serial.print(ax);
      Serial.print(",  ");
      Serial.print(ay);
      Serial.print(",  ");
      Serial.print(az);
      Serial.print(",      Gyro XYZ degrees: ");
      Serial.print(GyroX / 14.375); // Data to Degree conversion
      Serial.print(",  ");
      Serial.print(GyroY / 14.375);
      Serial.print(",  ");
      Serial.println(GyroZ / 14.375);
      Serial.println("--------------------------------------------");
      delay(100);
      break;
    }
  }
}

void RightMotorTest()  // The Right Motor will rotate clockwise (robot's forward direction).
{
  digitalWrite(M1, HIGH);    // direction
  analogWrite( E1, 80);      // PWM
}

void LeftMotorTest()  // The Left Motor will rotate counterclockwise (robot's forward direction).
{
  digitalWrite(M2, LOW);     // direction
  analogWrite( E2, 80);      // PWM
}

void Stop()  // Both Motors will stop.
{
  analogWrite( E2, 0);
  analogWrite( E1, 0);
}




//=====================ENCODER TEST======================//

void EncodersInit()
{
  pinMode(2, INPUT);  // left, INT 0
  pinMode(8, INPUT);  // left

  pinMode(3, INPUT);  // right, INT 1
  pinMode(9, INPUT);  // right

  // Most Arduino boards have two external interrupts: numbers 0 (on digital pin 2) and 1 (on digital pin 3)
  attachInterrupt(0, leftEncoder, CHANGE);   // int 0, pin 2  - left encoder interrupt
  attachInterrupt(1, rightEncoder, CHANGE);  // int 1, pin 3  - right encoder interrupt
}

// ************************************
//    Read distance from the encoders
// ************************************
void leftEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(10, HIGH);

  boolean vi = (PIND & _BV(PIND2)) == 0; // read pin D2 (PD2)
  boolean vd = (PINB & _BV(PINB0)) == 0; // read pin D8 (PB0)

  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Ldistance--;
  } 
  else {
    //digitalWrite(11, HIGH);
    Ldistance++;                // wheel moves forward, positive increase
  }
  //digitalWrite(10, LOW);
}

void rightEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(10, HIGH);

  boolean vi = (PIND & _BV(PIND3)) == 0; // read pin D3 (PD3)
  boolean vd = (PINB & _BV(PINB1)) == 0; // read pin D9 (PB1)

  if(vi == vd)
  {
    //digitalWrite(11, LOW);
    Rdistance++;                // wheel moves forward, positive increase
  } 
  else {
    //digitalWrite(11, HIGH);
    Rdistance--;
  }
  //digitalWrite(10, LOW);
}

//---------------- I2C Functions --------------------------------------------------------------
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        // send register address
  Wire.write(val);        // send value to write
  Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[])
{
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission

    Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

//-----------------------I2C Functions end---------------------------------------------------------

void initImu()
{
  // Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  // Turning on the ITG3200
  delay(100);
  initGyro();
  delay(100);
  GyroCalibrate();
  delay(100);
}

// **************************
// I2C Gyroscope ITG3200 
// **************************
void initGyro() {
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

void GyroCalibrate(){

  int tmpx = 0;
  int tmpy = 0;
  int tmpz = 0; 

  g_offx = 0;
  g_offy = 0;
  g_offz = 0;

  for (char i = 0;i<10;i++)
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


void GyroRead() {
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
  GyroTemp = (buff[0] << 8) | buff[1]; // temperature 


}




