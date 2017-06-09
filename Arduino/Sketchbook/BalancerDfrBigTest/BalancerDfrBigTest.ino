//===========================Device Test Code V1.0=============================================//

// The front is the USB Interface of Arduino
// Left Motor:Encoder1(6DOF_Shield),M2(Motor_driver)
// Right Motor:Encoder2(6DOF_Shield),M1(Motor_driver)
// Note:Please turn on the motor-driver's power before test!

//=============================KENT 2.26.2013==================================================//

#include <Wire.h>

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
                               // and http://www.hessmer.org/blog/2011/01/30/quadrature-encoder-too-fast-for-arduino/

// BalancerDfrBig has Arduino Mega 2560 board for a controller.

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

// motor pins - PWM connected to DBH-1B H-bridge pins:
//const int A_IN1 = 6;     // A-IN1 Speed Control  (Left Motor PWM)
//const int A_IN2 = 7;     // A-IN2 Speed Control  (Left Motor PWM)
//const int B_IN2 = 44;    // B-IN2 Speed Control  (Right Motor PWM)
//const int B_IN1 = 45;    // B-IN1 Speed Control  (Right Motor PWM)
//
//int DIS_AB = 12;  			// HIGH on DIS disables both sides. 

// H-Bridge Side A:
const int EN_A   = 5;
const int RPWM_A = 6;
const int LPWM_A = 7;
//const boolean reverse_A = false;

// H-Bridge Side B:
const int EN_B   = 4;
const int RPWM_B = 44;
const int LPWM_B = 45;
//const boolean reverse_B = true;

#define ATHB_MOTORS_ENABLE 	LOW
#define ATHB_MOTORS_DISABLE	HIGH

int Ldistance, Rdistance;

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

// debugging LEDs:
const int RED_LED_PIN   = 33;
const int GREEN_LED_PIN = 35;
const int BLUE_LED_PIN  = 31;

unsigned long lastPrintMillis = 0;

const int testPower = 50;


void setup()
{
  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);   // start serial for USB
  //delay(300);
  Serial3.begin(115200);  // start serial for BLE Link

  // enable pull-up resistor on RX3 pin to help bluetooth module with signal levels:
  //pinMode(15, INPUT);  
  //digitalWrite(15, HIGH);
  
  initImu();

  //digitalWrite(DIS_AB, HIGH);  // disable for now;
  //pinMode(DIS_AB,   OUTPUT);
  
  // Note: PWM pins cannot be controlled by Fast library. They remain in PWM mode.
  
  //pinModeFast(DIS_AB,   OUTPUT);
  pinModeFast(EN_A,   OUTPUT);
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinModeFast(EN_B,   OUTPUT);
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  
  StopMotors();

  setTimerForPWM();

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
  
  pinModeFast(RED_LED_PIN, OUTPUT);
  pinModeFast(BLUE_LED_PIN, OUTPUT);
  pinModeFast(GREEN_LED_PIN, OUTPUT);

  EncodersInit();    //Initialize the encoders
}

int cnt = 0;

void loop()
{
  cnt++;
  
  /*
  // test debug LEDs
  DbgBlue(true);
  delay(1000);
  DbgBlue(false);
  delay(1000);
  return;
  */  

  /*
  int pwr = 50; 
  
  if((cnt/1000) % 2 == 0)
  {
    analogWrite( RPWM_B, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
    //analogWrite( LPWM_B, 255);
    digitalWrite(LPWM_B, HIGH);
    digitalWriteFast(EN_B, HIGH);
    digitalWrite(13, HIGH);
  }
  else
  {
    analogWrite( LPWM_B, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
    //analogWrite( RPWM_B, 255);
    digitalWrite(RPWM_B, HIGH);
    digitalWriteFast(EN_B, HIGH);
    digitalWrite(13, LOW);
  }
  delay(5);
  
  return;
  */

  /*
  RightMotorGo();
  //LeftMotorGo();
  delay(1000);
  
  StopMotors();
  delay(100000);
  return;
  */
  
  if(Serial3.available())  // takes 8us if false
  {
    int val;
    val=Serial3.read();
    Serial.print(val);
  }

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
    case 'r':
      Rdistance = 0;
      RightMotorTest(testPower);
      delay(100);
      if(Rdistance > 10)
        Serial.print("Right Encoder OK    Rdistance: ");
      else
      {
        Serial.print("Right Encoder ERROR!    Rdistance: ");
      }
      Serial.print(Rdistance);
      Serial.println("    (expected value > 10 ticks) ****************************");
      StopMotors();
      break;

    case 'R':
      Rdistance = 0;
      RightMotorTest(-testPower);
      delay(100);
      if(Rdistance < -10)
        Serial.print("Right Encoder OK    Rdistance: ");
      else
      {
        Serial.print("Right Encoder ERROR!    Rdistance: ");
      }
      Serial.print(Rdistance);
      Serial.println("    (expected value < -10 ticks) ****************************");
      StopMotors();
      break;

    case 'l':
      Ldistance = 0;
      LeftMotorTest(testPower);
      delay(100);
      if(Ldistance > 10)
        Serial.print("Left Encoder OK    Ldistance: ");
      else
      {
        Serial.print("Left Encoder ERROR!    Ldistance: ");
      }
      Serial.print(Ldistance);
      Serial.println("    (expected value > 10 ticks) ****************************");
      StopMotors();
      break;

    case 'L':
      Ldistance = 0;
      LeftMotorTest(-testPower);
      delay(100);
      if(Ldistance < -10)
        Serial.print("Left Encoder OK    Ldistance: ");
      else
      {
        Serial.print("Left Encoder ERROR!    Ldistance: ");
      }
      Serial.print(Ldistance);
      Serial.println("    (expected value < -10 ticks) ****************************");
      StopMotors();
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

void RightMotorGo()  // The Right Motor will rotate clockwise (robot's forward direction).
{
  int pwr = -50; 
  analogWrite( LPWM_B, (byte)(255 + pwr));  // 0 is max speed; 255 - min speed
  digitalWrite(RPWM_B, HIGH);
  digitalWriteFast(EN_B, HIGH);
}

void LeftMotorGo()  // The Left Motor will rotate counter clockwise (robot's forward direction).
{
  int pwr = 50; 
  analogWrite( RPWM_A, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
  digitalWrite(LPWM_A, HIGH);
  digitalWriteFast(EN_A, HIGH);
}

void RightMotorTest(int pwm_R)  // The Right Motor will rotate clockwise (robot's forward direction).
{
  int pwr; 
  if(pwm_R > 0)
  {
    pwr = toMotorPower(pwm_R);
    // Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant
    // Arduino generates 488Hz PWM propportional to analogWrite value:
    analogWrite( LPWM_B, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
    digitalWrite(RPWM_B, HIGH);
  }
  else if(pwm_R < 0)
  {
    pwr = toMotorPower(-pwm_R);
    // Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant
    analogWrite( RPWM_B, (byte)(255 - pwr));
    digitalWrite(LPWM_B, HIGH);
  }
  else
  {
    digitalWrite(RPWM_B, HIGH);
    digitalWrite(LPWM_B, HIGH);
  }
  digitalWriteFast(EN_B, HIGH);
}

void LeftMotorTest(int pwm_L)  // The Left Motor will rotate counterclockwise (robot's forward direction).
{
  int pwr; 
  if(pwm_L > 0)
  {
    pwr = toMotorPower(pwm_L);
    // Rotate forward: EN = 1, RPWM = PWM, LPWM = 1, DIS = vacant
    // Arduino generates 488Hz PWM propportional to analogWrite value:
    analogWrite( RPWM_A, (byte)(255 - pwr));  // 0 is max speed; 255 - min speed
    digitalWrite(LPWM_A, HIGH);
  }
  else if(pwm_L < 0)
  {
    pwr = toMotorPower(-pwm_L);
    // Rotate reverse: EN = 1, RPWM = 1, LPWM = PWM, DIS = vacant
    analogWrite( LPWM_A, (byte)(255 - pwr));
    digitalWrite(RPWM_A, HIGH);
  }
  else
  {
    digitalWrite(RPWM_A, HIGH);
    digitalWrite(LPWM_A, HIGH);
  }
  digitalWriteFast(EN_A, HIGH);
}

int toMotorPower(int pwr)
{
  //pwr = 255 - pwr;
  pwr = constrain(pwr, 0, 230);  // PWM duty cycle cannot be less than 98% per DBH-1B H-bridge specs
  return pwr;
}

void StopMotors()  // Both Motors will stop and brake.
{
  // Parking but not brake: EN = 0, RPWM = 1, LPWM = 1, DIS = vacant
  // Parking and brake: EN = 1, RPWM = 1, LPWM = 1, DIS = vacant
  digitalWrite(RPWM_A, HIGH);
  digitalWrite(LPWM_A, HIGH);
  digitalWriteFast(EN_A, HIGH);
  digitalWrite(RPWM_B, HIGH);
  digitalWrite(LPWM_B, HIGH);
  digitalWriteFast(EN_B, HIGH);
}




//=====================ENCODER TEST======================//

void EncodersInit()
{
  pinMode(2, INPUT);  // left, INT 0
  pinMode(8, INPUT);  // left

  pinMode(3, INPUT);  // right, INT 1
  pinMode(9, INPUT);  // right

  // Most Arduino boards have two external interrupts: numbers 0 (on digital pin D2) and 1 (on digital pin D3)
  attachInterrupt(0, leftEncoder, CHANGE);   // int 0, pin D2  - left encoder interrupt
  attachInterrupt(1, rightEncoder, CHANGE);  // int 1, pin D3  - right encoder interrupt
}

// ************************************
//    Read distance from the encoders
// ************************************
void leftEncoder()
{
  // we spend around 10us in the interrupt, at approx 1kHz frequency at pwm=80
  //digitalWrite(10, HIGH);

  // for Mega and other boards we use digitalRead(), while 328 (UNO) chips are read directly:
  boolean vi = digitalRead(2) == 0; //(PIND & _BV(PIND2)) == 0; // read pin D2 (PD2 on UNO)
  boolean vd = digitalRead(8) == 0; //(PINB & _BV(PINB0)) == 0; // read pin D8 (PB0)

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

  // for Mega and other boards we use digitalRead(), while 328 (UNO) chips are read directly:
  boolean vi = digitalRead(3) == 0; //(PIND & _BV(PIND3)) == 0; // read pin D3 (PD3 on UNO)
  boolean vd = digitalRead(9) == 0; //(PINB & _BV(PINB1)) == 0; // read pin D9 (PB1)

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

// debugging LEDs:
void DbgRed(bool on)
{
  digitalWriteFast(RED_LED_PIN, on);
}

void DbgBlue(bool on)
{
  digitalWriteFast(BLUE_LED_PIN, on);
}

void DbgGreen(bool on)
{
  digitalWriteFast(GREEN_LED_PIN, on);
}


void setTimerForPWM()
{
// http://www.letsmakerobots.com/content/changing-pwm-frequencies-arduino-controllers
// Note: Divide the PWM frequency by 2 for an 8MHz clock, multiply by 1.25 for a 20MHz clock.

//For Arduino Uno, Nano, Micro Magician, Mini Driver, Lilly Pad and any other board using ATmega 8, 168 or 328

//---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

//---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
  
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz



//For Arduino Mega1280, Mega2560, MegaADK, Spider or any other board using ATmega1280 or ATmega2560

//---------------------------------------------- Set PWM frequency for D4 & D13 ------------------------------
  
//TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
//  TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz
//TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
//TCCR0B = TCCR0B & B11111000 | B00000101;    // set timer 0 divisor to  1024 for PWM frequency of    61.04 Hz


//---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
  
//TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000010;    // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR1B = TCCR1B & B11111000 | B00000011;    // set timer 1 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR1B = TCCR1B & B11111000 | B00000100;    // set timer 1 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz

//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  
//TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000010;    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
//TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
//  TCCR2B = TCCR2B & B11111000 | B00000100;    // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR2B = TCCR2B & B11111000 | B00000101;    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
//TCCR2B = TCCR2B & B11111000 | B00000110;    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR2B = TCCR2B & B11111000 | B00000111;    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz


//---------------------------------------------- Set PWM frequency for D2, D3 & D5 ---------------------------
  
//TCCR3B = TCCR3B & B11111000 | B00000001;    // set timer 3 divisor to     1 for PWM frequency of 31372.55 Hz
//TCCR3B = TCCR3B & B11111000 | B00000010;    // set timer 3 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR3B = TCCR3B & B11111000 | B00000100;    // set timer 3 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR3B = TCCR3B & B11111000 | B00000101;    // set timer 3 divisor to  1024 for PWM frequency of    30.64 Hz

  
//---------------------------------------------- Set PWM frequency for D6, D7 & D8 ---------------------------
  
//TCCR4B = TCCR4B & B11111000 | B00000001;    // set timer 4 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR4B = TCCR4B & B11111000 | B00000010;    // set timer 4 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR4B = TCCR4B & B11111000 | B00000011;    // set timer 4 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR4B = TCCR4B & B11111000 | B00000100;    // set timer 4 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR4B = TCCR4B & B11111000 | B00000101;    // set timer 4 divisor to  1024 for PWM frequency of    30.64 Hz


//---------------------------------------------- Set PWM frequency for D44, D45 & D46 ------------------------
  
//TCCR5B = TCCR5B & B11111000 | B00000001;    // set timer 5 divisor to     1 for PWM frequency of 31372.55 Hz
TCCR5B = TCCR5B & B11111000 | B00000010;    // set timer 5 divisor to     8 for PWM frequency of  3921.16 Hz
//  TCCR5B = TCCR5B & B11111000 | B00000011;    // set timer 5 divisor to    64 for PWM frequency of   490.20 Hz
//TCCR5B = TCCR5B & B11111000 | B00000100;    // set timer 5 divisor to   256 for PWM frequency of   122.55 Hz
//TCCR5B = TCCR5B & B11111000 | B00000101;    // set timer 5 divisor to  1024 for PWM frequency of    30.64 Hz

}

