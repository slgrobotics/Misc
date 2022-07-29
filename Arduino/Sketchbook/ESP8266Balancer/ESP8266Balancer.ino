
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
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS)
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

//-------------------------------------- Variable definitions ---------------------------------------------//

float GyroX, GyroY, GyroZ;     // delivered by the IMU
float AccelX, AccelY, AccelZ;  // delivered by the IMU

// variables calculated by the Kalman filter:
float angle;      // Optimal (estimated) angle from the vertical, degrees, positive when tilting backwards
float angle_dot;  // Optimal (estimated) angular velocity, positive when turning backwards

float k1, k2, k3, k4;   // Adjustable PID Parameters

// variables involved in calculating motor inputs:
float acceleration;  // Acceleration in the conversion
float theta;
float y_value;       // y value of acceleration sensor

float turn_flag=0.0;

volatile int Ldistance, Rdistance; // encoders - distance traveled

int pwm;
int pwm_R, pwm_L;
float range;
float direction_error_all;
float distance_per_cycle;
float wheel_speed;
float angle_setpoint_remote = 0.0;
float distance_setpoint_remote = 0.0;

#define TiltChannel          0   // for Ema
#define SpeedChannel         1

float tiltEma = 0.0;
float speedEma = 0.0;

// motor pins - PWM and Direction:
// we command PWM directly, and dir/stop/brake via shift register
const int rightPwmPin = D8;    // Speed Control      (RightMotorPWM, 1kHz)
const int leftPwmPin = D7;     // Speed Control      (LeftMotorPWM,  1kHz)

const int testBit = 0;        // bit 0 connected to yellow LED for testing
const int rightDirBit = 1;    // direction Control  (Right Motor)
const int leftDirBit = 2;     // direction Control  (Left Motor)
const int stopBit = 3;        // feathers both wheels
const int brakeBit = 4;       // applies moderate braking force to both motors
const int rightLedBit = 5;    // built-in LED in the hoverboard
const int leftLedBit = 6;     // built-in LED in the hoverboard

// Encoder interrupt pins:
const int rightEncoderPin = D6;
const int leftEncoderPin = D5;

// these are obsolete on ESP8266, no free pins ;-(
const int debugPin1 = 10;
const int debugPin2 = 11;

//-------------------------------------- End of variable definitions -----------------------------------------//

void setup()
{
  Serial.begin(115200);   // start serial 
  //delay(100);

  // DEBUG pins:
  //pinMode(debugPin1, OUTPUT);
  //pinMode(debugPin2, OUTPUT);

  // Set the number of bits you have (multiples of 8)
  shift.setBitCount(8);

  // Set the data, clock, and latch pins you are using
  // This also sets the pinMode for these pins
  shift.setPins(dataPin, clockPin, latchPin); 

  initMotors(); // uses stift register

  shift.batchWriteBegin();
  shift.writeBit(testBit, LOW);
  shift.writeBit(rightLedBit, LOW);
  shift.writeBit(leftLedBit, LOW);
  shift.batchWriteEnd();

  initImu();

  GyroCalibrate();
  delay(100);

  initPids();
  
  setEmaPeriod(TiltChannel, 100);
  setEmaPeriod(SpeedChannel, 100);

  initEncoders();	// Initialize the encoders
  
  //pinMode(buzzer, OUTPUT);
  shortBuzz();
}

// =========================== main loop timing ============================================
#define STD_LOOP_TIME 5000 // Fixed time loop of 5 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;  // see Kalman dt
unsigned long loopStartTime;
int loopCnt = 0;

const int slowLoopFactor = 200;
int slowLoopCnt = slowLoopFactor;    // should fire on first loop to read k* values
const int pidLoopFactor = 4;
boolean doTRACE = true;
// =========================================================================================

void loop()
{
  loopStartTime = micros(); 

  //digitalWrite(debugPin2, HIGH);

  if(slowLoopCnt++ >= slowLoopFactor)
  {
    slowLoopCnt = 0;

    if(doTRACE)
    {
      printAll();
    }
  }

  control();

  // we come here in 5us if no serial is available

  readFromImu();

  if(Angle_calculate())  // must be on the fast loop to keep track of accel/gyro all the time
  {
    // we come here at 3ms

    if(loopCnt % pidLoopFactor == 0)  // we do PID->pwm calculation on a slower scale
    {  
      pwm_calculate(); // takes 0.14ms

      // test: At both motors set to +80 expect Ldistance and Rdistance to show 6...7 at 2ms delay below
      pwm_R = -10;
      pwm_L = 10;

      set_motor();  // takes 0.12ms
    }
  }
  else
  {
    // angle outside limits - stop
    pwm_R = 0;
    pwm_L = 0;

    set_motor();  // takes 0.12ms
  }

  // we are done in about 3.4ms
  //digitalWrite(debugPin2, LOW);

  /* Wait here - use a time fixed loop */
  lastLoopUsefulTime = micros() - loopStartTime;
  if (lastLoopUsefulTime < STD_LOOP_TIME)
  {
    while((micros() - loopStartTime) < STD_LOOP_TIME)
    {
      //Usb.Task();
      ;
    }
  }
  loopCnt++;
}

void printAll()
{
  int Ldur = Ldistance;
  int Rdur = Rdistance;

  Serial.println("--------------------");

  Serial.print("Gyro:   X=");
  Serial.print(GyroX);
  Serial.print("   Y=");
  Serial.print(GyroY);
  Serial.print("   Z=");
  Serial.println(GyroZ);

  Serial.print("Accel:   X=");
  Serial.print(AccelX);
  Serial.print("   Y=");
  Serial.print(AccelY);
  Serial.print("   Z=");
  Serial.println(AccelZ);

  Serial.print("Angle calculation:   acceleration=");
  Serial.print(acceleration);
  Serial.print("   theta=");
  Serial.print(theta);
  Serial.print("       After Kalman:   angle=");
  Serial.print(angle);
  Serial.print("   angle_dot=");
  Serial.println(angle_dot);

  Serial.print("pwm=");
  Serial.print(pwm);
  Serial.print("   direction_error_all=");
  Serial.println(direction_error_all);

  Serial.print("Motors:   Right pwm_R: ");
  Serial.print(pwm_R);
  Serial.print("       Left pwm_L: ");
  Serial.println(pwm_L);

  Serial.print("Encoders:  Right Rdistance: ");
  Serial.print(Rdur);
  Serial.print("       Left Ldistance: ");
  Serial.println(Ldur);

  Serial.print("Ema:  tilt: ");
  Serial.print(tiltEma);
  Serial.print("       speed: ");
  Serial.println(speedEma);
  Serial.print("       angle: ");
  Serial.println(angle);
}

void shortBuzz()
{
  //digitalWrite(buzzer,HIGH);
  //delay(100);  
  //digitalWrite(buzzer,LOW);
}
