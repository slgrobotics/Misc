#include <Wire.h>
//#include <stdint.h> // Needed for uint8_t

//-------------------------------------- Variable definitions ---------------------------------------------//

double GyroX, GyroY, GyroZ, GyroTemp;  // raw variables delivered by the IMU

// variables calculated by the Kalman filter:
double angle;      // Optimal (estimated) angle from the vertical, degrees, positive when tilting backwards
double angle_dot;  // Optimal (estimated) angular velocity, positive when turning backwards

double k1, k2, k3, k4;   // Adjustable PID Parameters

// variables involved in calculating motor inputs:
double acceleration;  // Acceleration in the conversion
double theta;
double y_value;       // y value of acceleration sensor

double turn_flag=0.0;

volatile int Ldistance, Rdistance; // encoders - distance traveled

int pwm;
int pwm_R, pwm_L;
double range;
double direction_error_all;
double distance_per_cycle;
double wheel_speed;
double angle_setpoint_compensated = 0.0;  // degrees, positive when tilting backwards. Compensated for GC shifts under load.
double angle_setpoint_remote = 0.0;
double distance_setpoint_remote = 0.0;

#define TiltChannel          0   // for Ema
#define SpeedChannel         1

double tiltEma = 0.0;
double speedEma = 0.0;

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

// motor pins - PWM and Direction:
const int E1 = 5;    // M1 Speed Control      (RightMotorPWM, 975Hz, T0)
const int E2 = 6;    // M2 Speed Control      (LeftMotorPWM,  975Hz, T0)
const int M1 = 4;    // M1 Direction Control  (RightMotorEN)
const int M2 = 7;    // M2 Direction Control  (LeftMotorEN)

#define buzzer 13 // Connected to a BC547 transistor - there is a protection diode at the buzzer as well

//-------------------------------------- End of variable definitions -----------------------------------------//

void setup()
{
  Wire.begin();           // join i2c bus (address optional for master)
  Serial.begin(115200);   // start serial 

  initImu();

  GyroCalibrate();
  delay(100);

  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(E2, OUTPUT);

  analogWrite(E1, 0);    // make sure motors are stopped
  analogWrite(E2, 0);

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  //The balance PID Parameters which can read from The ADPin0-3
  //  k1 = 24.80;
  //  k2 = 9.66;
  //  k3 = 4.14;
  //  k4 = 0.99;

  // best experimental values:
  //  k1 = 16.57;
  //  k2 = 2.45;
  //  k3 = 1.82;
  //  k4 = 0.79;

  k1 = 0;
  k2 = 0;
  k3 = 0;
  k4 = 0;

  setEmaPeriod(TiltChannel, 100);
  setEmaPeriod(SpeedChannel, 100);

  EncodersInit();	// Initialize the encoders
  
  pinMode(buzzer, OUTPUT);
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

  //digitalWrite(11, HIGH);

  if(slowLoopCnt++ >= slowLoopFactor)
  {
    slowLoopCnt = 0;

    // multipliers to have the experimental values when POTs are in neutral position:
    k1 = analogRead(AD0pin) * 0.03;
    k2 = analogRead(AD1pin) * 0.005;
    k3 = analogRead(AD2pin) * 0.0009; // * 0.0003;
    k4 = analogRead(AD3pin) * 0.0015; //* 0.0002;

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
      //pwm_R = 80;
      //pwm_L = 80;

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
  //digitalWrite(11, LOW);

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

//  Serial.print("K1=");
//  Serial.print(k1);
//  Serial.print("    K2=");
//  Serial.print(k2);
//  Serial.print("    K3=");
//  Serial.print(k3);
//  Serial.print("    K4=");
//  Serial.println(k4);
//
//  Serial.print("Gyro:   X=");
//  Serial.print(GyroX);
//  Serial.print("   Y=");
//  Serial.print(GyroY);
//  Serial.print("   Z=");
//  Serial.print(GyroZ);
//  Serial.print("   Temp=");
//  Serial.println(GyroTemp);
//
//  Serial.print("Angle calculation:   acceleration=");
//  Serial.print(acceleration);
//  Serial.print("   theta=");
//  Serial.print(theta);
//  Serial.print("       After Kalman:   angle=");
//  Serial.print(angle);
//  Serial.print("   angle_dot=");
//  Serial.println(angle_dot);
//
//  Serial.print("pwm=");
//  Serial.print(pwm);
//  Serial.print("   direction_error_all=");
//  Serial.println(direction_error_all);
//
//  Serial.print("Motors:   Right pwm_R: ");
//  Serial.print(pwm_R);
//  Serial.print("       Left pwm_L: ");
//  Serial.println(pwm_L);
//
//  Serial.print("Encoders:  Right Rdistance: ");
//  Serial.print(Rdur);
//  Serial.print("       Left Ldistance: ");
//  Serial.println(Ldur);
//
  Serial.print("Control:  angle: ");
  Serial.print(angle_setpoint_remote);
  Serial.print("     turn: ");
  Serial.println(turn_flag);

  Serial.print("Ema:  tilt: ");
  Serial.print(tiltEma);
  Serial.print("       speed: ");
  Serial.print(speedEma);
  Serial.print("       angle: ");
  Serial.print(angle);
  Serial.print("       angle_setpoint_compensated: ");
  Serial.println(angle_setpoint_compensated);
}

void shortBuzz()
{
  digitalWrite(buzzer,HIGH);
  delay(100);  
  digitalWrite(buzzer,LOW);
}
