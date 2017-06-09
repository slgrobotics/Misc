
#include <Wire.h>
#include <AlfsTechHbridge.h>

int ledPin = 13;    // Arduino UNO Yellow LED

//-------------------------------------- Variable definitions --------------------------------------------- //

double GyroX,GyroY,GyroZ,GyroTemp;  // variables delivered by IMU

// variables calculated by the Kalman filter:
double angle;      // angle from the vertical, degrees, positive when tilting backwards
double angle_dot;  // angular speed 
double angle_dot_tmp;

double k1,k2,k3,k4;   // Adjustable PID Parameters

// variables involved in calculating motor inputs:
double acceleration;  // Acceleration in the conversion
double theta;
double y_value;       // y value of acceleration sensor

int turn_flag=0;

volatile int Ldistance, Rdistance; // encoders - distance traveled, in ticks

int pwm;
int pwm_R, pwm_L;
double range;
double direction_error_all;
double wheel_speed;
double last_wheel;
double error_a = 0;

// Analog pins connected to potentiometers to adjust k1...k4 :

const int AD0pin = 0;  // k1
const int AD1pin = 1;  // k2
const int AD2pin = 2;  // k3
const int AD3pin = 3;  // k4

// ------------------------------------------------------------------------------------------------------ //

AlfsTechHbridge motors;  	// constructor initializes motors and encoders

float G_Dt = 0.02;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer = 0;     // general purpose timer
long timer_old;

unsigned int counter = 0;
byte gyro_sat = 0;


void setup()
{ 
  Serial.begin(115200);
  pinMode (ledPin, OUTPUT);  // Status LED

  // ======================== init motors and encoders: ===================================

  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  EncodersInit();    // attach interrupts

  pwm_R = 0;
  pwm_L = 0;
  motors.init();
  set_motor();

  // The balance PID Parameters which can read from The ADPin0-3
  k1 = 0;
  k2 = 0;
  k3 = 0;
  k4 = 0;

  blinkLED(20, 50);

  // ========================= init Pololu MiniIMU9: ======================================

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(ledPin, LOW);

  initImu();

  digitalWrite(ledPin, HIGH);

  timer = millis();
  delay(20);
  counter = 0;
}

// **************************
// Calculate the Angle
// **************************
boolean Angle_calculate(void)
{
  boolean ret = true;

  readFromImu();

  // can't calculate angles around 90 degrees (robot fell down, or accelerated too much):
  if(acceleration > 0.8)
  {
    // robot fallen back
    acceleration = 0.8;
    ret = false;
  }
  if(acceleration < -0.8)
  {
    // robot fallen forward
    acceleration = -0.8;
    ret = false;
  }

  // positive acceleration and theta - robot fallen back, negative - fallen forward
  double angleByAccel = asin(acceleration / 0.98) * 57.3;

  theta = angleByAccel - error_a;

  // GyroX can be hundreds with gentle rotation back and forth.
  // Positive GyroX is when robot falling back, negative - falling forward:
  angle_dot_tmp = GyroX / 14.375;       // the angular velocity

  Kalman_Filter(theta, angle_dot_tmp);

  return ret;
}

// **************************
// Calculate the pwm
// **************************
void pwm_calculate()
{
  int direction_error;

  range += (Ldistance + Rdistance) * 0.5;
  range *= 0.9;

  direction_error = Ldistance - Rdistance;
  direction_error_all += direction_error;

  wheel_speed = range - last_wheel;
  last_wheel = range;

  pwm = (angle * k1) + (angle_dot * k2) - (range * k3) - (wheel_speed * k4);    // use PID to calculate the pwm

  //  if(pwm > 255)      // Maximum Minimum Limitations
  //    pwm = 255;
  //    
  //  if(pwm < -255)
  //    pwm = -255;

  if(turn_flag == 0)
  {
    pwm_R = -pwm + direction_error_all;
    pwm_L = -pwm - direction_error_all;
  }
  else
  {
    pwm_R = -pwm - turn_flag * 68;  // Direction PID control
    pwm_L = -pwm + turn_flag * 68;
    direction_error_all = 0;           // clear
  }

  Ldistance = 0;    // clear
  Rdistance = 0;
}

boolean doTRACE = false;

unsigned long lastPrintMillis = 0; 

int mydt = 5;    // 5 milliseconds make for 200 Hz operating cycle
double angleOffVertical = 0.0;
boolean useEulers = true;

// PID variables:
unsigned long last_PID = 0;
double Pgain = 500.0;
double Igain = 0.10; //12.0;
double Dgain = 900.0; //270.0;

double vRate = 0.0;
double last_angle = 0.0;
double Pterm = 0.0;
double Iterm = 0.0;
double Istate = 0.0;
double Dterm = 0.0;

void loop() //Main Loop
{
  delay(1);
  bool angleValid = Angle_calculate();
  
  if((millis() - timer) >= mydt)  // Main loop runs at 200Hz
  {
    counter++;
    timer_old = timer;
    timer = millis();

    // ========== Pololu MiniIMU9 code calculates Euler angles:
    if (timer > timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run in seconds. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    //readFromImu();

    /*
    // *** DCM algorithm
     // Data adquisition
     Read_Gyro();      // This reads raw gyro data
     Read_Accel();     // Read I2C accelerometer
     
     if (counter % 20 == 0)  // Read compass data at 10Hz... (20 loop runs)
     {
     Read_Compass();    // Read I2C magnetometer
     Compass_Heading(); // Calculate magnetic heading  
     }
     
     // Calculations...
     Matrix_update(); 
     Normalize();
     Drift_correction();
     
     if(useEulers)
     {
     Euler_angles();
     
     // at this point we have Euler angles, can convert them to degrees:  ToDeg(roll) ToDeg(pitch) ToDeg(yaw)   
     //printdata();
     
     // wheels axis is parallel to roll axis. So we base our balancing act on changes in the roll:
     angleOffVertical = roll;
     }
     else
     {
     //if (counter % 20 == 0)  // print data at 10Hz... (20 loop runs)
     //{
     //  Serial.print("Gyro xyz: "); Serial.print(gyro_x); Serial.print(" : "); Serial.print(gyro_y); Serial.print(" : "); Serial.println(gyro_z);
     //}
     
     // use raw gyro data as input to PID:
     angleOffVertical = gyro_z / 5000.0;
     }
     */

    if(millis() - lastPrintMillis > 1000)
    {
      lastPrintMillis = millis();

      // initial values as in original source:
      // k1 = 24.80;
      // k2 = 9.66;
      // k3 = 4.14;
      // k4 = 0.99;

      // multipliers to have the experimentally optimal values when POTs are in neutral position:
      k1 = analogRead(AD0pin) * 0.05;
      k2 = analogRead(AD1pin) * 0.001;
      k3 = analogRead(AD3pin) * 0.003;
      k4 = analogRead(AD2pin) * 0.001;

      if(doTRACE)
      {
        int Ldur = Ldistance;
        int Rdur = Rdistance;

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

        Serial.print("Angle calculation:   acceleration=");
        Serial.print(acceleration);
        Serial.print("   theta=");
        Serial.print(theta);
        Serial.print("   angle_dot_tmp=");
        Serial.print(angle_dot_tmp);
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
      }
    }

    control();

    if(angleValid)
    {
      pwm_calculate();

      // test: At both motors set to +80 expect Ldistance and Rdistance to show 6...7 at 2ms delay below
      //pwm_R = 0;
      //pwm_L = 0;

      set_motor();
    }
    else
    {
      // angle outside limits - stop
      pwm_R = 0;
      pwm_L = 0;

      set_motor();
    }
  }
}


