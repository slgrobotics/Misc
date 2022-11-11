
//#include <digitalWriteFast.h>  not working right here!

//#define DEBUG_STEP_RESPONSE
//#define TRACE

//#define RIGHT_SERVO
#define LEFT_SERVO
//#define THROTTLE_SERVO

#include <Metro.h> //Include Metro library

#include <PID_v1.h>

#ifdef RIGHT_SERVO
// actual low, high and neutral arm positions for RIGHT servo, tenths of degrees:
#define ROT_SENSOR_LOW_LIMIT    300.0
#define ROT_SENSOR_HIGH_LIMIT   1900.0
#define ROT_SENSOR_NEUTRAL      1300.0
// to compensate for timing differences between R/C source and Arduino:
#define RC_TRIM                 0
#define RC_DO_REVERSE
// if defined - When to switch off motor, tenths of degrees (so 2 degrees becomes 20):
#define PRETTY_CLOSE_THRESHOLD 10.0 
// for different motors deadzone can be adjusted here:
#define DEADZONE 30
#endif // RIGHT_SERVO

#ifdef LEFT_SERVO
// actual low, high and neutral arm positions for LEFT servo, tenths of degrees:
#define ROT_SENSOR_LOW_LIMIT    210.0
#define ROT_SENSOR_HIGH_LIMIT   1850.0
#define ROT_SENSOR_NEUTRAL      930.0
// to compensate for timing differences between R/C source and Arduino:
#define RC_TRIM                 0
//#define RC_DO_REVERSE
// if defined - When to switch off motor, tenths of degrees (so 2 degrees becomes 20):
#define PRETTY_CLOSE_THRESHOLD 10.0 
// for different motors deadzone can be adjusted here:
#define DEADZONE 30
#endif // LEFT_SERVO

#ifdef THROTTLE_SERVO
// actual low and high throttle arm positions, tenths of degrees:
#define ROT_SENSOR_LOW_LIMIT    570.0
#define ROT_SENSOR_HIGH_LIMIT   1650.0
#define ROT_SENSOR_NEUTRAL      ROT_SENSOR_LOW_LIMIT
// to compensate for timing differences between R/C source and Arduino:
#define RC_TRIM                 0
//#define RC_DO_REVERSE
// if defined - When to switch off motor, tenths of degrees (so 2 degrees becomes 20):
//#define PRETTY_CLOSE_THRESHOLD 10.0 
// for different motors deadzone can be adjusted here:
#define DEADZONE 100
#endif // THROTTLE_SERVO

// ==================================================================================

// cutoff motor power after that time, if input doesn't change:
#define SERVO_ACTIVE_TIME_MS  1000
// ignore lesser changes in input:
#define CHANGE_TOLERANCE      10

// 10 KOhm potentiometer and switch:
const int analogInPin = A1;  // Analog input pin that the "manual mode" potentiometer is attached to
const int switchInPin = A2;  // Switching between the RC input and potentiometer
const int LED_PIN = 13;      // LED is lit whem motor power is on
// see Motor, R/C receiver and Rotation Sensor pin assignments in corresponding tabs

// for PID see https://github.com/br3ttb/Arduino-PID-Library  and  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

// Define Variables used by PID. PID operates in tenths of degrees, at SAMPLE_TIME time grid:
double Setpoint, Measured;  // all in tenths of degrees, so 90 degrees becomes 900
double MotorPower;          // PID output, in PWM, -255...255

// Specify the links and initial tuning parameters:
//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.1, 0.001, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.4, 0.005, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.0, 0.005, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.8, 0.4, 0.06, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 2.5, 0.1, 0.05, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

#ifdef THROTTLE_SERVO
PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.0, 0.005, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE
#else
PID myPID(&Measured, &MotorPower, &Setpoint, 1.8, 0.1, 0.05, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE
#endif // THROTTLE_SERVO


#define SAMPLE_TIME 10

// Arduino task scheduler:
Metro mainMetro = Metro(SAMPLE_TIME);
#ifdef TRACE
Metro traceMetro = Metro(100);
#endif // TRACE

boolean SetpointChangedFlag = false;
unsigned long SetpointChangedMs;

boolean doRcControl;
double feedbackPosValue;      // tenths of degrees, so 90 degrees becomes 900
volatile unsigned int rcpwm;  // period, mirco sec, of R/C signal pulse, 800..2200
volatile unsigned long rc_lastAvailable = 0;  // milliseconds
int pwr;
int pwm;
int potValue;
double rcpwmEma;
boolean RC_availFlag = false;

// variables to compute exponential moving average for signal coming from RC receiver:
int emaPeriod;
double valuePrev;
double multiplier;

void setup()
{
  Serial.begin(115200);

  pinMode(analogInPin, INPUT);
  pinMode(switchInPin, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  setupRotationSensor();

  setupMotor();
  featherMotor();

  setupRcReceiverRead();

  Setpoint = ROT_SENSOR_NEUTRAL;    // on bootup, set it to neutral. Tenths of degrees

  rcpwm = 1500;
  rcpwmEma = valuePrev = 1500.0;

  setEmaPeriod(10);

  //turn the PID on and set parameters:
  myPID.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID
  myPID.SetOutputLimits(-255.0, 255.0);  // match to maximum PID outputs in both directions
  myPID.SetSampleTime(SAMPLE_TIME);      // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval.

  SetpointChangedFlag = true;
  SetpointChangedMs = millis();
}

void loop()
{
  unsigned long now = millis();
  //SetpointChangedFlag = false;
  
  if (RC_avail())
  {
    //print_RCpwm();
    // we process RC signal independently of the switch position.
    unsigned int rcpwmTmp = rcpwm; // get the volatile variable here

#ifdef RC_DO_REVERSE
    rcpwmTmp = map(rcpwmTmp, 1000, 2000, 2000, 1000);
#endif // RC_DO_REVERSE
    
    rcpwmTmp = constrain(rcpwmTmp, 1000, 2000);     // standard RC receiver range, microseconds

    // just a bit of EMA smoothing to eliminate RC signal rounding errors and imprecision:
    double tmp = (valuePrev < -1000000.0) ? rcpwmTmp : ((rcpwmTmp - valuePrev) * multiplier + valuePrev);  // ema
    valuePrev = tmp;
    if(abs(tmp - rcpwmEma) > CHANGE_TOLERANCE)
    {
      rcpwmEma = tmp;
      SetpointChangedFlag = true;
      SetpointChangedMs = now;
    }
    RC_availFlag = true;
  }

  if (mainMetro.check() == 1)
  {
    doRcControl = digitalRead(switchInPin); // TRUE if switch is in "RC" position

    if(doRcControl && millis() - rc_lastAvailable > 1000 && Setpoint != ROT_SENSOR_NEUTRAL)
    {
      Setpoint = ROT_SENSOR_NEUTRAL;    // R/C Failsafe, set it to low throttle (idle, neutral). Tenths of degrees
      SetpointChangedFlag = true;
      SetpointChangedMs = now;
    }

    if (RC_availFlag)
    {
      RC_availFlag = false;
      if (doRcControl && rcpwmEma >= 800 && rcpwmEma <= 2200)
      {
        // switch is in "RC" position, and signal is within the range.
        
#ifdef THROTTLE_SERVO
        // set the Setpoint value for PID:
        Setpoint = map(rcpwmEma, 1000.0, 2000.0, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_HIGH_LIMIT);  // RC value to tenths of degrees
#else
        // set the Setpoint value for PID, keeping 1500us pointed to neutral:
        if(rcpwmEma >= 1500.0)
        {
          Setpoint = map(rcpwmEma, 1500.0, 2000.0, ROT_SENSOR_NEUTRAL, ROT_SENSOR_HIGH_LIMIT);  // RC value to tenths of degrees
        }
        else
        {
          Setpoint = map(rcpwmEma, 1000.0, 1500.0, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_NEUTRAL);  // RC value to tenths of degrees
        }
#endif // THROTTLE_SERVO
        //Serial.println(Setpoint);
      }
    }

    if(!doRcControl)
    {
      // switch is in "Pot" position.
      // read the control potentiometer analog value:
      int tmp = analogRead(analogInPin);

#ifdef DEBUG_STEP_RESPONSE
      // for "step response" PID debugging, use POT as a switch:
      //if(tmp > 512) tmp = 1000; else tmp = 20;  // full range, 180 degrees
      if(tmp > 512) tmp = 750; else tmp = 250;  // half range, 90 degrees
#endif // DEBUG_STEP_RESPONSE
      
      if(abs(tmp - potValue) > CHANGE_TOLERANCE)
      {
        potValue = tmp;
        SetpointChangedFlag = true;
        SetpointChangedMs = now;
      }

      // pot value is in the range 0 to 1023
      // set the Setpoint value for PID:
      Setpoint = map(potValue, 0, 1023, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_HIGH_LIMIT);  // pot value to tenths of degrees
      //Serial.println(Setpoint);
    }

#ifdef THROTTLE_SERVO
    feedbackPosValue = readRotationSensor();
#else
    feedbackPosValue = readRotationSensor() - 900.0; // avoiding transition from 0 to 360
#endif // THROTTLE_SERVO

    //Serial.print("Feedback: ");
    //Serial.println(feedbackPosValue);

    // set the Measured value for PID:
    Measured = feedbackPosValue;  // already tenths of degrees

    myPID.Compute();

    pwr = (int)MotorPower;  // that's what PID computed

#ifdef PRETTY_CLOSE_THRESHOLD
    bool prettyClose = abs(Setpoint - Measured) < PRETTY_CLOSE_THRESHOLD;   // tenths of degrees, so 2 degrees becomes 20
#else
    bool prettyClose = false;
#endif // PRETTY_CLOSE_THRESHOLD

    if(SetpointChangedFlag && now - SetpointChangedMs < SERVO_ACTIVE_TIME_MS && !prettyClose)
    {
      myPID.SetMode(AUTOMATIC); 
      digitalWrite(LED_PIN, HIGH);
      enableMotor();
      pwm = setMotorPower(pwr);
    }
    else
    {
      myPID.SetMode(MANUAL); 
      digitalWrite(LED_PIN, LOW);
      SetpointChangedFlag = false;
      featherMotor();
      pwm = 0;
    }
  }

  //Serial.println(rcpwmEma);
  //Serial.println(Measured - Setpoint);

#ifdef TRACE
    if (traceMetro.check() == 1)
    {
      /*
      Serial.print(doRcControl);
      Serial.print("   ");
      Serial.print(potValue);
      Serial.print("   ");
      Serial.print(pwm);
      Serial.print("   ");
      */
      Serial.print("  M: ");
      Serial.print(MotorPower);
      Serial.print("  S: ");
      Serial.print(Setpoint);
      Serial.print("  F: ");
      Serial.print(feedbackPosValue);
      Serial.print("  d: ");
      Serial.print(Setpoint - feedbackPosValue);
      Serial.print("   RC: ");
      if (millis() - rc_lastAvailable < 1000)
      {
        print_RCpwm();
        Serial.print(" / ");
        Serial.print(rcpwmEma);
      }
      Serial.println();
    }
#endif // TRACE
}

void setEmaPeriod(int period)
{
  valuePrev = -2000000.0;  // signal to use value directly for the first point
  emaPeriod = period;
  multiplier = 2.0 / (1.0 + (double)emaPeriod);
}
