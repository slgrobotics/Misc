

// ======== this is work in progress - use LawnMowerThrottleGood instead!!! =========

//#include <digitalWriteFast.h>  not working right here!

//#define DEBUG_STEP_RESPONSE
//#define TRACE

#include <Metro.h> //Include Metro library

#include <PID_v1.h>

// actual low and high throttle arm positions:
#define ROT_SENSOR_LOW_LIMIT    -20.0
#define ROT_SENSOR_HIGH_LIMIT   2100.0
#define ROT_SENSOR_NEUTRAL      1351.0

/*
#define ROT_SENSOR_LOW_LIMIT    570
#define ROT_SENSOR_HIGH_LIMIT   1650
#define ROT_SENSOR_NEUTRAL      ROT_SENSOR_LOW_LIMIT
*/

// to compensate for timing differences between R/C source and Arduino:
#define RC_TRIM                 0
//#define RC_DO_REVERSE


// for different motors deadzone can be adjusted here:
//#define DEADZONE 100
#define DEADZONE 30
// if defined - When to switch off motor, tenths of degrees (so 2 degrees becomes 20):
#define PRETTY_CLOSE_THRESHOLD 10.0 

// cutoff motor power after that time, if input doesn't change:
#define SERVO_ACTIVE_TIME_MS  3000
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

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.01, 0.001, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.0, 0.0, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

PID myPID(&Measured, &MotorPower, &Setpoint, 0.5, 0.0, 0.005, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE


#define SAMPLE_TIME 10

// Arduino task scheduler:
Metro mainMetro = Metro(SAMPLE_TIME);
#ifdef TRACE
Metro traceMetro = Metro(100);
#endif // TRACE

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
boolean prettyClose = false;

// variables to compute exponential moving average for signal coming from RC receiver:
int emaPeriod;
double valuePrev;
double multiplier;

void setup()
{
  pinMode(analogInPin, INPUT);
  pinMode(switchInPin, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  setupRotationSensor();

  setupMotor();
  featherMotor();

  setupRcReceiverRead();

  Setpoint = ROT_SENSOR_NEUTRAL;    // on bootup, set it to neutral. Tenths of degrees

  rcpwm = 1000;
  rcpwmEma = valuePrev = 1000.0;

  setEmaPeriod(10);

  //turn the PID on and set parameters:
  myPID.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID
  myPID.SetOutputLimits(-255.0, 255.0);  // match to maximum PID outputs in both directions
  myPID.SetSampleTime(SAMPLE_TIME);      // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval.

  SetpointChangedMs = millis();

  Serial.begin(115200);
}

enum States {
              STATE_NONE,
              STATE_IDLE,
              STATE_RC_CHANGED,
              STATE_POT_CHANGED,
              STATE_RC_FAILOVER
};

const char* StateStrings[] = { "NONE", "IDLE", "RC_CHANGED", "POT_CHANGED", "RC_FAILOVER" };

States state = STATE_IDLE;
States state_prev = STATE_NONE;

#define mustIdle (prettyClose || now - SetpointChangedMs > SERVO_ACTIVE_TIME_MS)

void loop()
{
  unsigned long now = millis();

#ifdef TRACE
  // Print state change, if any:
  if(state != state_prev)
  {
    Serial.print("STATE: ");
    Serial.print(StateStrings[(int)state_prev]);
    Serial.print(" --> ");
    Serial.println(StateStrings[(int)state]);
    state_prev = state;
  }
#endif // TRACE
  
  checkRc();  // call often, picks up R/C signal from the interrupt
  
  if (mainMetro.check() == 1)
  {
    doRcControl = digitalRead(switchInPin); // TRUE if switch is in "RC" position

    switch(state)
    {
      case STATE_IDLE:
        processInput();
        //computePid();
        break;
        
      case STATE_POT_CHANGED:
        processInput();
        computePid();
        pwm = setMotorPower(pwr);
        if(mustIdle)
        {
          toIdleState();
        }
        break;

      case STATE_RC_CHANGED:
        processInput();
        computePid();
        pwm = setMotorPower(pwr);
        if(mustIdle)
        {
          toIdleState();
        }
        break;

      case STATE_RC_FAILOVER:
        computePid();
        pwm = setMotorPower(pwr);
        if(mustIdle)
        {
          toIdleState();
        }
        break;

    } // end switch
    
  } // end mainMetro.check()

#ifdef TRACE
    if (traceMetro.check() == 1)
    {
      Serial.print(StateStrings[(int)state]);
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

void processInput()
{
  if(doRcControl)
  {
    processRc();
  }
  else
  {
    processPot();
  }
}

void processPot()
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
    // pot value is in the range 0 to 1023
    // set the Setpoint value for PID:
    setSetpoint(map(potValue, 1023, 0, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_HIGH_LIMIT));  // pot value to tenths of degrees
    //Serial.println(Setpoint);
    state = STATE_POT_CHANGED;
  }
}

void processRc()
{
  if(millis() - rc_lastAvailable > 1000)
  {
    if(Setpoint != ROT_SENSOR_NEUTRAL)
    {
      setSetpoint(ROT_SENSOR_NEUTRAL);    // R/C Failsafe, set it to low throttle (idle, neutral). Tenths of degrees
      state = STATE_RC_FAILOVER;
    }
  }
  else if(RC_availFlag)
  {
    RC_availFlag = false;
    // signal guaranteed to be within the 1000..2000 range.
    // set the Setpoint value for PID:
    setSetpoint(map(rcpwmEma, 1000, 2000, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_HIGH_LIMIT));  // RC value to tenths of degrees
    //Serial.println(Setpoint);
    state = STATE_RC_CHANGED;
  }
}

void checkRc()
{
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
      RC_availFlag = true; 
      rcpwmEma = tmp; //constrain(tmp, 1000, 2000);
    }
  }
}

void toIdleState()
{
  myPID.SetMode(MANUAL); 
  digitalWrite(LED_PIN, LOW);
  featherMotor();
  pwm = 0;
  state = STATE_IDLE;
}

void toActive()
{
  myPID.SetMode(AUTOMATIC); 
  digitalWrite(LED_PIN, HIGH);
  enableMotor();
}

void setSetpoint(double sp)
{
  Setpoint = sp;
  SetpointChangedMs = millis();
  toActive();
}

void computePid()
{
  feedbackPosValue = readRotationSensor() - 900; // no need here // - 900; // avoiding transition from 0 to 360
  //Serial.print("Feedback: ");
  //Serial.println(feedbackPosValue);

  // set the Measured value for PID:
  Measured = feedbackPosValue;  // already tenths of degrees

  myPID.Compute();

  pwr = (int)MotorPower;  // that's what PID computed

#ifdef PRETTY_CLOSE_THRESHOLD
  prettyClose = abs(Setpoint - Measured) < PRETTY_CLOSE_THRESHOLD;   // tenths of degrees, so 2 degrees becomes 20
#endif // PRETTY_CLOSE_THRESHOLD
}

void setEmaPeriod(int period)
{
  valuePrev = -2000000.0;  // signal to use value directly for the first point
  emaPeriod = period;
  multiplier = 2.0 / (1.0 + (double)emaPeriod);
}
