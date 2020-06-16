
//#include <digitalWriteFast.h>  not working right here!

#include <Metro.h> //Include Metro library

#include <PID_v1.h>

// actual low and high throttle arm positions:
#define ROT_SENSOR_LOW_LIMIT    450
#define ROT_SENSOR_HIGH_LIMIT   1650
const boolean doReverseRc = false;

// for different motors deadzone can be adjusted here:
#define DEADZONE 100

// 10 KOhm potentiometer:
const int analogInPin = A1;  // Analog input pin that the potentiometer is attached to
const int switchInPin = A2;  // switching between the RC input and potentiometer

// for PID see https://github.com/br3ttb/Arduino-PID-Library  and  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

// Define Variables used by PID:
double Setpoint, Measured;  // all in tenths of degrees, so 90 degrees becomes 900
double MotorPower;          // in PWM, -255...255

// Specify the links and initial tuning parameters:

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.1, 0.001, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.01, 0.001, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE

PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.0, 0.005, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE


#define SAMPLE_TIME 10

Metro mainMetro = Metro(SAMPLE_TIME);
Metro traceMetro = Metro(100);

boolean SetpointChangedFlag = false;
unsigned long SetpointChangedMs;

#define SERVO_ACTIVE_TIME_MS  3000
#define CHANGE_TOLERANCE      10

const int LED_PIN = 13;

void setup()
{
  pinMode(analogInPin, INPUT);
  pinMode(switchInPin, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  setupRotationSensor();

  setupMotor();
  featherMotor();

  setupRcReceiverRead();

  // to make things simpler, we operate actuator using Servo-like microsecond scale, 800 to 2200

  Setpoint = ROT_SENSOR_LOW_LIMIT;    // tenths of degrees

  setEmaPeriod(10);

  //turn the PID on and set parameters:
  myPID.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID
  myPID.SetOutputLimits(-255.0, 255.0);  // match to maximum PID outputs in both directions
  myPID.SetSampleTime(SAMPLE_TIME);      // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval.

  delay(100);
  Serial.begin(115200);

  SetpointChangedFlag = false;
  SetpointChangedMs = millis();
}

int feedbackPosValue;
volatile int rcpwm;                  // RC receiver pulsewidth measurements from the interrupt
int pwr;
int pwm;
int potValue;
double rcpwmEma;
boolean RC_availFlag = false;

// variables to compute exponential moving average for signal coming from RC receiver:
int emaPeriod;
double valuePrev;
double multiplier;

void loop()
{
  unsigned long now = millis();
  //SetpointChangedFlag = false;
  
  if (RC_avail())
  {
    // we process RC signal independently of the switch position.
    rcpwm = constrain(rcpwm, 1000, 2000);     // standard RC receiver range, microseconds

    if(doReverseRc)
    {
      rcpwm = map(rcpwm, 1000, 2000, 2000, 1000);
    }
    
    // just a bit of EMA smoothing to eliminate RC signal rounding errors and imprecision:
    double tmp = (valuePrev < -1000000.0) ? rcpwm : ((rcpwm - valuePrev) * multiplier + valuePrev);  // ema
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
    boolean doRcControl = digitalRead(switchInPin); // TRUE if switch is in "RC" position

    if (RC_availFlag)
    {
      RC_availFlag = false;
      if (doRcControl && rcpwmEma >= 800 && rcpwmEma <= 2200)
      {
        // switch is in "RC" position, and signal is within the range.
        // set the Setpoint value for PID:
        Setpoint = map(rcpwmEma, 1000, 2000, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_HIGH_LIMIT);  // RC value to tenths of degrees
      }
    }

    if(!doRcControl)
    {
      // switch is in "Pot" position.
      // read the control potentiometer analog value:
      int tmp = analogRead(analogInPin);
      if(abs(tmp - potValue) > CHANGE_TOLERANCE)
      {
        potValue = tmp;
        SetpointChangedFlag = true;
        SetpointChangedMs = now;
      }

      // pot value is in the range 0 to 1023
      // set the Setpoint value for PID:
      Setpoint = map(potValue, 1023, 0, ROT_SENSOR_LOW_LIMIT, ROT_SENSOR_HIGH_LIMIT);  // pot value to tenths of degrees
    }

    feedbackPosValue = readRotationSensor(); // no need here // - 900; // avoiding transition from 0 to 360
    // set the Measured value for PID:
    Measured = feedbackPosValue;  // already tenths of degrees

    myPID.Compute();

    pwr = (int)MotorPower;  // that's what PID computed

    if(SetpointChangedFlag && now - SetpointChangedMs < SERVO_ACTIVE_TIME_MS)
    {
      digitalWrite(LED_PIN, HIGH);
      enableMotor();
      pwm = setMotorPower(pwr);
	}
    else
    {
      digitalWrite(LED_PIN, LOW);
      SetpointChangedFlag = false;
      featherMotor();
    }
  }

  //Serial.println(rcpwmEma);
  //Serial.println(Measured - Setpoint);

  /*
    if (traceMetro.check() == 1)
    {
      Serial.print(potValue);
      Serial.print("   ");
      Serial.print(MotorPower);
      Serial.print("   ");
      Serial.print(pwm);
      Serial.print("   ");
      Serial.print(feedbackPosValue);
      Serial.print("   ");
      if (RC_avail())
      {
        Serial.print(rcpwm);
      }
      Serial.println();
    }
  */
}

void setEmaPeriod(int period)
{
  valuePrev = -2000000.0;  // signal to use value directly for the first point
  emaPeriod = period;
  multiplier = 2.0 / (1.0 + (double)emaPeriod);
}
