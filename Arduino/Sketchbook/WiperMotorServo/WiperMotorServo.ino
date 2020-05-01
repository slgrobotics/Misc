
//#include <digitalWriteFast.h>  not working right here!

#include <Metro.h> //Include Metro library

#include <PID_v1.h>

// 10 KOhm potentiometer:
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

// for PID see https://github.com/br3ttb/Arduino-PID-Library  and  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

// Define Variables used by PID:
double Setpoint, Measured;  // all in tenths of degrees, so 90 degrees becomes 900
double MotorPower;          // in PWM, -255...255

// Specify the links and initial tuning parameters:
//PID myPID(&Measured, &MotorPower, &Setpoint, 1.5, 3.0, 0.01, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE

//PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 1.0, 0.005, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

PID myPID(&Measured, &MotorPower, &Setpoint, 0.3, 0.1, 0.001, REVERSE);       // double Kp, Ki, Kd, DIRECT or REVERSE

#define SAMPLE_TIME 10

Metro mainMetro = Metro(SAMPLE_TIME);
Metro traceMetro = Metro(100);

void setup()
{
  setupRotationSensor();

  setupMotor();

  enableMotor();

  setupRcReceiverRead();

  // to make things simpler, we operate actuator using Servo-like microsecond scale, 800 to 2200

  Setpoint = 900;    // tenths of degrees

  setEmaPeriod(10);

  //turn the PID on and set parameters:
  myPID.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID
  myPID.SetOutputLimits(-255.0, 255.0);  // match to maximum PID outputs in both directions
  myPID.SetSampleTime(SAMPLE_TIME);      // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval.

  delay(100);
  Serial.begin(115200);
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
  //
  // the way it works that when you boot up with RC receiver signal off - the POT will be in control.
  // once RC signal is detected, the RC will be in control, even after its signal disappears (last position will be held).
  //
  
  if (RC_avail())
  {
    // just a bit of EMA smoothing to eliminate RC signal rounding errors and imprecision:
    rcpwmEma = (valuePrev < -1000000.0) ? rcpwm : ((rcpwm - valuePrev) * multiplier + valuePrev);  // ema
    valuePrev = rcpwmEma;
    RC_availFlag = true;
  }

  if (mainMetro.check() == 1)
  {
    feedbackPosValue = readRotationSensor() - 900; // avoiding transition from 0 to 360

    if (RC_availFlag)
    {
      //RC_availFlag = false;
      if (rcpwmEma >= 900 && rcpwmEma <= 2000)
      {
        Setpoint = map(rcpwmEma, 900, 2000, 0, 1800);  // RC value to tenths of degrees
      }
    }
    else
    {
      // read the feedback pot analog value:
      potValue = analogRead(analogInPin);   // value read from the pot

      // pot value is in the range 0 to 1023
      // the lower half of it we use for reverse rotation; the upper half for forward rotation:
      //pwr = map(potValue, 0, 1023, -255, 255);

      Setpoint = map(potValue, 0, 1023, 0, 1800);  // ADC value to tenths of degrees
    }

    //mappedFeedback = map(feedbackPosValue, 0, 180, 0, 180);    // map(value, fromLow, fromHigh, toLow, toHigh)

    //Measured = mappedFeedback;
    Measured = feedbackPosValue;  // already tenths of degrees

    myPID.Compute();

    pwr = (int)MotorPower;

    pwm = setMotorPower(pwr);
  }

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
