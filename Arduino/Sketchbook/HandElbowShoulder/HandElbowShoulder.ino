
//#define SERIAL_ROBOREALM
#define SERIAL_MY

#include <Wire.h>

// see http://www.chrismoos.com/2012/12/05/avr-os-multitasking-on-arduino 
// works on UNO, not on Mega or Leonardo 

#include <PID_v1.h>

// for PID see https://github.com/br3ttb/Arduino-PID-Library  and  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ 

// Define Variables used by PID:
#define PID_SAMPLE_TIME  50  // milliseconds. 20 HZ
double ElbowSetpointEma, ElbowSetpoint, ElbowMeasured, pidOutElbow;
double ShoulderPanSetpointEma, ShoulderPanSetpoint, ShoulderPanMeasured, pidOutShoulderPan;
double ShoulderTiltSetpointEma, ShoulderTiltSetpoint, ShoulderTiltMeasured, pidOutShoulderTilt;
double ShoulderTurnSetpointEma, ShoulderTurnSetpoint, ShoulderTurnMeasured, pidOutShoulderTurn;

int ElbowMotorChannelPower;
int ShoulderPanMotorChannelPower;
int ShoulderTiltMotorChannelPower;
int ShoulderTurnMotorChannelPower;

// copies used by ReportState():
double _ElbowSetpoint, _ElbowMeasured, _ElbowMotorChannelPower;

/*
 - http://en.wikipedia.org/wiki/PID_controller#Loop_tuning (see Manual Tuning):
  If the system must remain online, one tuning method is to first set K_i and K_d values to zero.
  Increase the K_p until the output of the loop oscillates, then the K_p should be set to approximately half of that value for a "quarter amplitude decay" type response.
  Then increase K_i until any offset is corrected in sufficient time for the process. However, too much K_i will cause instability.
  Finally, increase K_d, if required, until the loop is acceptably quick to reach its reference after a load disturbance.
  However, too much K_d will cause excessive response and overshoot.
  A fast PID loop tuning usually overshoots slightly to reach the setpoint more quickly; however, some systems cannot accept overshoot,
  in which case an over-damped closed-loop system is required, which will require a K_p setting significantly less than half that of the K_p setting that was causing oscillation.[citation needed]
*/

// Specify the links and initial tuning parameters
//PID myPID_A(&ElbowMeasured, &pidOutElbow, &ElbowSetpointEma, 14.0, 5.0, 1.5, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale

PID myPID_A(&ElbowMeasured, &pidOutElbow, &ElbowSetpointEma,                       36.0, 0.0, 20.0, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale
PID myPID_B(&ShoulderPanMeasured, &pidOutShoulderPan, &ShoulderPanSetpointEma,     36.0, 0.0, 20.0, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale
PID myPID_C(&ShoulderTiltMeasured, &pidOutShoulderTilt, &ShoulderTiltSetpointEma,  20.0, 0.0, 10.0, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale
PID myPID_D(&ShoulderTurnMeasured, &pidOutShoulderTurn, &ShoulderTurnSetpointEma,  36.0, 0.0, 20.0, DIRECT);       // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale

// "servos" have 5KOhm feedbak potentiometers. Can’t use A4,5 – taken by I2C:
#define analogInPinA A0  // Analog input pin that the potentiometer A is attached to
#define analogInPinB A1  // Analog input pin that the potentiometer B is attached to
#define analogInPinC A2  // Analog input pin that the potentiometer C is attached to
#define analogInPinD A3  // Analog input pin that the potentiometer D is attached to

// standard LED
//#define led 13  // conflict - also assigned to in4 in the seeed motor shield

// PWM - http://www.righto.com/2009/07/secrets-of-arduino-pwm.html   pins 5,6 - 1kHz, 3,9,10,11 - 500Hz

// main motor shield:  http://www.seeedstudio.com/depot/Motor-Shield-p-913.html?cPath=73
#define in1A  8    // define I1 interface - hardwired on the shield
#define in2A  11   // define I2 interface - hardwired
#define EnA   9    // PWM enable motor A (marked M1) - Elbow

#define in3B  12   // define I3 interface - hardwired
#define in4B  13   // define I4 interface - hardwired
#define EnB   10   // PWM enable motor B (marked M2) - Shoulder Pan

// second motor shield: http://tronixlabs.com/news/tutorial-l298n-dual-motor-controller-module-2a-and-arduino/
#define in1C  2    // define I1 interface for second motor controller (separate from shield)
#define in2C  3    // define I2 interface
#define EnC   5    // PWM enable motor C (marked Out1-Out2) - Shoulder Tilt

#define in3D  4    // define I3 interface for second motor controller (separate from shield)
#define in4D  7    // define I4 interface
#define EnD   6    // PWM enable motor B (marked Out3-Out4) - Shoulder Turn

#define ElbowMotorChannel          0
#define ShoulderPanMotorChannel    1
#define ShoulderTiltMotorChannel   2
#define ShoulderTurnMotorChannel   3

// variables to compute exponential moving average:
int emaPeriod[4];
double valuePrev[4];
double multiplier[4];

// coming from Hand via I2C:
int palmSensorValue;
int servoCurrentValue;

void setup()
{
  // Elbow Angle actuator operate using degrees -90...90 scale
  ElbowSetpoint = 0.0;
  ElbowSetpointEma = 0.0;
  ElbowMeasured = 0.0;
  ElbowMotorChannelPower = 0;

  // Shoulder pan actuator operate using degrees -90...90 scale
  ShoulderPanSetpoint = 0.0;
  ShoulderPanSetpointEma = 0.0;
  ShoulderPanMeasured = 0.0;
  ShoulderPanMotorChannelPower = 0;

  // Shoulder tilt actuator operate using degrees -90...90 scale
  ShoulderTiltSetpoint = 0.0;
  ShoulderTiltSetpointEma = 0.0;
  ShoulderTiltMeasured = 0.0;
  ShoulderTiltMotorChannelPower = 0;

  // Shoulder turn actuator operate using degrees -90...90 scale
  ShoulderTurnSetpoint = 0.0;
  ShoulderTurnSetpointEma = 0.0;
  ShoulderTurnMeasured = 0.0;
  ShoulderTurnMotorChannelPower = 0;

  Serial.begin(115200);

  InitializeSerial();

  InitializeI2c();

  // turn the PID on and set its parameters:
  myPID_A.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. Motor PWM -255...255 can be too violent.
  myPID_A.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_A.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive
  
  myPID_B.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. Motor PWM -255...255 can be too violent.
  myPID_B.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_B.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_C.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. Motor PWM -255...255 can be too violent.
  myPID_C.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_C.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_D.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. Motor PWM -255...255 can be too violent.
  myPID_D.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_D.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  setEmaPeriod(ElbowMotorChannel, 100);
  setEmaPeriod(ShoulderPanMotorChannel, 100);
  setEmaPeriod(ShoulderTiltMotorChannel, 100);
  setEmaPeriod(ShoulderTurnMotorChannel, 100);
  
  enableMotors();

  Serial.println("Starting up finished");
}

// =========================== main loop timing ============================================
#define STD_LOOP_TIME 50000 // Fixed time loop of 50 milliseconds, 20HZ
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime;

void loop()
{
    // Wait here, if not time yet - we use a time fixed loop
    if (lastLoopUsefulTime < STD_LOOP_TIME)
    {
      while((micros() - loopStartTime) < STD_LOOP_TIME)
      {
        //Usb.Task();
        CheckSerial();
      }
    }

    ReadFeedbackPots();

    //Serial.println("p0");
    
    //digitalWrite(10, HIGH);
    loopStartTime = micros(); 
    
    while(CheckSerial())
    {
      ;  // false if no input, true if there is something to read
    }

    receiveI2cPacket(); // get palmSensorValue and servoCurrentValue from Hand

    // smooth movement by using ema:
    ema(ElbowMotorChannel);
    ema(ShoulderPanMotorChannel);
    ema(ShoulderTiltMotorChannel);
    ema(ShoulderTurnMotorChannel);
    
    ElbowRotation();  // uses ElbowSetpointEma
    ShoulderPanRotation();
    ShoulderTiltRotation();
    ShoulderTurnRotation();      
    
    //setLightsPower();
    
    // mark how much time we spent:
    lastLoopUsefulTime = micros() - loopStartTime;
    
    //digitalWrite(10, LOW);
}

/// using PID rotates elbow to position defined by "ElbowSetpoint"
void ElbowRotation()
{
  myPID_A.Compute();
  //ElbowMotorChannelPower = constrain(ElbowMotorChannelPower + pidOutElbow, -250, 250);
  ElbowMotorChannelPower = constrain(pidOutElbow, -250, 250);
  SetMotorPower(ElbowMotorChannel, ElbowMotorChannelPower);
}

void ShoulderPanRotation()
{
  myPID_B.Compute();
//  Serial.print(ShoulderPanSetpointEma);
//  Serial.print("   ");
//  Serial.print(ShoulderPanMeasured);
//  Serial.print("   ");
//  Serial.print(pidOutShoulderPan);
//  ShoulderPanMotorChannelPower = constrain(ShoulderPanMotorChannelPower + pidOutShoulderPan, -250, 250);
  ShoulderPanMotorChannelPower = constrain(pidOutShoulderPan, -250, 250);
//  Serial.print("   ");
//  Serial.println(ShoulderPanMotorChannelPower);
  SetMotorPower(ShoulderPanMotorChannel, ShoulderPanMotorChannelPower);
}

void ShoulderTiltRotation()
{
  myPID_C.Compute();
  //ShoulderTiltMotorChannelPower = constrain(ShoulderTiltMotorChannelPower + pidOutShoulderTilt, -250, 250);
  ShoulderTiltMotorChannelPower = constrain(pidOutShoulderTilt, -250, 250);
  SetMotorPower(ShoulderTiltMotorChannel, ShoulderTiltMotorChannelPower);
}

void ShoulderTurnRotation()
{
  myPID_D.Compute();
  //ShoulderTurnMotorChannelPower = constrain(ShoulderTurnMotorChannelPower + pidOutShoulderTurn, -250, 250);
  ShoulderTurnMotorChannelPower = constrain(pidOutShoulderTurn, -250, 250);
  SetMotorPower(ShoulderTurnMotorChannel, ShoulderTurnMotorChannelPower);
}

void ReadFeedbackPots()
{
      // read the feedback pots analog value:
      double feedbackPotValueA = (double)analogRead(analogInPinA);   // value read from the potA: 0...1023; 500 - middle     
      double feedbackPotValueB = (double)analogRead(analogInPinB);   // value read from the potB
      double feedbackPotValueC = (double)analogRead(analogInPinC);   // value read from the potC
      double feedbackPotValueD = (double)analogRead(analogInPinD);   // value read from the potD
   
      //Serial.println(feedbackPotValue);
    
      // actual rotation range is: Left - ... degrees, Right - ... degrees.
      // TODO: have exact degrees or tenth of degrees here as a "to" scale. 
      double mappedFeedbackA = feedbackPotValueA > 501 ? myMap(feedbackPotValueA, 501, 929, 0, -90) : myMap(feedbackPotValueA, 501, 124, 0, 90);    // map(value, fromLow, fromHigh, toLow, toHigh)
      double mappedFeedbackB = feedbackPotValueB > 501 ? myMap(feedbackPotValueB, 501, 929, 0, -90) : myMap(feedbackPotValueB, 501, 124, 0, 90);
      double mappedFeedbackC = feedbackPotValueC > 501 ? myMap(feedbackPotValueC, 501, 929, 0, -90) : myMap(feedbackPotValueC, 501, 124, 0, 90);
      double mappedFeedbackD = feedbackPotValueD > 501 ? myMap(feedbackPotValueD, 501, 929, 0, -90) : myMap(feedbackPotValueD, 501, 124, 0, 90);

/*        Serial.print(feedbackPotValueB);
      Serial.print(" : ");
      Serial.println(mappedFeedbackB);
//        os.os_sleep(1000);  // must be at least TICK_INTERVAL 2
*/
    
      ElbowMeasured = -mappedFeedbackA;  // account for rotation direction, may need negative sign here
      ShoulderPanMeasured = -mappedFeedbackB;
      ShoulderTiltMeasured = -mappedFeedbackC;
      ShoulderTurnMeasured = -mappedFeedbackD;
}

void ema(int ch)
{
  // smooth movement by using ema:
  switch (ch)
  {
    case ElbowMotorChannel:
      //ElbowSetpointEma = (valuePrev[ch] < -1000000.0) ? ElbowSetpoint : ((ElbowSetpoint - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      ElbowSetpointEma = ElbowSetpoint; // no ema
      valuePrev[ch] = ElbowSetpointEma;
      break;

    case ShoulderPanMotorChannel:
      //ShoulderPanSetpointEma = (valuePrev[ch] < -1000000.0) ? ShoulderPanSetpoint : ((ShoulderPanSetpoint - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      ShoulderPanSetpointEma = ShoulderPanSetpoint; // no ema
      valuePrev[ch] = ShoulderPanSetpointEma;
      break;

    case ShoulderTiltMotorChannel:
      //ShoulderTiltSetpointEma = (valuePrev[ch] < -1000000.0) ? ShoulderTiltSetpoint : ((ShoulderTiltSetpoint - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      ShoulderTiltSetpointEma = ShoulderTiltSetpoint; // no ema
      valuePrev[ch] = ShoulderTiltSetpointEma;
      break;
      
    case ShoulderTurnMotorChannel:
      //ShoulderTurnSetpointEma = (valuePrev[ch] < -1000000.0) ? ShoulderTurnSetpoint : ((ShoulderTurnSetpoint - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      ShoulderTurnSetpointEma = ShoulderTurnSetpoint; // no ema
      valuePrev[ch] = ShoulderTurnSetpointEma;
      break;
  }
}

void setEmaPeriod(int ch, int period)
{
  valuePrev[ch] = -2000000.0;  // signal to use ElbowSetpoint directly for the first point
  emaPeriod[ch] = period;
  multiplier[ch] = 2.0 / (1.0 + (double)emaPeriod[ch]);
}

double myMap(double x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - (double)in_min) * (double)(out_max - out_min) / (double)(in_max - in_min) + (double)out_min;
}


