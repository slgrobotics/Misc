
#define SERIAL_ROBOREALM
//#define SERIAL_MY

#include <AVR_OS.h>

// see http://www.chrismoos.com/2012/12/05/avr-os-multitasking-on-arduino 
// works on UNO, not on Mega  

#include <PID_v1.h>

#include <Servo.h>

Servo gimbalPitchServo;  // connected to camera gimbal pitch control, pin D9

// for PID see https://github.com/br3ttb/Arduino-PID-Library  and  http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/ 

// Define Variables used by PID:
double HeadPanSetpointEma, HeadPanSetpoint, HeadPanMeasured, HeadPanMotorChannelPower;
#define PID_SAMPLE_TIME  10	// milliseconds.

// copies used by ReportState():
double _HeadPanSetpoint, _HeadPanMeasured, _HeadPanMotorChannelPower;
int _lightsPowerMks;

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
//PID myPID(&HeadPanMeasured, &HeadPanMotorChannelPower, &HeadPanSetpointEma, 30.0, 15.0, 1.0, DIRECT);    // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale
//PID myPID(&HeadPanMeasured, &HeadPanMotorChannelPower, &HeadPanSetpointEma, 10.0, 5.0, 0.3, DIRECT);    // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale
//PID myPID(&HeadPanMeasured, &HeadPanMotorChannelPower, &HeadPanSetpointEma, 20.0, 10.0, 0.6, DIRECT);    // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale
PID myPID(&HeadPanMeasured, &HeadPanMotorChannelPower, &HeadPanSetpointEma, 8.0, 3.0, 1.0, DIRECT);    // double Kp, Ki, Kd, DIRECT or REVERSE - for +-90 scale

// turret "servo" 10KOhm feedbak potentiometer:
#define analogInPin A0  // Analog input pin that the potentiometer is attached to

// standard LED
#define led 13

#define gimbalPitchPin 9

// connect arduino pins 2...5 to motor shield in1...in4 (Direction)  and pin 9,10 to EnA,EnB (PWM)
// PWM pins must be one of 3, 5, 6, 9, 10, or 11 for Arduino Uno
// For pins 3,9,10,11 it is approximately 488 Hz. For pins 5 and 6, it is about 977 Hz
// see http://arduino-info.wikispaces.com/MotorDrivers  for L298N based H-Bridge description
#define EnA 5  
#define EnB 6  
#define in1 2
#define in2 3                        
#define in3 4                          
#define in4 7                          

#define HeadPanMotorChannel   1                          
#define LightHbridgeChannel   2                          

int lightsPowerMks;
int GimbalPitchSetpointMks;

// variables to compute exponential moving average:
int emaPeriod;
double valuePrev;
double multiplier;

//spinlock_t myLock;

// Create instance of OS multitasker
AVR_OS os = AVR_OS();

void setup()
{
  // gimbal and light actuators operate using Servo microsecond scale, 800 to 2200
  
  gimbalPitchServo.attach(gimbalPitchPin);    // attaches the servo on pin 9 to the servo object. Uses Timer 1.
  GimbalPitchSetpointMks = 1500;     // level the camera - gimball Pitch input
  gimbalPitchServo.write(GimbalPitchSetpointMks);
  lightsPowerMks = 800;           // Servo-like  800 - off, 2200 - on

  // head pan actuator operate using degrees -90...90 scale
  HeadPanSetpoint = 0.0;
  HeadPanSetpointEma = 0.0;
  HeadPanMeasured = 0.0;
  HeadPanMotorChannelPower = 0.0;

  Serial.begin(57600);

  InitializeSerial();

  // turn the PID on and set its parameters:
  myPID.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. Motor PWM -255...255 can be too violent.
  myPID.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive
  
  setEmaPeriod(100);
  
  enableMotors();
  //gimbalPitchRotation();
  setLightsPower();

  os.os_init();
  //os.spinlock_init(&myLock);

  os.os_schedule_task(taskReadFeedbackPot, NULL, 0);
  os.os_schedule_task(taskActuate, NULL, 0);
  //os.os_schedule_task(taskRemote, NULL, 0);
  os.os_schedule_task(taskReportState, NULL, 0);

  //Serial.println("Starting up finished");
}

void loop()
{
  // -----------------------------------------------------------
  /*
  SetMotorPower(1, 0);
  delay(3000);
  return;
  
  int value;
  // read the feedback pot analog value.
  //   124 = 90 degrees to the left
  //   501 = clicked at the middle
  //   940 = 90 degrees to the rigt
  int feedbackPotValue = analogRead(analogInPin);     

  Serial.print("POT=");
  Serial.println(feedbackPotValue);
  return;

  value = feedbackPotValue / 2 - 255;
  SetMotorPower(1, value);  // head pan - motor rotates same direction as POT, -255=counterclockwise
  return;

  for(value = -255 ; value <= 255; value+=1)
  {
    SetMotorPower(1, value);  // head pan
    //SetMotorPower(2, value);  // bubble blower fan
    delay(50);
  } 

  SetMotorPower(1, 0);
  //SetMotorPower(2, 0);
  delay(3000);
  */
  // -----------------------------------------------------------
  
  os.os_loop(); 
}

// =========================== main loop timing ============================================
#define STD_LOOP_TIME 10000 // Fixed time loop of 10 milliseconds
unsigned long lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime;

// Define task 1
void taskActuate(void *arg)
{
    while(1)
    {  
      // Wait here, if not time yet - we use a time fixed loop
      if (lastLoopUsefulTime < STD_LOOP_TIME)
      {
        while((micros() - loopStartTime) < STD_LOOP_TIME)
        {
          //Usb.Task();
          //os.os_yield();
          os.os_sleep(2);  // must be at least TICK_INTERVAL 2
        }
      }

      //Serial.println("p0");
      
      //os.spinlock_acquire(&myLock);

      //Serial.println("p1");      

      //digitalWrite(10, HIGH);
      loopStartTime = micros(); 
      
      while(CheckSerial())
      {
        ;  // false if no input, true if there is something to read
      }

      cli();
      
      // smooth movement by using ema:
      ema();
    	
      headPanRotation();  // uses HeadPanSetpointEma
      //setLightsPower();
      gimbalPitchRotation();
      
      sei();
    
      // mark how much time we spent:
      lastLoopUsefulTime = micros() - loopStartTime;
      
      //digitalWrite(10, LOW);

      //os.spinlock_release(&myLock);
    }
}

// Define task 2 - read feedback POT to produce HeadPanMeasured
void taskReadFeedbackPot(void *arg)
{
    while(1) {
        // read the feedback pot analog value:
        double feedbackPotValue = (double)analogRead(analogInPin);   // value read from the pot: 0 head turned all way to the left; 1023 - to the right; 500 - middle     
     
        //Serial.println(feedbackPotValue);
        //os.os_sleep(1000);  // must be at least TICK_INTERVAL 2
      
        // actual rotation range is: Left - ... degrees, Right - ... degrees.
        // TODO: have exact degrees or tenth of degrees here as a "to" scale. 
        double mappedFeedback = feedbackPotValue > 501 ? myMap(feedbackPotValue, 501, 929, 0, -90) : myMap(feedbackPotValue, 501, 124, 0, 90);    // map(value, fromLow, fromHigh, toLow, toHigh)

//        Serial.print(feedbackPotValue);
//        Serial.print(" ");
//        Serial.println(mappedFeedback);
//        os.os_sleep(1000);  // must be at least TICK_INTERVAL 2
      
        //os.spinlock_acquire(&myLock);  
        cli();
        HeadPanMeasured = mappedFeedback;  // account for rotation direction, may need negative sign here
        sei();
        //os.spinlock_release(&myLock);
        
        os.os_sleep(10);  // must be at least TICK_INTERVAL 2
        //os.os_yield();
    }
}

// Define task 3 - check for commands over the serial line
void taskRemote(void *arg)
{
    while(1)
    {
      if(!CheckSerial())  // false if no input
      {      
        os.os_sleep(20);  // no input; must be at least TICK_INTERVAL 2
      }
      else
      {
        os.os_yield();  // have more to read
      }
    }
}

#define PRINT_LOOP_TIME 100000 // Fixed time loop of 100 milliseconds
unsigned long printStartTime;
unsigned long lastPrintUsefulTime = PRINT_LOOP_TIME;

// Define task 4 - report values over the serial line
void taskReportState(void *arg)
{
    while(1)
    {
        // Wait here, if not time yet - we use a time fixed loop
        if (lastPrintUsefulTime < PRINT_LOOP_TIME)
        {
          while((micros() - printStartTime) < PRINT_LOOP_TIME)
          {
            //Usb.Task();
            //os.os_yield();
            os.os_sleep(10);  // must be at least TICK_INTERVAL 2
          }
        }
        
        printStartTime = micros();
        
        //os.spinlock_acquire(&myLock);
        cli();
        _HeadPanSetpoint = HeadPanSetpoint;
        _HeadPanMeasured = HeadPanMeasured;
        _HeadPanMotorChannelPower = HeadPanMotorChannelPower;
        _lightsPowerMks = lightsPowerMks;
        sei();
        //os.spinlock_release(&myLock);

        ReportState();    
        // mark how much time we spent:
        lastPrintUsefulTime = micros() - printStartTime;
    }
}

/// using PID rotates head to position defined by "HeadPanSetpoint"
void headPanRotation()
{
  myPID.Compute();
  SetMotorPower(HeadPanMotorChannel, (int)round(HeadPanMotorChannelPower));
}

void gimbalPitchRotation()
{
  gimbalPitchServo.write(GimbalPitchSetpointMks);
}

void setLightsPower()
{
  int lightsPower = map(lightsPowerMks, 800, 2200, 0, 255);
  SetMotorPower(LightHbridgeChannel, lightsPower);
}

void ema()
{
  // smooth movement by using ema:
  HeadPanSetpointEma = (valuePrev < -1000000.0) ? HeadPanSetpoint : ((HeadPanSetpoint - valuePrev) * multiplier + valuePrev);  // ema
  //HeadPanSetpointEma = HeadPanSetpoint; // no ema
  valuePrev = HeadPanSetpointEma;
}

void setEmaPeriod(int period)
{
  valuePrev = -2000000.0;  // signal to use HeadPanSetpoint directly for the first point
  emaPeriod = period;
  multiplier = 2.0 / (1.0 + (double)emaPeriod);
}

double myMap(double x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - (double)in_min) * (double)(out_max - out_min) / (double)(in_max - in_min) + (double)out_min;
}


