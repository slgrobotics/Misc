
#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
 
#define LED     13
#define LED_RED 5
 
// these might need to be tuned for different lighting conditions, surfaces, etc.
//#define QTR_A_THRESHOLD           900
#define QTR_RC_THRESHOLD            3800
#define DISTANCE_SENSOR_THRESHOLD   45
 
// set according to the type of QTR sensor you're using
#define QTR_THRESHOLD_L         2050     //R_RC_THRESHOLD
#define QTR_THRESHOLD_R         2750     //R_RC_THRESHOLD
 
#define QTR_THRESHOLD_RAMP_L    2500     //R_RC_THRESHOLD
#define QTR_THRESHOLD_RAMP_R    3000     //R_RC_THRESHOLD
 
// these might need to be tuned for different motor types
#define REVERSE_SPEED           200 // 0 is stopped, 400 is full speed
#define TURN_SPEED              200
#define FORWARD_SPEED           300
#define FORWARD_MAX_SPEED       400
#define FORWARD_BRAKING_SPEED   100
#define FORWARD_RAMP_SPEED      200
#define REVERSE_DURATION        200 // ms
#define TURN_DURATION           300 // ms
 
ZumoBuzzer buzzer;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON); // pushbutton on pin 12
 
// uncomment according to the type of QTR sensor you're using
QTRSensorsRC sensors((unsigned char[]){A0, A3}, 2); // two QTR-1RC sensors on pins A0 (left) and A3 (right)
//QTRSensorsAnalog sensors((unsigned char[]){A0, A3}, 2); // two QTR-1A sensors on pins A0 (left) and A3 (right)
 
// Analog input pin that the Sharp GP2Y0A02YK0F 20-150cm is attached to:
const int sharpAnalogSensorPin = A2;
// input pins Sharp GP2Y0D810Z0F Digital Distance Sensors 0-10cm are attached to:
const int sharpDigitalSensorLeftPin = 2;
const int sharpDigitalSensorRightPin = 4; 
const int sharpDigitalSensorRearLeftPin = A4;    // used as digital
const int sharpDigitalSensorRearRightPin = A5;   // used as digital 

void waitForButtonAndCountDown()
{
  digitalWrite(LED, HIGH);
  button.waitForButton();
  digitalWrite(LED, LOW);
   
  // play audible countdown
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);  
  delay(1000);
}

boolean digitalSensorLeft;
boolean digitalSensorRight;
boolean digitalSensorRearLeft;
boolean digitalSensorRearRight;
boolean closeRangeOn;
boolean targetStraightOn;
int distanceSensorValue;
int distanceInches;
boolean lastTurnWasRight;
unsigned long timeEnteredRing = 0;
unsigned long timePushingStarted;
boolean isPushing = false;
int behavior = 0;  // 0 - line following; 1 - fight
int bothHitCounter = 0;


void setup()
{
  pinMode(sharpDigitalSensorLeftPin, INPUT);     
  pinMode(sharpDigitalSensorRightPin, INPUT);     
  pinMode(sharpDigitalSensorRearLeftPin, INPUT);     
  pinMode(sharpDigitalSensorRearRightPin, INPUT);     

  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 

  // uncomment if necessary to correct motor directions
  motors.flipLeftMotor(true);
  motors.flipRightMotor(true);
   
  pinMode(LED, HIGH);
  pinMode(LED_RED, HIGH);

  behavior = 1;  // switch to fighting mode
  //behavior = 0;  // do line following first
   
  waitForButtonAndCountDown();
}

void loop() 
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }
  
  switch(behavior)
  {
    case 0:
      digitalWrite(LED, HIGH);
      doLineFollowing();
      break;

    case 1:
      digitalWrite(LED, LOW);
      doFight();
      break;
  }
}



