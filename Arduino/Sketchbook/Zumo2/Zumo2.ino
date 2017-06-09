
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
#define QTR_THRESHOLD_L  2050     //R_RC_THRESHOLD
#define QTR_THRESHOLD_R  2750     //R_RC_THRESHOLD
 
 
// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     300
#define FORWARD_MAX_SPEED 400
#define FORWARD_BRAKING_SPEED 100
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms
 
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

void readDistanceSensors()
{
  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);                     

  // read the analog distance sensor value:
  distanceSensorValue = analogRead(sharpAnalogSensorPin); 

  // read digital distance sensors:
  digitalSensorLeft = !digitalRead(sharpDigitalSensorLeftPin);
  digitalSensorRight = !digitalRead(sharpDigitalSensorRightPin);
  digitalSensorRearLeft = !digitalRead(sharpDigitalSensorRearLeftPin);
  digitalSensorRearRight = !digitalRead(sharpDigitalSensorRearRightPin);

  closeRangeOn = digitalSensorLeft || digitalSensorRight;
  targetStraightOn = digitalSensorLeft && digitalSensorRight;
  
  // map it to the range of the analog out:
  distanceInches = sharpToInches(distanceSensorValue, closeRangeOn);  
}

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
   
  waitForButtonAndCountDown();
}

unsigned long timePushingStarted;
boolean isPushing = false;

void loop() 
{
  if (button.isPressed())
  {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown();
  }

  readDistanceSensors();

  if(distanceInches < DISTANCE_SENSOR_THRESHOLD)
    digitalWrite(LED_RED, HIGH);
  else
    digitalWrite(LED_RED, LOW);

  // print the results to the serial monitor:
  //Serial.print("sensor = " );                       
  //Serial.print(distanceSensorValue);      
  //Serial.print("\t inches = ");      
  //Serial.println(distanceInches);   

  // read line reflectancy sensors:
  unsigned int sensor_values[2];
  sensors.read(sensor_values);

  // process order reflectance first:   
  if (sensor_values[0] < QTR_THRESHOLD_L)  // left line sensor activated
  {
    // if left sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(200);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(300);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    lastTurnWasRight = true;
  }
  else if (sensor_values[1] < QTR_THRESHOLD_R)   // right line sensor activated
  {
    // if right sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(200);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(300);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    lastTurnWasRight = false;
  }
  else
  {
    if(digitalSensorRearLeft)
    {
      motors.setSpeeds(-FORWARD_MAX_SPEED, FORWARD_MAX_SPEED);
      delay(400);
    }
    else if(digitalSensorRearRight)
    {
      motors.setSpeeds(FORWARD_MAX_SPEED, -FORWARD_MAX_SPEED);
      delay(400);
    }
    else if(targetStraightOn)
    {
      if(!isPushing)
      {
        isPushing = true;
        timePushingStarted = millis();
        motors.setSpeeds(FORWARD_MAX_SPEED, FORWARD_MAX_SPEED);
      }
      else
      {
        if(millis() - timePushingStarted > 2000)
        {
          if(lastTurnWasRight)
          {
            lastTurnWasRight = false;
            motors.setSpeeds(-FORWARD_MAX_SPEED, FORWARD_MAX_SPEED);
          }
          else
          {
            lastTurnWasRight = true;
            motors.setSpeeds(FORWARD_MAX_SPEED, -FORWARD_MAX_SPEED);
          }
          delay(500);
        }
        else
        {
          motors.setSpeeds(FORWARD_MAX_SPEED, FORWARD_MAX_SPEED);
        }
        
        if(millis() - timePushingStarted > 6000)
        {
          isPushing = false;
        }
      }
    }
    else
    {
      isPushing = false;
      if(digitalSensorLeft)
      {
        motors.setSpeeds(0, FORWARD_MAX_SPEED);
      }
      else if(digitalSensorRight)
      {
        motors.setSpeeds(FORWARD_MAX_SPEED, 0);
      }
      else if(distanceInches < DISTANCE_SENSOR_THRESHOLD)
      {
        // analog distance sensor reads something ahead, go straight
        motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      }
      // otherwise, rotate a bit:
      else if(lastTurnWasRight)
      {
        motors.setSpeeds(FORWARD_SPEED, FORWARD_BRAKING_SPEED);
      }
      else
      {
        motors.setSpeeds(FORWARD_BRAKING_SPEED, FORWARD_SPEED);
      }
    }
  }
}

int sharpToInches(int sv, boolean closeRangeOn)
{
  if(closeRangeOn)
  {
    // 0-4 inches is within the range of Sharp GP2Y0D810Z0F Digital Distance Sensors 0-10cm
    // 220 - 0
    // 330 - 1
    // 350 - 2
    // 400 - 3
    // 500 - 4
    if(sv < 270)
      return 0;
    if(sv < 340)
      return 1;
    if(sv < 370)
      return 2;
    if(sv < 450)
      return 3;
    return 4;
  }
  
  if(sv > 400)
    return 7;
    
  if(sv < 90)
    return 100;  

  return 50 - (sv - 90) * 42 / 410;

/*
  if(sv < 230)
  {
    return 50 - (sv - 90) * 30 / 140;
    //return (int)(50.0f - ((float)sv - 90.0f) * 30.0f / 140.0f);
  }
  
  return 20 - (sv - 230) * 12 / 270;
  //return (int)(20.0f - ((float)sv - 230.0f) * 12.0f / 270.0f);
*/
}

