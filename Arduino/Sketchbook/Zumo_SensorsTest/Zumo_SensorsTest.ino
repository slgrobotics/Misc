#include <QTRSensors.h>
 
#define LED     13
#define LED_RED 5
 
// these might need to be tuned for different lighting conditions, surfaces, etc.
//#define QTR_A_THRESHOLD           900
#define QTR_RC_THRESHOLD            3800
#define DISTANCE_SENSOR_THRESHOLD    45
 
// set according to the type of QTR sensor you're using
#define QTR_THRESHOLD     QTR_RC_THRESHOLD
 
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

int distanceSensorValue;
boolean digitalSensorLeft;
boolean digitalSensorRight;
boolean digitalSensorRearLeft;
boolean digitalSensorRearRight;
boolean closeRangeOn;
boolean targetStraightOn;
int distanceInches;

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

  pinMode(LED, HIGH);
  pinMode(LED_RED, HIGH);
}

void loop() 
{
  readDistanceSensors();

  if(distanceInches < DISTANCE_SENSOR_THRESHOLD)
    digitalWrite(LED_RED, HIGH);
  else
    digitalWrite(LED_RED, LOW);

  // print the results to the serial monitor:
  Serial.print("sensor raw = " );                       
  Serial.print(distanceSensorValue);      
  Serial.print("\t inches = ");      
  Serial.print(distanceInches);   

  Serial.print("\t FrontL = ");      
  Serial.print(digitalSensorLeft);   
  Serial.print("\t FrontR = ");      
  Serial.print(digitalSensorRight);   
  Serial.print("\t RearL = ");      
  Serial.print(digitalSensorRearLeft);   
  Serial.print("\t RearR = ");      
  Serial.print(digitalSensorRearRight);   

  // read line reflectancy sensors:
  unsigned int sensor_values[2];
  sensors.read(sensor_values);

  Serial.print("\t LineL = ");      
  Serial.print(sensor_values[0]);   
  Serial.print("\t LineR = ");      
  Serial.print(sensor_values[1]);   

  Serial.println("");
  delay(200);
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

