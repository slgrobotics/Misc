
// for 0-150cm Sharp sensor:
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

void readRearSensors()
{
  digitalSensorRearLeft = !digitalRead(sharpDigitalSensorRearLeftPin);
  digitalSensorRearRight = !digitalRead(sharpDigitalSensorRearRightPin);
}


