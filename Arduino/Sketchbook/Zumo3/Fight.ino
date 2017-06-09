
void doFight()
{
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
  
  boolean doReflective = (millis() - timeEnteredRing) > 1000;  // ignore white border for 1 second after entering the ring

  // process order reflectance first:   
  if (doReflective && sensor_values[0] < QTR_THRESHOLD_L)  // left line sensor activated
  {
    // if left sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(200);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(300);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
    lastTurnWasRight = true;
  }
  else if (doReflective && sensor_values[1] < QTR_THRESHOLD_R)   // right line sensor activated
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
      while(digitalSensorRearLeft)
      {
        readRearSensors();
      }
      delay(100);
    }
    else if(digitalSensorRearRight)
    {
      motors.setSpeeds(FORWARD_MAX_SPEED, -FORWARD_MAX_SPEED);
      while(digitalSensorRearRight)
      {
        readRearSensors();
      }
      delay(100);
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
        if(millis() - timePushingStarted > 4000)
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

