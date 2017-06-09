void doLineFollowing()
{
  // read all distance sensors - we need forward close range ones to be able to confront opponent waiting for us at the border:
  readDistanceSensors();
  
  if(closeRangeOn)
  {
    // opponent waiting for us at the border:
    behavior = 1;  // switch to fighting mode
    timeEnteredRing = millis();
    motors.setSpeeds(FORWARD_MAX_SPEED, FORWARD_MAX_SPEED);
    buzzer.playNote(NOTE_G(4), 100, 15);  
    buzzer.playNote(NOTE_A(4), 100, 15);  
    buzzer.playNote(NOTE_G(4), 100, 15);  
    return;
  }
  
  // read line reflectancy sensors - black-on-white line following:
  unsigned int sensor_values[2];
  sensors.read(sensor_values);
  
  boolean leftHit = sensor_values[0] > QTR_THRESHOLD_RAMP_L;     // left line sensor activated
  boolean rightHit = sensor_values[1] > QTR_THRESHOLD_RAMP_R;    // right line sensor activated

  if(leftHit && rightHit)  // both reflective sensors indicate "black"
  {
    if(bothHitCounter++ > 10)
    {
      // we entered the black ring - proceed to fight mode
      behavior = 1;  // switch to fighting mode
      timeEnteredRing = millis();
  //motors.setSpeeds(0, 0);
      motors.setSpeeds(FORWARD_MAX_SPEED, FORWARD_MAX_SPEED);
      buzzer.playNote(NOTE_A(4), 500, 15);  
      //delay(300);
    }
    else
    {
      // it could be a glitch, proceed cautiously:
      motors.setSpeeds(50, 50);
      delay(30);
    }
    return; 
  }
  
  bothHitCounter = 0;  // at least one sensor on "white"

  int rightSpeedFactor = leftHit ? ((sensor_values[0] - QTR_THRESHOLD_RAMP_L) >> 2) : 0;
  int leftSpeedFactor = rightHit ? ((sensor_values[1] - QTR_THRESHOLD_RAMP_R) >> 2) : 0;

  if (leftHit)  // left line sensor activated
  {
    // if left sensor detects line, turn to the left
    motors.setSpeeds(FORWARD_RAMP_SPEED - rightSpeedFactor, FORWARD_RAMP_SPEED + rightSpeedFactor);
  }
  else if (rightHit)   // right line sensor activated
  {
    // if right sensor detects line, turn to the right
    motors.setSpeeds(FORWARD_RAMP_SPEED + leftSpeedFactor, FORWARD_RAMP_SPEED - leftSpeedFactor);
  }
  else
  {
    motors.setSpeeds(FORWARD_RAMP_SPEED, FORWARD_RAMP_SPEED);
  }
}

