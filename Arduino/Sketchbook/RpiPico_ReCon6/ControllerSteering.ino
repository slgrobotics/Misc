
void controlSteering()
{
  // Based on steeringControlEffort, adjust desired speeds.

  desiredSpeedR += steeringControlEffort;
  desiredSpeedL -= steeringControlEffort;

  desiredSpeedR = constrain(desiredSpeedR, -100.0, 100.0);
  desiredSpeedL = constrain(desiredSpeedL, -100.0, 100.0);
}
