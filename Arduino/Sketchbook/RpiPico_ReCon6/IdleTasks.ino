
// While waiting for a fixed time loop, check out Pixy camera, joystick...

void idleTasks()
{
  pixyBlocksCount += readPixyCamera();

  if(isControlByJoystick())
  {
    // joystick on ADC0 and ADC1:

    computeJoystickSpeeds();
    
    desiredSpeedL = joystickSpeedL;
    desiredSpeedR = joystickSpeedR;
  }

  if(isJoystickPressed())
  {
    encodersReset();
    odometryReset();
  }
}
