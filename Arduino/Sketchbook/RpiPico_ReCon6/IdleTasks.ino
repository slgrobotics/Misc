
// While waiting for a fixed time loop, check out Pixy camera, joystick, MPU...

void idleTasks()
{
  pixyBlocksCount += readPixyCamera();

  //digitalWrite(DBG2_PIN, HIGH);
  processMpu();
  //digitalWrite(DBG2_PIN, LOW);

  if(isControlByJoystick())
  {
    // joystick on ADC0 and ADC1:

    computeJoystickSpeeds();
    
    desiredSpeedL = joystickSpeedL;
    desiredSpeedR = joystickSpeedR;

    if(isJoystickPressed())
    {
      encodersReset();
      odometryReset();
    }
  }
}
