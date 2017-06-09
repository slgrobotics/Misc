
int grabState = GRAB_NONE;
long lastGrab = 0;

bool CheckGrab()
{
  bool ret = false;
  
  if (millis() - lastGrab > timeToHoldGrabMs)
  {
    if (isPalmSensorActivated && grabState == GRAB_NONE)
    {
      grabState = GRAB_FULL;
      lastGrab = millis();
      ret = true;
    }
    else if(grabState == GRAB_FULL)
    {
      grabState = GRAB_RELEASE;
      lastGrab = millis();
      ret = true;
    }
  }
  else
  {
    ret = true;
  }

  switch (grabState)
  {
    default:
      break;
      
    case GRAB_RELEASE:
      Grab();
      grabState = GRAB_NONE;
      break;

    case GRAB_FULL:
      Grab();
      ret = true;
      break;
  }
  
  return ret;
}

void ResetGrab()
{
    grabState = GRAB_RELEASE;
    Grab();
    grabState = GRAB_NONE;
}

void Grab()
{
  switch (grabState)
  {
    default:
      break;
      
    case GRAB_RELEASE:
      servoThumb.write(160);
      servoIndex.write(150);
      servoMiddle.write(70);
      servoPinky.write(100);
      break;

    case GRAB_FULL:
      servoThumb.write(120);
      servoIndex.write(90);
      servoMiddle.write(140);
      servoPinky.write(30);
      break;
  }
}

