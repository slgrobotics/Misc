
boolean isJoystickActive()
{
  return !digitalRead(JOYSTICK_ACTIVATE_PIN);
}

boolean isJoystickPressed()
{
  return !digitalRead(JOYSTICK_PRESSED_PIN);
}

double joystickX()
{
  double jx = constrain(map((double)analogRead(JOYSTICK_X_PIN) + JOYSTICK_TRIM_X, 0.0, 1024.0, -100.0, 100.0), -100.0, 100.0);

  if (abs(jx) < DEADZONE_JS) // deadzone
    jx = 0.0;

  return jx;
}

double joystickY()
{
  double jy = constrain(map((double)analogRead(JOYSTICK_Y_PIN) + JOYSTICK_TRIM_Y, 0.0, 1024.0, -100.0, 100.0), -100.0, 100.0);

  if (abs(jy) < DEADZONE_JS) // deadzone, percent of max stick deflection
    jy = 0.0;

  return jy;
}

// for percentage speed output, -100...100:
const int OUT_MAX = 100;
const int OUT_SHIFT = 0;

void computeJoystickSpeeds()
{
  double leftMix;
  double rightMix;

  double x = joystickX() / 100.0;
  double y = joystickY() / 100.0;

  // we need absolute values later:
  double ax = abs(x);
  double ay = abs(y);

  // transition zone flag. This is two sectors between 0 and -45 degrees:
  boolean tz = (y < 0 && ay < ax) ? true : false;

  // we scale output in diagonal directions:
  double alpha = (ax == 0 || ay == 0) ? 0 : atan(ay < ax ? ay / ax : ax / ay);
  double factor = tz ? cos(alpha / 2.0) : cos(alpha);

#ifdef USE_ARCADE_DRIVE
  // Mix for arcade drive
  // see https://github.com/slgrobotics/Misc/blob/master/Arduino/Sketchbook/RC_ArcadeMixer/RC_ArcadeMixer.ino
  if (y >= 0.0)
  {
    // normal zone, forward movement. Upper sector -90...+90 degrees:
    leftMix = y + x;
    rightMix = y - x;
  }
  else if (!tz)
  {
    // backwards driving zone, inverted output. Lower sector, 135...215 degrees:
    leftMix = y - x;
    rightMix = y + x;
  }
  else if (x > 0)
  {
    // right transition zone, 90...135 degrees:
    leftMix = 2.0 * y + x;
    rightMix = -(y + x);
  }
  else
  {
    // left transition zone, 215...270 (a.k.a. -90) degrees:
    leftMix = -y + x;
    rightMix = 2.0 * y - x;
  }
#else // USE_ARCADE_DRIVE
  leftMix = y + x;
  rightMix = y - x;
#endif // USE_ARCADE_DRIVE

  factor = factor * factor * 1.0;   // experimental

  // scale it to desired output, convert to integer:
  joystickSpeedL = (int)(leftMix * factor * (double)OUT_MAX);
  joystickSpeedR = (int)(rightMix * factor * (double)OUT_MAX);

  joystickSpeedL = constrain(joystickSpeedL, -OUT_MAX, OUT_MAX) + OUT_SHIFT;
  joystickSpeedR = constrain(joystickSpeedR, -OUT_MAX, OUT_MAX) + OUT_SHIFT;
}
