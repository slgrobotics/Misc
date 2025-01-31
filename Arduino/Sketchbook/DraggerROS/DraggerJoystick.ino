#define JOYSTICK_ACTIVATE_PIN 22
#define JOYSTICK_PRESSED_PIN 24

const int AD_Xpin = 0;         // Joystick X pin
const int AD_Ypin = 1;         // Joystick Y pin

#define TRIM_X 0
#define TRIM_Y 0


#define DEADZONE_JS 8.0

boolean isControlByJoystick()
{
  return !digitalReadFast(JOYSTICK_ACTIVATE_PIN);
}

boolean isJoystickPressed()
{
  return !digitalReadFast(JOYSTICK_PRESSED_PIN);
}

double joystickX()
{
  double jx = map((double)analogRead(AD_Xpin) + TRIM_X, 0.0, 1023.0, -100.0, 100.0);

  if (abs(jx) < DEADZONE_JS) // deadzone
    jx = 0.0;

  return jx;
}

double joystickY()
{
  double jy = map((double)analogRead(AD_Ypin) + TRIM_Y, 27.0, 1023.0, -100.0, 95.0) + 5.0;

  if (abs(jy) < DEADZONE_JS) // deadzone
    jy = 0.0;

  return jy;
}

// for percentage speed output, -100...100:
const int OUT_MAX = 100;
const int OUT_SHIFT = 0;

void computeJoystickSpeeds()
{
  // see RC_ArcadeMixer.ino

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

  // Mix for arcade drive
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

  factor = factor * factor * 1.1;   // experimental

  // scale it to desired output, convert to integer:
  joystickSpeedL = (int)(leftMix * factor * (double)OUT_MAX);
  joystickSpeedR = (int)(rightMix * factor * (double)OUT_MAX);

  joystickSpeedL = constrain(joystickSpeedL, -OUT_MAX, OUT_MAX) + OUT_SHIFT;
  joystickSpeedR = constrain(joystickSpeedR, -OUT_MAX, OUT_MAX) + OUT_SHIFT;
}
