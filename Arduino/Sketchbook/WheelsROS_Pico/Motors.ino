
#include <Servo.h>

// see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/miniPRO

Servo servo_Steer;      // create servo object to control a servo
Servo servo_Throttle;
Servo servo_Extra;

const int steer_neutral = 85; // degrees
const int steer_throw = 50;

const int throttle_neutral = 100; // degrees
const int throttle_throw = 55;

const int extra_off = 45; // degrees
const int extra_on = 135;

void MotorsInit()
{
  servo_Steer.attach(PIN_STEER, 1000, 2000); // Minimum and maximum pulse width (in µs) to go from 0° to 180°.
  servo_Throttle.attach(PIN_THROTTLE, 1000, 2000);
  servo_Extra.attach(PIN_EXTRA, 1000, 2000);

  // All angles are adjusted for particular servo placement:
  servo_Steer.write(steer_neutral);    // newtral - angle 0
  servo_Throttle.write(throttle_neutral);
  servo_Extra.write(extra_off);

  pwm_R = 0;
  pwm_L = 0;

  set_motors();
}

void constrainPwm()
{
  // Maximum / Minimum Limitations:
  pwm_R = constrain(pwm_R, -255, 255);
  pwm_L = constrain(pwm_L, -255, 255);
}

void pwmToAngles()
{
  int pwm_steer = (pwm_L - pwm_R) / 2;
  int pwm_throttle = (pwm_L + pwm_R) / 2;

  int max_pwm = abs(pwm_throttle) + abs(pwm_steer);

  if(max_pwm > 255) {
    int excess = abs(max_pwm - 255);
    if(pwm_throttle > 0) {
      pwm_throttle -= excess;
    } else {
      pwm_throttle += excess;
    }
  }

  angle_steer = map(pwm_steer, -510, 510, steer_neutral + steer_throw, steer_neutral - steer_throw);
  angle_throttle = map(pwm_throttle, -510, 510, throttle_neutral + throttle_throw, throttle_neutral - throttle_throw);

  /*
  Serial.print("angles: steer: ");
  Serial.print(angle_steer);
  Serial.print("  throttle: ");
  Serial.println(angle_throttle);
  */
}

// ******************************************************************************************
//
//   Set motor power for both motors. Positive is forward.
//
// ******************************************************************************************
void set_motors()
{
  constrainPwm();

  pwmToAngles();

  servo_Steer.write(angle_steer);
  servo_Throttle.write(angle_throttle);
  servo_Extra.write(isJoystickPressed() ? extra_on : extra_off);
}
