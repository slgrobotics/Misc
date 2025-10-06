
#include <Servo.h>

Servo servo_Steer;      // create servo object to control a servo
Servo servo_Throttle;
Servo servo_Extra;

void MotorsInit()
{
  servo_Steer.attach(PIN_STEER, 1000, 2000); // Minimum and maximum pulse width (in µs) to go from 0° to 180°.
  servo_Throttle.attach(PIN_THROTTLE, 1000, 2000);
  servo_Extra.attach(PIN_EXTRA, 1000, 2000);

  // All angles are adjusted for particular servo placement:
  servo_Steer.write(90);    // newtral - angle 0
  servo_Throttle.write(90);
  servo_Extra.write(45);

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
  
  angle_steer = map(pwm_steer, -510, 510, 140, 50);
  angle_throttle = map(pwm_throttle, -510, 510, 130, 40);

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
  servo_Extra.write(isJoystickPressed() ? 135 : 45);
}
