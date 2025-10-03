
#include <Servo.h>

Servo servo_Steer;      // create servo object to control a servo
Servo servo_Throttle;

void MotorsInit()
{
  servo_Steer.attach(PIN_STEER, 1000, 2000); // Minimum and maximum pulse width (in µs) to go from 0° to 180°.
  servo_Steer.attach(PIN_THROTTLE, 1000, 2000);
  
  servo_Steer.write(0); // angle 0
  servo_Throttle.write(0);

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
  angle_steer = map(pwm_L - pwm_R, -510, 510, -180, 180);
  angle_throttle = map((pwm_L + pwm_R) / 2, -510, 510, -180, 180);
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
}
