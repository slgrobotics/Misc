
#include <Servo.h>

Servo servo_Steer;      // create servo object to control a servo
Servo servo_Throttle;
Servo servo_Extra;

void MotorsInit()
{
  servo_Steer.attach(PIN_STEER, 1000, 2000); // Minimum and maximum pulse width (in µs) to go from 0° to 180°.
  servo_Throttle.attach(PIN_THROTTLE, 1000, 2000);
  servo_Extra.attach(PIN_EXTRA, 1000, 2000);
  
  servo_Steer.write(45); // angle 0
  servo_Throttle.write(45);
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
  angle_steer = map((pwm_L - pwm_R) / 2, -510, 510, 135, 45);
  angle_throttle = map((pwm_L + pwm_R) / 2, -510, 510, 135, 45);

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
