// **************************
// Calculate the pwm
// **************************
void pwm_calculate()
{
  // we spend 126us here
  //digitalWrite(10, HIGH);
  int direction_error;

  range += (Ldistance + Rdistance) * 0.5 + forward_factor;
  range *= 0.9;

  direction_error = Ldistance - Rdistance;
  direction_error_all += direction_error;

  wheel_speed = range - last_wheel;
  last_wheel = range;

  // angle and angle_dot have been calculated by Kalman Filter in Angle_calculate()

  pwm = (angle * k1) + (angle_dot * k2) - (range * k3) - (wheel_speed * k4);    // use PID to calculate the pwm

  //  if(pwm > 255)      // Maximum Minimum Limitations
  //    pwm = 255;
  //    
  //  if(pwm < -255)
  //    pwm = -255;

  if(turn_flag == 0.0)
  {
    pwm_R = -pwm + direction_error_all;
    pwm_L = -pwm - direction_error_all;
  }
  else
  {
    // turn flag set:
    int turnFactor = (int)(turn_flag * 48.0);
    pwm_R = -pwm - turnFactor;  // Direction PID control
    pwm_L = -pwm + turnFactor;
    direction_error_all = 0;           // clear
  }

  Ldistance = 0;    // clear
  Rdistance = 0;
  turn_flag *= 0.8;
  forward_factor *= 0.8;
  //digitalWrite(10, LOW);
}

