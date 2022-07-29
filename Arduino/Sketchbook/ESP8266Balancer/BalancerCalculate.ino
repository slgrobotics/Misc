// **************************
// Calculate the pwm (torque)
// **************************

void initPids()
{
  // best experimental values:
  k1 = 16.57;
  k2 = 2.45;
  k3 = 1.82;
  k4 = 0.79;
}

void pwm_calculate()
{
  // see where is average tilt by using ema:
  ema(TiltChannel);
  
  // distance traveled since last calculation:
  distance_per_cycle = (Ldistance + Rdistance) * 0.5;

  // accumulated distance from starting point, which we want to return to:
  range += distance_per_cycle;
  
  range *= 0.9;    // our reluctance to forget that starting point and accept current position
  
  if(abs(range) > 1000.0)
  {
    // something is wrong (wheels spinning?) - reset calculations
    range = 0.0;
    shortBuzz();
  }

  float range_error = range - distance_setpoint_remote;  // how far we are from setpoint

  int direction_error = Ldistance - Rdistance;  // turn error
  direction_error_all += direction_error;

  wheel_speed = distance_per_cycle;

  // see where is average speed by using ema:
  ema(SpeedChannel);

  float angle_error = angle - angle_setpoint_remote;

  pwm = (angle_error * k1) + (angle_dot * k2) - (range_error * k3) - (distance_per_cycle * k4);    // use PID to calculate the pwm

  if(turn_flag == 0.0)
  {
    // compensate for unexpected turns:
    pwm_R = -pwm + direction_error_all;
    pwm_L = -pwm - direction_error_all;
  }
  else
  {
    // we are in turn mode:
    int turnFactor = (int)(turn_flag * 68.0);
    pwm_R = -pwm - turnFactor;  // Direction PID control
    pwm_L = -pwm + turnFactor;
    direction_error_all = 0;           // clear
  }

  Ldistance = 0;    // clear encoders accumulators
  Rdistance = 0;
}

// variables to compute exponential moving average:
int emaPeriod[4];
float valuePrev[4];
float multiplier[4];

void ema(int ch)
{
  // smooth movement by using ema:
  switch (ch)
  {
    case TiltChannel:
      tiltEma = (valuePrev[ch] < -1000000.0) ? angle : ((angle - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      //tiltEma = angle; // no ema
      valuePrev[ch] = tiltEma;
      break;

    case SpeedChannel:
      speedEma = (valuePrev[ch] < -1000000.0) ? distance_per_cycle : ((distance_per_cycle - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      //speedEma = distance_per_cycle; // no ema
      valuePrev[ch] = speedEma;
      break;
  }
}

void setEmaPeriod(int ch, int period)
{
  valuePrev[ch] = -2000000.0;  // signal to use value directly for the first point
  emaPeriod[ch] = period;
  multiplier[ch] = 2.0 / (1.0 + (float) emaPeriod[ch]);
}
