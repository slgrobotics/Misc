

void controlSpeed()
{
  bool isDirectLaw = true;  // desired speed is motor PWM, no PID

  if(isDirectLaw)
  {
    // no PID or EMA, just direct power inputs:
    pwm_R = desiredSpeedR * 255.0 / 100.0;
    pwm_L = desiredSpeedL * 255.0 / 100.0;

    // do not allow float values to grow beyond motor maximums:
    pwm_R = constrain(pwm_R, -255.0, 255.0);     // Maximum / Minimum Limitations
    pwm_L = constrain(pwm_L, -255.0, 255.0);
  }
  else
  {
    // based on encoder distance increments, measure speed:
    speed_measure();
    
    // With PIDs - having desires speeds, compute control inputs (i.e. current PWM)
    myPID_R.Compute();
    myPID_L.Compute();

    // Calculate the pwm - pick up values computed by PIDs:
    pwm_calculate();
  }
}

long distR;
long distL;

// speed control is based on measuring ticks per cycle (say, 100ms), scaling to 100% and feeding it to PID which has a setting of 0...100 also in %
// for a drive configuration (particular robot) we need to measure ticks per cycle when on full power and wheels in the air.

void speed_measure()
{
  distR = Rdistance;   // around 570 with 20Hz cycle, full power and wheels in the air
  //Rdistance = 0;     // same in EncodersReset()
  distL = Ldistance;
  //Ldistance = 0;
  
  // we don't need to keep track of total distances per wheel, only increments - for speed loop.
  // beware - if EncodersReset() is not called often enough, L/Rdistance will
  // overflow at 32K and cause violent jerking of the wheels!
  //EncodersReset();

  // note: this is the place to scale -100..100% speed to 255 pwm

  speedMeasured_R = (double)map(distR, -570, 570, -100, 100);  // first pair is number of ticks at full speed, we map it to 100%
  speedMeasured_L = (double)map(distL, -570, 570, -100, 100);
}

// **********************************************
// Calculate the pwm, given the desired speed
// **********************************************
void pwm_calculate()
{
  // pick up values computed by PIDs. We must do it in float:
  pwm_R += dpwm_R;
  pwm_L += dpwm_L;

  // do not allow float values to grow beyond motor maximums:
  pwm_R = constrain(pwm_R, -255.0, 255.0);     // Maximum / Minimum Limitations
  pwm_L = constrain(pwm_L, -255.0, 255.0);

  // we also let set_motor() constrain local integer ipwm_L and ipwm_R to -255...255
}
