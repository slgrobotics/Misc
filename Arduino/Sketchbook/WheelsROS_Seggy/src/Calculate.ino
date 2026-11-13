
// speed control is based on measuring ticks per cycle (say, 100ms), scaling to 100% and feeding it to PID which has a setting of 0...100 also in %
// for a drive configuration (particular robot) we need to measure ticks per cycle when on full power and wheels in the air.

void speed_calculate()
{
  distR = Rdistance - RdistancePrev; // Seggy robot: ~3 with 10Hz cycle, at full power and wheels in the air
  distL = Ldistance - LdistancePrev;

  // note: this is the place to scale -100..100% speed to 255 pwm:
  speedMeasured_R = (double)map(distR, -4, 4, -100, 100);  // first pair is number of ticks at full speed, we map it to 100%
  speedMeasured_L = (double)map(distL, -4, 4, -100, 100);

  // we don't need to keep track of total distances per wheel, only increments - for speed loop.
  // beware - if EncodersReset() is not called often enough, L/Rdistance will
  // overflow at 32K and cause violent jerking of the wheels!
  //EncodersReset();

  RdistancePrev = Rdistance;
  LdistancePrev = Ldistance;
}
