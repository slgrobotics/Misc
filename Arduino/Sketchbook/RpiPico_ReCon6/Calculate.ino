
// Generic calculations here

// ------------------------------------------------------------------------------------------------------

void odometryReset()
{
  oLdistance = 0;
  oRdistance = 0;

  X = Y = Theta = 0;

  odometry->Reset();
}

void odometryProcess()
{
  // odometry calculation takes 28us
  // process encoders readings into X, Y, Theta using odometry library:
  odometry->wheelEncoderLeftTicks = oLdistance;
  odometry->wheelEncoderRightTicks = oRdistance;

  odometry->Process();

  if (odometry->displacement.halfPhi != 0.0 || odometry->displacement.dCenter != 0.0)
  {
    double theta = Theta + odometry->displacement.halfPhi;   // radians

    // calculate displacement in the middle of the turn:
    double dX = odometry->displacement.dCenter * cos(theta);      // meters
    double dY = odometry->displacement.dCenter * sin(theta);      // meters

    X += dX;
    Y += dY;

    Theta += odometry->displacement.halfPhi * 2.0;
  }
}

// ------------------------------------------------------------------------------------------------------

// variables to compute exponential moving average:
int emaPeriod[2];
double valuePrev[2];
double multiplier[2];

void ema(int ch)
{
  // smooth movement by using ema:
  switch (ch)
  {
    case RightMotorChannel:
      setpointSpeedR = (valuePrev[ch] < -1000000.0) ? desiredSpeedR : ((desiredSpeedR - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      //setpointSpeedR = desiredSpeedR; // no ema
      valuePrev[ch] = setpointSpeedR;
      break;

    case LeftMotorChannel:
      setpointSpeedL = (valuePrev[ch] < -1000000.0) ? desiredSpeedL : ((desiredSpeedL - valuePrev[ch]) * multiplier[ch] + valuePrev[ch]);  // ema
      //setpointSpeedL = desiredSpeedL; // no ema
      valuePrev[ch] = setpointSpeedL;
      break;
  }
}

void resetEma(int ch)
{
  valuePrev[ch] = -2000000.0;  // signal to use desiredSpeed directly for the first point
}

void setEmaPeriod(int ch, int period)
{
  resetEma(ch);
  emaPeriod[ch] = period;
  multiplier[ch] = 2.0 / (1.0 + (double)emaPeriod[ch]);
}
