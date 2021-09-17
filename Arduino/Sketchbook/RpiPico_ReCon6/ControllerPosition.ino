
// given a goal (waypoint or direction) compute steering and speed

void controlPosition()
{
  // start with default behavior: "stop"
  desiredSpeedR = 0.0;
  desiredSpeedL = 0.0;
  steeringControlEffort = 0;

  if (pixyBlocksCount > 0)
  {
    // we mostly are interested in width, but filter by height too:
    bool closeEnough = pixyBlock.width > 60 && pixyBlock.height > 20;  // red cone a bit vertical. 2 ft distance 100x130
    bool tooClose = pixyBlock.width > 80 && pixyBlock.height > 20;

    if(!closeEnough)
    {
      // move towards the object:
      desiredSpeedR = 50.0;
      desiredSpeedL = 50.0;
      steeringControlEffort = -pixyBlock.x * 0.5;
    } else if(tooClose) {
      // back up slowly, keeping object in view:
      desiredSpeedR = -30.0;
      desiredSpeedL = -30.0;
      steeringControlEffort = -pixyBlock.x * 0.5;
    }
    
  } else {
    // no object in view, remain stopped and wait.
  }


#ifdef USE_EMA
  // smooth movement by using ema: take desiredSpeed and produce setpointSpeed
  ema(RightMotorChannel);
  ema(LeftMotorChannel);
#else // USE_EMA
  setpointSpeedR = desiredSpeedR; // no ema
  setpointSpeedL = desiredSpeedL;
#endif // USE_EMA

  // test - controller should hold this speed:
  //desiredSpeedR = 10;
  //desiredSpeedL = 10;

  controlSteering();

  controlSpeed();

}

void controllersInit()
{
  int PID_SAMPLE_TIME = mydt * pidLoopFactor;  // milliseconds.

  // turn the PID on and set its parameters:
  myPID_R.SetOutputLimits(-250.0, 250.0);  // match to maximum PID outputs in both directions. PID output will be added to PWM on each cycle.
  myPID_R.SetSampleTime(PID_SAMPLE_TIME);  // milliseconds. Regardless of how frequently Compute() is called, the PID algorithm will be evaluated at a regular interval (no more often than this).
  myPID_R.SetMode(AUTOMATIC);              // AUTOMATIC means the calculations take place, while MANUAL just turns off the PID and lets the man drive

  myPID_L.SetOutputLimits(-250.0, 250.0);
  myPID_L.SetSampleTime(PID_SAMPLE_TIME);
  myPID_L.SetMode(AUTOMATIC);

  k1 = 0;
  k2 = 0;
  k3 = 0;
  k4 = 0;
}
