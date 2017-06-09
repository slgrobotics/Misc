// KasBot V1  -  PID module

#define   GUARD_GAIN   20.0
  
int last_error = 0;
int integrated_error = 0;
int pTerm = 0, iTerm = 0, dTerm = 0;

int updatePid(int targetPosition, int currentPosition, float K, int Kp, int Ki, int Kd)   {
  error = targetPosition - currentPosition; 
  pTerm = Kp * error;
  integrated_error += error;                                       
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);                            
  last_error = error;
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}
