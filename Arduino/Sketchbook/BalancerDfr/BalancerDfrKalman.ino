// ********************************************
// Calculate the Angle
// Takes acceleration, GyroY, dt
// Calls Kalman to calculate pitch, pitch_dot
// ********************************************
boolean calculatePitch(void)
{
  boolean ret = true;
  
  // can't calculate angles around 90 degrees (robot fell down, or accelerated too much):
  if(acceleration > 0.8)
  {
    acceleration = 0.8;
    ret = false;
  }
  if(acceleration < -0.8)
  {
    acceleration = -0.8;
    ret = false;
  }

  pitchByAccel = asin(acceleration / 0.98) * 57.3;  // degrees
  
  float pitch_dot_tmp = GyroY / 14.375;       // the angular velocity for this specific gyro
  
  Kalman_Filter(pitchByAccel + pitchAdjust, pitch_dot_tmp);         // calculates global variables: pitch, pitch_dot

  return ret;  // returns success flag
}

// --------- Kalman filter implementation ------------------

// local persistent variables:
static const float C_0 = 1;

static const float Q_angle = 0.001,
                    Q_gyro  = 0.003,
                    R_angle = 0.5,
                    dt      = FAST_LOOP_TIME / 1000.0;
                    
float P[2][2] = {{ 1, 0 },
                  { 0, 1 }};
  
float Pdot[4] ={ 0, 0, 0, 0 };

float q_bias = 0.0,
      angle_err = 0.0,
      PCt_0 = 0.0,
      PCt_1 = 0.0,
      E = 0.0,
      K_0 = 0.0,
      K_1 = 0.0,
      t_0 = 0.0, 
      t_1 = 0.0;


// ************************************************************
//       Kalman Filter
//
//  takes accelerometer angle and rotation from gyro
//  calculates "estimated" global variables: pitch, pitch_dot
//
// ************************************************************

void Kalman_Filter(float angle_m, float gyro_m)   
{
  pitch += (gyro_m - q_bias) * dt;    // global variable
  
  angle_err = angle_m - pitch;
  Pdot[0] = Q_angle - P[0][1] - P[1][0];
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  q_bias += K_1 * angle_err;
  
  pitch += K_0 * angle_err;    // Optimal (estimated) pitch, global variable
  pitch_dot = gyro_m - q_bias; // Optimal (estimated) angular velocity, global variable
}
