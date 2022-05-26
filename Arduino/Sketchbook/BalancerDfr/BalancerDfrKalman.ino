// **************************
// Calculate the Angle
// Takes acceleration, GyroY, dt
// Calls Kalman to calculate angle, angle_dot
// **************************
boolean Angle_calculate(void)
{
  boolean ret = true;
  
  //if(acceleration > 0.8 || acceleration < -0.8)
  //{
  //  return false;  // can't calculate angles around 90 degrees (robot fell down, or accelerated too much)
  //}
  
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
  
  double angleByAccel = asin(acceleration / 0.98) * 57.3;
  
  theta = angleByAccel - angle_setpoint_remote;
  
  double angle_dot_tmp = GyroY / 14.375;       // the angular velocity for this specific gyro
  
  Kalman_Filter(theta, angle_dot_tmp);         // calculates global variables: angle, angle_dot
  
  return ret;  // returns success flag
}

//*--------- Kalman filter implementation ------------------*/

// local persistent variables:
static const double C_0 = 1;

static const double Q_angle = 0.001,
                    Q_gyro  = 0.003,
                    R_angle = 0.5,
                    dt      = 0.005;  // see STD_LOOP_TIME
                    
double P[2][2] = {{ 1, 0 },
                  { 0, 1 }};
  
double Pdot[4] ={ 0, 0, 0, 0 };

double q_bias,
       angle_err,
       PCt_0,
       PCt_1,
       E,
       K_0,
       K_1,
       t_0, 
       t_1;


// **************************
//       Kalman Filter
// **************************

void Kalman_Filter(double angle_m, double gyro_m)   
{
  angle += (gyro_m - q_bias) * dt;    // global variable
  
  angle_err = angle_m - angle;
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
  
  angle += K_0 * angle_err;    // Optimal (estimated) angle, global variable
  angle_dot = gyro_m - q_bias; // Optimal (estimated) angular velocity, global variable
}


