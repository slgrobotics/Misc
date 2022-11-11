
// see http://interface.khm.de/index.php/lab/interfaces-advanced/rotary-positionsensor-mlx90316/index.html
// library at http://interface.khm.de/index.php/lab/experiments/rotary-positionsensor-mlx90316/attachment/mlx90316/index.html

#include <MLX90316.h>

const int pinSS = 5;
const int pinSCK = 3;
const int pinMOSI = 4;

MLX90316 mlx_1 = MLX90316();

void setupRotationSensor()
{
  mlx_1.attach(pinSS, pinSCK, pinMOSI);
  Serial.println("MLX90316 Rotary Position Sensor");
  rs_setEmaPeriod(3);
}

double readRotationSensor()
{ 
  return rs_ema((double)mlx_1.readAngle());
}

// variables to compute exponential moving average for rotation sensor:
int rs_emaPeriod;
double rs_valuePrev;
double rs_multiplier;

double rs_ema(double curVal)
{
  // smooth movement by using ema:
  double emaVal = (rs_valuePrev < -1000000.0) ? curVal : ((curVal - rs_valuePrev) * rs_multiplier + rs_valuePrev);
  //emaVal = curVal; // no ema
  rs_valuePrev = emaVal;
  return emaVal;
}

void rs_setEmaPeriod(int period)
{
  rs_valuePrev = -2000000.0;  // signal to use desiredSpeed directly for the first point
  rs_emaPeriod = period;
  rs_multiplier = 2.0 / (1.0 + (double)rs_emaPeriod);
}
