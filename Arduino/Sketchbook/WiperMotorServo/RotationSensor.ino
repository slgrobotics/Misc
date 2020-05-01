
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
}

int readRotationSensor()
{ 
  return mlx_1.readAngle();
}
