/**
 * Two Channel Receiver
 * Author: Shawn Hymel (SparkFun Electronics)
 * Date: Aug 24, 2017
 * 
 * https://gist.github.com/ShawnHymel/ccc28335978d5d5b2ce70a2a9f6935f4
 * https://www.youtube.com/watch?v=Bx0y1qyLHQ4
 * 
 * modified to print original and mixed values. removed motor code - Sergei Grichine 2020 
 * Transmitter: FlySky FS-I6X 10ch. Receiver: FS-iA10B channels 1 (ailerons) and 2 (elevator)
 * all beer goes to original authors.
 * 
 * Mixes two channels for arcade drive.
 * 
 * This code is beerware; if you see me (or any other SparkFun 
 * employee) at the local, and you've found our code helpful, 
 * please buy us a round! 
 * Distributed as-is; no warranty is given.
 */

// Receiver pins
const int CH_1_PIN = 6;
const int CH_2_PIN = 2;

// for motor-like PWM output, -255...255:
const int OUT_MAX = 1000; //255;
const int OUT_SHIFT = 0;

// for RC-like 1000...2000 us output:
//const int OUT_MAX = 500;
//const int OUT_SHIFT = 1500;

// for Servo library object output. Allows 0...180 degrees, we limit it to 45...135 
// so that we match standard receiver 1000...2000us range:
const int OUT_MAX = 45;
const int OUT_SHIFT = 90;

void setup() {
  Serial.begin(115200);
  delay(100);
}

void loop() {

  double leftMix;
  double rightMix;

  // Read pulse width from receiver
  // Read pulse width from receiver. We spend 39 milliseconds here, which amounts to 25Hz cycle:
  unsigned long yp = pulseIn(CH_2_PIN, HIGH); //, 25000); // doesnt work with timeout
  unsigned long xp = pulseIn(CH_1_PIN, HIGH); //, 25000);

  // from here down it takes 0.5 ms on Arduino Uno to calculate all:

  // assuming pulse range 1000...2000us with neutral at 1500us
  yp = constrain(yp, 1000, 2000);
  xp = constrain(xp, 1000, 2000);

  // Convert to normalized "motor" value (-1.0 to +1.0):
  double x = pulseToMotor(xp);
  double y = pulseToMotor(yp);

  // we need absolute values later:
  double ax = abs(x);
  double ay = abs(y);

  // transition zone flag. This is two sectors between 0 and -45 degrees:
  boolean tz = (y < 0 && ay < ax) ? true : false;

  // we scale output in diagonal directions:
  double alpha = (ax == 0 || ay == 0) ? 0 : atan(ay < ax ? ay/ax : ax/ay);
  double factor = tz ? cos(alpha/2.0) : cos(alpha);

  // Mix for arcade drive
  if (y >= 0.0)
  {
    // normal zone, forward movement. Upper sector -90...+90 degrees:
    leftMix = y + x;
    rightMix = y - x;
  }
  else if(!tz)
  {
    // backwards driving zone, inverted output. Lower sector, 135...215 degrees:
    leftMix = y - x;
    rightMix = y + x;
  }
  else if (x > 0)
  {
    // right transition zone, 90...135 degrees:
    leftMix = 2.0 * y + x;
    rightMix = -(y + x);
  }
  else
  {
    // left transition zone, 215...270 (a.k.a. -90) degrees:
    leftMix = -y + x;
    rightMix = 2.0 * y - x;
  }

  factor = factor * factor * 1.1;   // experimental

  // scale it to desired output, convert to integer:
  int leftMixI = (int)(leftMix * factor * (double)OUT_MAX);
  int rightMixI = (int)(rightMix * factor * (double)OUT_MAX);

  leftMixI = constrain(leftMixI, -OUT_MAX, OUT_MAX) + OUT_SHIFT;
  rightMixI = constrain(rightMixI, -OUT_MAX, OUT_MAX) + OUT_SHIFT;

  // print all
  Serial.print("tz: ");
  Serial.print(tz ? "true" : "false");
  Serial.print("  x: ");
  Serial.print(xp);
  Serial.print("   y: ");
  Serial.print(yp);
  
  Serial.print("  |  ");
  Serial.print(x);
  Serial.print("   ");
  Serial.print(y);
  Serial.print("   (a: ");
  Serial.print(alpha);
  Serial.print(" f: ");
  Serial.print(factor);
  Serial.print(")  ==>  L: ");
  Serial.print(leftMix);
  Serial.print("   R: ");
  Serial.print(rightMix);
  Serial.print("  ====>  L: ");
  Serial.print(leftMixI);
  Serial.print("   R: ");
  Serial.print(rightMixI);

  Serial.println();
  Serial.println();

  delay(100);
}

double pulseToMotor(unsigned long pulseWidthUs)
{
  double ret = (((double)pulseWidthUs) - 1500.0) / 500.0; // stick deflection normalized to -1...+1
  return ret;
}
