
#include <Odometry.h>


volatile int oLdistance, oRdistance; // encoders - distance traveled, used by odometry calculator

// robot parameters:
double wheelBaseMeters = 0.570;
double wheelRadiusMeters = 0.1805;
double encoderTicksPerRevolution = 1460;  // one wheel rotation

// current robot pose, updated by odometry:
double X;      // meters
double Y;      // meters
double Theta;  // radians, positive clockwise

DifferentialDriveOdometry *odometry;

int cnt = 0;

void setup()
{
  Serial.begin(115200);   // start serial for USB

  // DEBUG pins:
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  //EncodersInit();	// Initialize the encoders

  odometry = new DifferentialDriveOdometry();
  odometry->Init(wheelBaseMeters, wheelRadiusMeters, encoderTicksPerRevolution);
}

void loop()
{
  digitalWrite(10, HIGH);
  delay(1);
  digitalWrite(10, LOW);

  // imitate encoders for left and right side:
  if(cnt % 2 == 0)
    oLdistance++;

  if(cnt % 3 == 0)
    oRdistance++;


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

  if(cnt % 100 == 0)
  {
    printAll();
  }

  delay(10);
  cnt++;
}

void printAll()
{
    Serial.print("Encoders:  R: ");
    Serial.print(oRdistance);
    Serial.print("  L: ");
    Serial.print(oLdistance);

    Serial.print("     Odometry: X=");
    Serial.print(X);
    Serial.print("  Y=");
    Serial.print(Y);
    Serial.print(" meters    Theta=");
    Serial.print(Theta);
    Serial.print("  (");
    Serial.print(to360(Theta * 57.295));
    Serial.println(" degrees)");
}

// limits degrees "a" to 0...360
double to360(double a)
{
  if(a > 0.0)
  {
    while (a >= 360.0)
    {
      a -= 360.0;
    }
  }
  else if(a < 0.0)
  {
    while (a < 0.0)
    {
      a += 360.0;
    }
  }
  
  return a;
}


