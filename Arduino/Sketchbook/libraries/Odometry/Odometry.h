#ifndef Odometry_h
#define Odometry_h

#include <Arduino.h>

/// <summary>
/// returned values for Odometry calculations
/// </summary>
struct Displacement
{
  public:
	double dCenter;
	double halfPhi;
};

/// <summary>
/// calculates robot displacement based on current wheel encoders ticks,
/// memorizes previous values.
/// </summary>
class DifferentialDriveOdometry
{
  private:
	// we memorize previous calls to Odometry() to calculate wheels travel.
	long lastWheelEncoderLeftTicks;
	long lastWheelEncoderRightTicks;
	bool firstTime;  // flag to initialize "last" values

  public:
	// parameters:
	double wheelBaseMeters;
	double wheelRadiusMeters;
	double encoderTicksPerRevolution;
	
	// Process() arguments:
	long wheelEncoderLeftTicks;
	long wheelEncoderRightTicks;

	// Process() result:
	Displacement displacement;
	
	void Init(double _wheelBaseMeters, double _wheelRadiusMeters, double _encoderTicksPerRevolution);
	void Reset();
	void Process();	// result in displacement
};

#endif // Odometry_h
