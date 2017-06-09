	
	
	#include <Odometry.h>
	
	/// <summary>
    /// calculates robot displacement based on current wheel encoders ticks,
    /// memorizes previous values.
    /// </summary>
	void DifferentialDriveOdometry::Init(double _wheelBaseMeters, double _wheelRadiusMeters, double _encoderTicksPerRevolution)
	{
		wheelBaseMeters = _wheelBaseMeters;
		wheelRadiusMeters = _wheelRadiusMeters;
		encoderTicksPerRevolution = _encoderTicksPerRevolution;
		
		lastWheelEncoderLeftTicks = 0L;
		lastWheelEncoderRightTicks = 0L;
		firstTime = true;
	}

	/// <summary>
	/// resets odometry algorithm in case of invalid wheel ticks changes
	/// </summary>
	void DifferentialDriveOdometry::Reset()
	{
		lastWheelEncoderLeftTicks = 0L;
		lastWheelEncoderRightTicks = 0L;
		firstTime = true;
	}

	/// <summary>
	/// calculates robot displacement based on current wheel encoders ticks
	/// </summary>
	/// <param name="robotPose">will be adjusted based on wheels travel</param>
	/// <param name="encoderTicks">wheel encoder ticks - left, right...</param>
	void DifferentialDriveOdometry::Process()
	{
		displacement.dCenter = 0.0;
		displacement.halfPhi = 0.0;

		if (firstTime)
		{
			lastWheelEncoderLeftTicks = wheelEncoderLeftTicks;
			lastWheelEncoderRightTicks = wheelEncoderRightTicks;
			firstTime = false;
		}
		else
		{
			long dLticks = wheelEncoderLeftTicks - lastWheelEncoderLeftTicks;
			long dRticks = wheelEncoderRightTicks - lastWheelEncoderRightTicks;

			if (dLticks != 0L || dRticks != 0L)
			{
				double metersPerTick = 2.0 * PI * wheelRadiusMeters / encoderTicksPerRevolution;    // meters per tick

				double distanceLeftMeters = dLticks * metersPerTick;
				double distanceRightMeters = dRticks * metersPerTick;

				// Now, calculate the final angle, and use that to estimate
				// the final position.  See Gary Lucas' paper for derivations
				// of the equations.

				/*
				 * Coursera formulas for reference:
				 * 
					d_right = (right_ticks - prev_right_ticks) * m_per_tick;
					d_left = (left_ticks - prev_left_ticks) * m_per_tick;
		
					d_center = (d_right + d_left)/2;
					phi = (d_right - d_left)/L;
		
					x_dt = d_center*cos(theta);
					y_dt = d_center*sin(theta);
					theta_dt = phi;
					
					theta_new = theta + theta_dt;
					x_new = x + x_dt;
					y_new = y + y_dt;                           
				 */

				displacement.halfPhi = (distanceLeftMeters - distanceRightMeters) / (wheelBaseMeters * 2.0);   // radians, assuming really small value for phi
				displacement.dCenter = (distanceRightMeters + distanceLeftMeters) / 2.0;

				lastWheelEncoderLeftTicks = wheelEncoderLeftTicks;
				lastWheelEncoderRightTicks = wheelEncoderRightTicks;
			}
		}
	}
