#ifndef LM629PID_h
#define LM629PID_h

// LM 629 PID and motor operations

#include <LM629.h>

//  LM629 PID related commands and parameters
#define LM629PID_LOAD_KP_ONLY  			0x08
#define LM629PID_LOAD_KP_KD_KI 			0x0E
#define LM629PID_LOAD_KP_KD_KI_IL 		0x0F

// the following is based on Trajectory Control Word Bit Allocations:
//		Bit 5 - Acceleration Will Be Loaded
//		Bit 4 - Acceleration Data Is Relative
//		Bit 3 - Velocity Will Be Loaded
//		Bit 2 - Velocity Data Is Relative
//		Bit 1 - Position Will Be Loaded
//		Bit 0 - Position Data Is Relative

#define LM629PID_LOAD_ACCELERATION								0x20
#define LM629PID_LOAD_ACCELERATION_VELOCITY						0x28
#define LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_ABSOLUTE 	0x2A
#define LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_RELATIVE 	0x2B

#define LM629PID_LOAD_VELOCITY 									0x08
#define LM629PID_LOAD_VELOCITY_POSITION_ABSOLUTE 				0x0A
#define LM629PID_LOAD_VELOCITY_POSITION_RELATIVE 				0x0B

#define LM629PID_LOOP_PHASING 									0x00
#define LM629PID_STOP_MOTOR 									0x01
#define LM629PID_NO_TRAJECTORY_PARAMETERS_TO_LOAD 				0x00

class LM629PID
{
  public:

    // constructor
	LM629PID(LM629 *plm);
	
	~LM629PID(void);
	
	void init(void);
	
	void stopMotor(byte side);
	
	void loopPhasing01(byte side);
	
	void loadFilter(byte side, byte interval, short kp, short ki, short kd, short il);
	void loadFilter01(byte side);
	void loadFilter02(byte side);
	void loadFilter03(byte side);

	void loadTrajectory01(byte side);

	void simpleMove01(byte side);
	
	void simpleMove02(byte side);
	
	void madMove01(byte side);

	// all acceleration related methods are allowed only when not moving:
	void loadAcceleration(byte side, long acceleration);

	void moveAccelerationVelocity(byte side, long acceleration, long velocity);
	void moveAccelerationVelocityPositionAbsolute(byte side, long acceleration, long velocity, long position);
	void moveAccelerationVelocityPositionRelative(byte side, long acceleration, long velocity, long position);

	// these can be issued while moving:
	void moveVelocity(byte side, long velocity);
	void moveVelocityPositionAbsolute(byte side, long velocity, long position);
	void moveVelocityPositionRelative(byte side, long velocity, long position);

#ifdef LM629_DEBUG_PRINT
	void checkPoint(char *xComment);
#endif // LM629_DEBUG_PRINT

  private:
  
	LM629 *pLm629;

};

#endif