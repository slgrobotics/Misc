#ifndef AlfsTechHbridge_h
#define AlfsTechHbridge_h

// class AlfsTechHbridge is respoonsible for controlling 50A "ALFS Tech" H-Bridge, available on eBay for around $45

#include <Arduino.h>
//#include <WProgram.h>

// this library is made intentionally similar to Pololu ZumoMotors library.

class AlfsTechHbridge
{
  public:

    // constructor
	AlfsTechHbridge(void);
	
	~AlfsTechHbridge(void);
	
	void init(void);	// must be called after constructor is called.
	
	// controls DIS pins of the H-Bridge, guaranteeing disabling the motors.
	void motorsDisable();
	void motorsEnable();

	void motorsStop();
	void stopLeft();
	void stopRight();
    
    // enable/disable flipping of motors
    void flipLeftMotor(boolean flip);
    void flipRightMotor(boolean flip);
    
    // set speed for left, right, or both motors
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    void setSpeeds(int leftSpeed, int rightSpeed);

	int ENCODER_A_B;	  	// side B (interrupt 0)
	int ENCODER_B_B;	  	// side B

	int ENCODER_A_A;		// side A (interrupt 1)
	int ENCODER_B_A;		// side A
	
  private:

	boolean flipLeft;
	boolean flipRight;
};

#endif