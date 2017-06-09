#ifndef LM629_h
#define LM629_h

// class LM629 is respoonsible for LM 629 bus operations

//#define LM629_DEBUG_PRINT  1

#include <Arduino.h>
//#include <WProgram.h>

//  LM629 electrical interface
#define LM629_OUTPUT_MODE 			OUTPUT
#define LM629_INPUT_MODE 			INPUT_PULLUP
#define LM629_FEATHER_RW 			INPUT
#define LM629_CHIP_SELECT_1 		LOW
#define LM629_CHIP_SELECT_2 		HIGH
#define LM629_PORT_SELECT_COMMAND 	LOW
#define LM629_PORT_SELECT_DATA 		HIGH
#define LM629_READ_ENABLE 			LOW
#define LM629_WRITE_ENABLE 			HIGH
#define LM629_RESET_ENABLE 			LOW 
#define LM629_RESET_DISABLE 		HIGH
#define LM629_HOST_INTERRUPT_ENABLE HIGH
#define LM629_HOST_INTERRUPT_DISABLE LOW

//  LM629 basic commands
#define LM629_COMMAND_RESET 		0x00
#define LM629_COMMAND_RSTI  		0x1D
#define LM629_COMMAND_RDSIGS 		0x0C		// RDSIGS COMMAND: READ SIGNALS REGISTER

//  LM629 advanced commands
#define LM629_COMMAND_LFIL			0x1E		// Load Filter Parameters;  2 to 10 bytes to write
#define LM629_COMMAND_UDF			0x04 		// Update Filter

#define LM629_COMMAND_LTRJ			0x1F 		// Load Trajectory; 2 to 14 bytes to write
#define LM629_COMMAND_STT			0x01 		// Start Motion

#define LM629_COMMAND_RDIP			0x09 		// Read Index Position 		4 bytes to read
#define LM629_COMMAND_RDDP			0x08 		// Read Desired Position 	4 bytes to read
#define LM629_COMMAND_RDRP			0x0A 		// Read Real Position 		4 bytes to read
#define LM629_COMMAND_RDDV			0x07 		// Read Desired Velocity 	4 bytes to read
#define LM629_COMMAND_RDRV			0x0B 		// Read Real Velocity 		2 bytes to read
#define LM629_COMMAND_RDSUM			0x0D 		// Read Integration Sum 	2 bytes to read


// commonly used bytes - fillers and masks
#define LM629_HB_ALL_ZERO  			0x00 
#define LM629_HB_DUMMY 				0x00
#define LM629_LB_ALL_INTERRUPT_BITS_RESET 0x00 	// 1 - 6 bits meaningful, 0 resets corresponding interrupt

class LM629
{
  public:

	boolean chip_1_selected;
	
    // constructor
	LM629(void);
	
	~LM629(void);
	
	void init(void);	// must be called after constructor is called. Will do hardware and software reset.
	
	byte readStatusByte(void);
	void writeCommandByte(byte xCommand);
	void writeTwoDataBytes(byte xHighByte, byte xLowByte);
	void writeShortData(short twoByteData);
	void writeLongData(long lWord);
	int readShortData(void);
	long readLongData(void);
	
	boolean softwareInterruptReset(void);
	boolean softwareSystemReset(void);
	
	void readSignalsRegister(void);
	long readRealPosition(void);
	long readDesiredPosition(void);
	int readRealVelocity(void);
	long readDesiredVelocity(void);
	int readIntegrationSum(void);
	boolean waitNotBusy(void);

  private:

	void setupLM629(void);	
	void hardwareSystemReset(void);
	//boolean resetLM629(void);
	
	void setDataPinMode(int xMode);
	byte readInputOutputPort(void);
	inline void delay120ns(void);
	inline void delay180ns(void);
	byte readDataByte(void);
	void writeByte(byte bt, boolean isCommandByte);
	void setDataPinsByte(byte xDataByte);
	void strobeWritePin(boolean isCommandByte);

	//
	// *** Declarations - Arduino UNO Atmel 328PU ***
	//
	// see constructor for pin assignments
	//
	// A0, A2 not used; A4 - SDA, A5 - SCL for I2C;  0 and 1 - reserved RX/TX
	// see http://www.arduino.cc/en/Reference/PortManipulation for ports to pins allocation. RX,TX and crystal make whole-register operations impossible.
	//

	int chipSelectPin;
	int portSelectPin;
	int readWritePin;
	int interruptPin;
	int resetPin;
	int dataPin0;
	int dataPin1;
	int dataPin2;
	int dataPin3;
	int dataPin4;
	int dataPin5;
	int dataPin6;
	int dataPin7;

	int dataPin[8];
};

#endif