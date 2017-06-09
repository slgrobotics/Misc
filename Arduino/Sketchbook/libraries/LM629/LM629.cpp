#ifndef F_CPU
#define F_CPU 16000000UL  // Standard Arduinos run at 16 MHz
#endif //!F_CPU

#include "LM629.h"

// Constructor
LM629::LM629(void)
{
// *** Declarations - Arduino UNO Atmel 328PU ***
// Arduino Data Pin           LM629 pin     Atmel 328PU pin 
	chipSelectPin = 3;   // CS LM629 pin 12    // 328PU pin 5
	portSelectPin = 4;   // PS LM629 pin 16    // 328PU pin 6
	//                       		// RD LM629 pin 13    // multiplexed to pin A3
	readWritePin = A3;   // WR LM629 pin 15    // 328PU pin 26
	interruptPin = 2;    // HI LM629 pin 17    // 328PU pin 4      // connected for now
	resetPin = A1;       // RS LM629 pin 27    // 328PU pin 24
	dataPin0 = 12;       // D0 LM629 pin 11    // 328PU pin 18
	dataPin1 = 11;       // D1 LM629 pin 10    // 328PU pin 17
	dataPin2 = 10;       // D2 LM629 pin  9    // 328PU pin 16
	dataPin3 = 9;        // D3 LM629 pin  8    // 328PU pin 15
	dataPin4 = 8;        // D4 LM629 pin  7    // 328PU pin 14
	dataPin5 = 7;        // D5 LM629 pin  6    // 328PU pin 13
	dataPin6 = 6;        // D6 LM629 pin  5    // 328PU pin 12
	dataPin7 = 5;        // D7 LM629 pin  4    // 328PU pin 11

	// we use array dataPin for convenience - switching the data bus between INPUT_PULLUP and OUTPUT mode 
	dataPin[0] = dataPin0;
	dataPin[1] = dataPin1;
	dataPin[2] = dataPin2;
	dataPin[3] = dataPin3; 
	dataPin[4] = dataPin4;
	dataPin[5] = dataPin5;
	dataPin[6] = dataPin6;
	dataPin[7] = dataPin7;

	// this will be set before every bus operation:
	chip_1_selected = false;
}

// Destructor
LM629::~LM629(void)
{
}

void LM629::init(void)
{
	setupLM629();
	
	boolean resetOk1;
	boolean resetOk2;

	do
	{
		hardwareSystemReset();	// resets both chips
		
		chip_1_selected = true;
		resetOk1 = softwareInterruptReset();
		
		chip_1_selected = false;
		resetOk2 = softwareInterruptReset();

	} while (!resetOk1 || !resetOk2);
}

// ======================== region LM629 bus operations =====================================
  
void LM629::setupLM629(void)
  {
    pinMode(interruptPin, INPUT_PULLUP);  
    pinMode(readWritePin, LM629_FEATHER_RW);  // feather the read and write strobes (to HIGH)

    digitalWrite(resetPin, LM629_RESET_DISABLE); 
    pinMode(resetPin, OUTPUT);

    digitalWrite(chipSelectPin, LM629_CHIP_SELECT_1); 
    digitalWrite(portSelectPin, LM629_PORT_SELECT_DATA);	// HIGH
    
    setDataPinMode(LM629_INPUT_MODE);

    pinMode(chipSelectPin, OUTPUT); 
    pinMode(portSelectPin, OUTPUT);
  }  

/// reset pin is common for both chips and does not depend on CS. Calling this function resets both chips.
void LM629::hardwareSystemReset(void)
  {
	// pulse reset for 10us, check status byte 0, complete time 1.5ms (3ms to be sure)

	byte statusByte1;
	byte statusByte2;
	byte statusByte1a;
	byte statusByte2a;

	int cnt = 0;
	
	do
	{
		pinMode(readWritePin, LM629_FEATHER_RW);  	// feather the read and write strobes (to HIGH)
		digitalWrite(portSelectPin, HIGH); 
		
		// Strobe RST, pin 27, logic low for eight clock periods minimum (8 * 125ns = 1us)
		digitalWrite(resetPin, LM629_RESET_ENABLE);
		delayMicroseconds(10); 
		
		digitalWrite(resetPin, LM629_RESET_DISABLE);
		
		// Immediately after releasing the reset pin from the LM628, the status port should read '00'.
		chip_1_selected = true;
		statusByte1 = readStatusByte();
		chip_1_selected = false;
		statusByte2 = readStatusByte();

		delay(3);  // allow some time for reset - 1.5ms (3ms to be sure)

		// expect status byte to be C4 hex or 84 hex
		chip_1_selected = true;
		statusByte1a = readStatusByte();
		chip_1_selected = false;
		statusByte2a = readStatusByte();

#ifdef LM629_DEBUG_PRINT
		Serial.print("After hardware system reset, Chip 1: "); 
		Serial.print(statusByte1, HEX);
		Serial.print(" -> "); 
		Serial.print(statusByte1a, HEX);
		Serial.print("    Chip 2: "); 
		Serial.print(statusByte2, HEX);
		Serial.print(" -> "); 
		Serial.print(statusByte2a, HEX);
		Serial.print("   CNT: "); 
		Serial.println(cnt);
#endif // LM629_DEBUG_PRINT
		
		delay(2); 
		cnt++;
	} 
	while( (cnt < 20 && (statusByte1 != 0 || statusByte2 != 0)) || (statusByte1a != 0x84 && statusByte1a != 0xC4) || (statusByte2a != 0x84 && statusByte2a != 0xC4));
	// after 20 cycles we can suspect that we don't read initial zeroes fast enough from both chips. Ignore that condition and judge by the 84/C4 after reset

}

boolean LM629::softwareSystemReset(void)
  {
    writeCommandByte(LM629_COMMAND_RESET);
    delay(3);

	byte statusByte = readStatusByte();

#ifdef LM629_DEBUG_PRINT
    Serial.print(chip_1_selected ? "Chip 1  " : "Chip 2  ");
	Serial.print("After software RESET command   Status Byte: "); Serial.println(statusByte, HEX);  
#endif // LM629_DEBUG_PRINT

	return statusByte == 0xC4 || statusByte == 0x84;
  }  

boolean LM629::softwareInterruptReset(void)
  {
    writeCommandByte(LM629_COMMAND_RSTI);
    writeShortData(LM629_LB_ALL_INTERRUPT_BITS_RESET); 	// 1 - 6 bits, 0 resets corresponding interrupt   

	waitNotBusy();
	byte statusByte = readStatusByte();

#ifdef LM629_DEBUG_PRINT
    Serial.print(chip_1_selected ? "Chip 1  " : "Chip 2  ");
	Serial.print("After software interrupt reset (RSTI command)   Status Byte: "); Serial.println(statusByte, HEX);  
#endif // LM629_DEBUG_PRINT

	return statusByte == 0xC0 || statusByte == 0x80;
  }

void LM629::setDataPinMode(int xMode)
  {
    for (int i = 0; i < 8; i++)  
    {
      pinMode(dataPin[i], xMode);             
    }
  }

inline void LM629::delay120ns(void)
  {
    // at 16 MHZ Atmel328PU each assembly "nop" is 62.5 nanoseconds.
    // some delay is associated also with calling this function (unless "inline"), so total delay may be more.
    asm("nop");
    asm("nop");
  }

inline void LM629::delay180ns(void)
  {
    // at 16 MHZ Atmel328PU each assembly "nop" is 62.5 nanoseconds.
    // some delay is associated also with calling this function (unless "inline"), so total delay may be more.
    asm("nop");
    asm("nop");
    asm("nop");
  }

// 	waits 120ns and then reads a data byte. Waits 120ns after reading as well.
byte LM629::readInputOutputPort(void)
  {
    //delay120ns();    // 120ns wait needed in case there was a read or write before

	pinMode(readWritePin, LM629_FEATHER_RW);  // feather the read and write strobes (to HIGH) - just in case
	digitalWrite(chipSelectPin, chip_1_selected ? LM629_CHIP_SELECT_1 : LM629_CHIP_SELECT_2); 
	digitalWrite(portSelectPin, LM629_PORT_SELECT_DATA); 
	setDataPinMode(LM629_INPUT_MODE);	// also serves as a small delay, only 30ns needed
    
    // RD strobe:
    digitalWrite(readWritePin, LM629_READ_ENABLE); 
    pinMode(readWritePin, OUTPUT); 		// read pin LOW
    delay180ns();    // 180ns wait needed for LM629 to set data on the bus
  
    byte ret = 0;
	int i = 0;

	// read 8 bits into ret:
    while (true)  
    {
      if (digitalRead(dataPin[i]))
        ret = ret | 0x80;
      if (i++ == 7)
		break;
      ret = ret >> 1;
    }

    pinMode(readWritePin, LM629_FEATHER_RW); // release RD strobe
	
    delay120ns();    // 120ns min wait needed for LM629 to recover after read
    return ret;
  }

byte LM629::readStatusByte(void)
  { 
	// same code as in readInputOutputPort() - less waiting, leaned for speed, LOW on PS
	digitalWrite(chipSelectPin, chip_1_selected ? LM629_CHIP_SELECT_1 : LM629_CHIP_SELECT_2); 
	digitalWrite(portSelectPin, LOW); 
	setDataPinMode(LM629_INPUT_MODE);	// also serves as a small delay, only 30ns needed
    
    // RD strobe:
    digitalWrite(readWritePin, LM629_READ_ENABLE); 
    pinMode(readWritePin, OUTPUT); 		// read pin LOW
    delay180ns();    // 180ns wait needed for LM629 to set data on the bus
  
    byte ret = 0;
	int i = 0;

	// read 8 bits into ret:
    while (true)  
    {
      if (digitalRead(dataPin[i]))
        ret = ret | 0x80;
      if (i++ == 7)
		break;
      ret = ret >> 1;
    }

    pinMode(readWritePin, LM629_FEATHER_RW); // release RD strobe
	
    // delay120ns();    // 120ns min wait needed for LM629 to recover after read
    return ret;
  }

// read the status byte bit 0 - check the state of a flag called the “busy bit” (Bit 0).
// If the busy bit is logic high, no command write may take place.
// The busy bit is never high longer than 100 µs, and typically falls within 15 µs to 25 µs.
// returns false on error (i.e. busy bit not reset within 150us)
boolean LM629::waitNotBusy(void)
  {
    pinMode(readWritePin, LM629_FEATHER_RW); 		// feather the read and write strobes (to HIGH) - just in case
    digitalWrite(chipSelectPin, chip_1_selected ? LM629_CHIP_SELECT_1 : LM629_CHIP_SELECT_2); 
    digitalWrite(portSelectPin, LOW); 	// must be LOW to read status bit/byte
  
    setDataPinMode(LM629_INPUT_MODE);
    
    byte busyBit;
    int nTries = 0;
    while(true)
    {
      // RD strobe:
      digitalWrite(readWritePin, LM629_READ_ENABLE); 
      pinMode(readWritePin, OUTPUT);   	// set RD strobe to LOW
      delay180ns();						// let the chip set data on the bus
      busyBit = digitalRead(dataPin0);
      pinMode(readWritePin, LM629_FEATHER_RW); 	// release RD strobe - RD and WR HIGH
      if(busyBit == 0 || nTries++ > 150)
      {
		delay180ns();	// let the chip recover from read
        break;  		// either not busy (normal), or waited for 150 us (error)
      }
      delayMicroseconds(1);  // we expect the busy bit to stay for about 15-25 microseconds... 150us means trouble.
    }
    
#ifdef LM629_DEBUG_PRINT
    if(busyBit != 0)
    {
      Serial.println("Error: waitNotBusy() - still busy after 150us wait");
    }
#endif // LM629_DEBUG_PRINT

	return busyBit == 0;
  }

// set data pins to output the byte, but do not strobe the write pin yet
void LM629::setDataPinsByte(byte dataByte)
  {
    pinMode(readWritePin, LM629_FEATHER_RW); 	// feather the read and write strobes (to HIGH) - just in case
    
    setDataPinMode(LM629_OUTPUT_MODE);
  
    for (int i = 0; i < 8; i++)  
    {   
	  digitalWrite(dataPin[i], ((dataByte >> i) & 0x01) ? HIGH : LOW);
    }
  }

/// just write a byte - no busy bit checking or waiting before or after.
void LM629::writeByte(byte bt, boolean isCommandByte)
  {
    setDataPinsByte(bt);    
    strobeWritePin(isCommandByte);
  }  
  
// set portSelectPin depending on isCommandByte and strobe the write pin for 180 nanoseconds.
// the data is latched into LM629 when the strobe goes HIGH. No need to hold it longer.
// if multiple bytes are written, a 120ns wait must preceed next WR strobe (going LOW)
void LM629::strobeWritePin(boolean isCommandByte)
  {
    digitalWrite(chipSelectPin, chip_1_selected ? LM629_CHIP_SELECT_1 : LM629_CHIP_SELECT_2); 
  
    if(isCommandByte)
    {
      digitalWrite(portSelectPin, LM629_PORT_SELECT_COMMAND); 
    }
    else
    {
      digitalWrite(portSelectPin, LM629_PORT_SELECT_DATA); 
    }
    
    // strobe the WR:
    digitalWrite(readWritePin, LM629_WRITE_ENABLE);
    pinMode(readWritePin, OUTPUT); // write strobe LOW
    delay120ns();	 // **********  100ns min to have LM629 prepare for accepting byte
    pinMode(readWritePin, LM629_FEATHER_RW);  // read and write HIGH, the byte is latched in LM629 buffer. Write done.
  }
  
// ======================== endregion LM629 bus operations =====================================

byte LM629::readDataByte(void)
  { 
    byte dataByte = readInputOutputPort();	// has a 120ns wait before, no WaitNotBusy
    return dataByte;      
  }

// wait for busy bit and write the byte as command
void LM629::writeCommandByte(byte xCommand)
  {
    waitNotBusy();

	writeByte(xCommand, true);
	
    delay120ns(); // **********  120ns min before first (high) byte write
	
	// Note: Busy bit will be set here.
  }

// Data writes and reads are always an integral number (from one to seven) of two-byte words,
// with the first byte of each word being the more significant. Each byte requires a write (WR ) or read (RD ) strobe.
// When transferring data words (byte-pairs), it is necessary to first read the status byte and check the state
// of the busy bit. When the busy bit is logic low, the user may then sequentially transfer both bytes comprising a
// data word, but the busy bit must again be checked and found to be low before attempting to transfer the next
// byte pair (when transferring multiple words).  
  
// just wait 120ns and write the two bytes as data. Called always after a command byte write.
void LM629::writeTwoDataBytes(byte xHighByte, byte xLowByte)
  {
    waitNotBusy();

    writeByte(xHighByte, false);
  
    delay120ns(); // **********  120ns min before second (low) byte write

    writeByte(xLowByte, false);
	
	// Note: Busy bit will be set here.
  }

// just wait 120ns and write the short as two bytes (as data). Called always after a command byte write.
void LM629::writeShortData(short twoByteData)
  {
    waitNotBusy();

    writeByte((byte)(twoByteData >> 8), false);
  
    delay120ns(); // **********  120ns min before second (low) byte write

    writeByte((byte)twoByteData, false);
	
	// Note: Busy bit will be set here.
  }

// wait for Not Busy and write the four bytes as data (waiting for NotBusy before the 3d byte)
void LM629::writeLongData(long longWord)
  {
    byte bytes[4];
    
    for(int i=0; i < 4 ;i++)
    {
      byte bt = (byte)longWord;
      bytes[3 - i] = bt;
      longWord = longWord >> 8;
    }
	// no need to wait 120ns here, the loop above did that

	// write two pairs, checking for Busy bit twice:
    for(int i=0; i < 4 ;i++)
    {
		if(i==0 || i == 2)
		{
			waitNotBusy();
		}
		else
		{
			delay120ns(); // **********  120ns min before byte write
		}
		writeByte(bytes[i], false);		// false means data
    }  
  }

 /// waits for Not Busy, reads four bytes and packs them into a long word
long LM629::readLongData(void)
{
	long ret = 0L;
	
    for(int i=0; i < 4 ;i++)
    {
		if(i==0 || i == 2)
		{
			waitNotBusy();
		}
		ret = ret << 8;
		byte dataByte = readInputOutputPort();	// has a 120ns wait before, no WaitNoBusy
		ret = ret | dataByte;
	}
	
	return ret;
}  

 /// waits for Not Busy, reads two bytes and packs them into an int word
int LM629::readShortData(void)
{
	int ret = 0L;
	
	waitNotBusy();
	
    for(int i=0; i < 2 ;i++)
    {
		ret = ret << 8;
		byte dataByte = readInputOutputPort();	// has a 120ns wait before, no WaitNoBusy
		ret = ret | dataByte;
	}
	
	return ret;
}  

void LM629::readSignalsRegister(void)
  {
	// read two signal bytes via a command:
    writeCommandByte(LM629_COMMAND_RDSIGS);

    waitNotBusy();

    byte signalsRegisterHighByte = readDataByte();

    byte signalsRegisterLowByte = readDataByte();

    waitNotBusy();

	// now read status byte:
    byte xStatusByte = readStatusByte();
    
#ifdef LM629_DEBUG_PRINT
	// print out Signal and Status bytes:
    Serial.print(chip_1_selected ? "Chip 1  signals register:" : "Chip 2  signals register:");
    Serial.print("   High Byte=");
    Serial.print(signalsRegisterHighByte, BIN);
    Serial.print("   Low Byte=");
    Serial.print(signalsRegisterLowByte, BIN);

    Serial.print("   Status Byte: "); Serial.println(xStatusByte, HEX);  
#endif // LM629_DEBUG_PRINT
  }  
 
/// Read Real Position (native RDRP command)
long LM629::readRealPosition(void)
{
    writeCommandByte(LM629_COMMAND_RDRP);
	
	long ret = readLongData();
	
	return ret;
}

/// Read Real Velocity (native RDRV command)
int LM629::readRealVelocity(void)
{
    writeCommandByte(LM629_COMMAND_RDRV);
	
	int ret = readShortData();
	
	return ret;
}

/// Read Desired Velocity (native RDDV command)
long LM629::readDesiredVelocity(void)
{
    writeCommandByte(LM629_COMMAND_RDDV);
	
	long ret = readLongData();
	
	return ret;
}

/// Read Desired Position (native RDDP command)
long LM629::readDesiredPosition(void)
{
    writeCommandByte(LM629_COMMAND_RDDP);
	
	long ret = readLongData();
	
	return ret;
}

/// Read Integration Sum (native RDSUM command)
int LM629::readIntegrationSum(void)
{
    writeCommandByte(LM629_COMMAND_RDSUM);
	
	int ret = readShortData();
	
	return ret;
}
