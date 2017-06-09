#ifndef F_CPU
#define F_CPU 16000000UL  // Standard Arduinos run at 16 MHz
#endif //!F_CPU

#include "LM629PID.h"

/// Constructor
LM629PID::LM629PID(LM629 *plm)
{
	pLm629 = plm;
}

/// Destructor
LM629PID::~LM629PID(void)
{
}

void LM629PID::init(void)
{
}

#ifdef LM629_DEBUG_PRINT

void LM629PID::checkPoint(char *xComment)
  {
    Serial.print(pLm629->chip_1_selected ? "Chip 1  " : "Chip 2  ");
    Serial.print(xComment);
	
    pLm629->waitNotBusy();
    byte xStatusByte = pLm629->readStatusByte();

    Serial.print("   Status Byte: "); Serial.println(xStatusByte, HEX);  
  }

#endif // LM629_DEBUG_PRINT


/// * LM629 movement *

void LM629PID::stopMotor(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeTwoDataBytes(LM629PID_STOP_MOTOR, LM629PID_NO_TRAJECTORY_PARAMETERS_TO_LOAD);    
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After stopping motor, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::loopPhasing01(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeTwoDataBytes(LM629PID_LOOP_PHASING, LM629PID_NO_TRAJECTORY_PARAMETERS_TO_LOAD);    
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loop phasing, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::simpleMove02(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_ABSOLUTE);   
    pLm629->writeLongData(2L);
    pLm629->writeLongData(36000L);
    pLm629->writeLongData(32192L);

    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After simpleMove 02, ");  
#endif // LM629_DEBUG_PRINT
  }
  
void LM629PID::madMove01(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_ABSOLUTE);   
    pLm629->writeShortData(0xFFFF);
    pLm629->writeShortData(0xFFFF);
    pLm629->writeShortData(0xFFFF);
    pLm629->writeShortData(0xFFFF);
    pLm629->writeShortData(0xFFFF);
    pLm629->writeShortData(0xFFFF);

    pLm629->writeCommandByte(LM629_COMMAND_STT);  
    
#ifdef LM629_DEBUG_PRINT
    checkPoint("After madMove 01, ");  
#endif // LM629_DEBUG_PRINT
  }  

void LM629PID::loadFilter01(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LFIL);
    pLm629->writeTwoDataBytes(1, LM629PID_LOAD_KP_ONLY);    
    pLm629->writeShortData(20);			// KP
    
    pLm629->writeCommandByte(LM629_COMMAND_UDF);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loading filter 01, ");  
#endif // LM629_DEBUG_PRINT
  }      

void LM629PID::loadFilter02(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LFIL);
    pLm629->writeTwoDataBytes(64, LM629PID_LOAD_KP_KD_KI);    
    pLm629->writeShortData(20); 		// KP    
    pLm629->writeShortData(4);     		// KI
    pLm629->writeShortData(64);     	// KD
    
    pLm629->writeCommandByte(LM629_COMMAND_UDF);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loading filter 02, ");  
#endif // LM629_DEBUG_PRINT
  }      

void LM629PID::loadFilter03(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LFIL);
    pLm629->writeTwoDataBytes(1, LM629PID_LOAD_KP_KD_KI_IL);    
    pLm629->writeShortData(20); 		// KP    
    pLm629->writeShortData(4);     		// KI
    pLm629->writeShortData(64);     	// KD
    pLm629->writeShortData(4000);       // IL
    
    pLm629->writeCommandByte(LM629_COMMAND_UDF);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loading filter 03, ");  
#endif // LM629_DEBUG_PRINT
  }      

  
/// sampling interval is in increments of 2048 * 125ns = 256us  - used for derivative caclulations
void LM629PID::loadFilter(byte side, byte interval, short kp, short ki, short kd, short il)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LFIL);
    pLm629->writeTwoDataBytes(interval, LM629PID_LOAD_KP_KD_KI_IL);    
    pLm629->writeShortData(kp); 		// KP    
    pLm629->writeShortData(ki);     	// KI
    pLm629->writeShortData(kd);     	// KD
    pLm629->writeShortData(il);     	// IL
    
    pLm629->writeCommandByte(LM629_COMMAND_UDF);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loading filter, ");  
#endif // LM629_DEBUG_PRINT
  }      

void LM629PID::loadTrajectory01(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_VELOCITY_POSITION_ABSOLUTE);    
    pLm629->writeLongData(0L);
  
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loading trajectory 01, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::simpleMove01(byte side)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_ABSOLUTE);   
    pLm629->writeLongData(2L);
    pLm629->writeLongData(36000L);
    pLm629->writeLongData(32192L);
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After simple move 01, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::loadAcceleration(byte side, long acceleration)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION);   
	pLm629->writeLongData(acceleration);
    
    //pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After loadAcceleration, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::moveAccelerationVelocity(byte side, long acceleration, long velocity)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION_VELOCITY);   
	pLm629->writeLongData(acceleration);
	pLm629->writeLongData(velocity);
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After moveAccelerationVelocity, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::moveAccelerationVelocityPositionAbsolute(byte side, long acceleration, long velocity, long position)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_ABSOLUTE);   
	pLm629->writeLongData(acceleration);
	pLm629->writeLongData(velocity);
	pLm629->writeLongData(position);
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After moveAccelerationVelocityPositionAbsolute, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::moveAccelerationVelocityPositionRelative(byte side, long acceleration, long velocity, long position)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_ACCELERATION_VELOCITY_POSITION_RELATIVE);   
	pLm629->writeLongData(acceleration);
	pLm629->writeLongData(velocity);
	pLm629->writeLongData(position);
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After moveAccelerationVelocityPositionRelative, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::moveVelocityPositionAbsolute(byte side, long velocity, long position)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_VELOCITY_POSITION_ABSOLUTE);   
	pLm629->writeLongData(velocity);
	pLm629->writeLongData(position);
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After moveVelocityPositionAbsolute, ");  
#endif // LM629_DEBUG_PRINT
  }

void LM629PID::moveVelocityPositionRelative(byte side, long velocity, long position)
  {
	pLm629->chip_1_selected = side == 'A' || side == 'L';
  
    pLm629->writeCommandByte(LM629_COMMAND_LTRJ);
    pLm629->writeShortData(LM629PID_LOAD_VELOCITY_POSITION_RELATIVE);   
	pLm629->writeLongData(velocity);
	pLm629->writeLongData(position);
    
    pLm629->writeCommandByte(LM629_COMMAND_STT);  

#ifdef LM629_DEBUG_PRINT
    checkPoint("After moveVelocityPositionRelative, ");  
#endif // LM629_DEBUG_PRINT
  }

