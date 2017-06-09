
#include <LM629.h>
#include <LM629PID.h>

int ledPin = 13;       // Arduino UNO Yellow LED

LM629    *plm629;      // LM 629 bus operations
LM629PID *plm629pid;   // LM 629 PID and motor operations

void blinkLED(int nTimes, int halfPeriodMs)
  {
    for (int i = 0; i < nTimes; i++)
    {
      digitalWrite(ledPin, HIGH);
      delay(halfPeriodMs);
      digitalWrite(ledPin, LOW);
      delay(halfPeriodMs);
    }
  }

// ==========================================================

void setup()
  {
    Serial.begin(19200);

    digitalWrite(ledPin, LOW);
    pinMode(ledPin, OUTPUT);     

    delay(1000);
    
    Serial.println("*** Program start ***");

    plm629 = new LM629();
    
    plm629pid = new LM629PID(plm629);
  }
 
void loop()
  {
    blinkLED(20, 50);

    plm629->init();
    
    //plm629pid->loadFilter01('A');
    //plm629pid->loadFilter02('A');
    plm629pid->loadFilter('A', 1, 20, 4, 64, 4000);
    //plm629pid->loadTrajectory01('A');
    //plm629pid->stopMotor01('A');
    //plm629pid->loopPhasing01('A');
    plm629->readSignalsRegister();
    //delay(1000);    
    //plm629pid->simpleMove01('A');
    
    long acc = 50L;
    long vel = 1000000L;
    long pos = 50000L;
    
    Serial.print("acc: "); Serial.println(acc);
    Serial.print("vel: "); Serial.println(vel);
    Serial.print("pos: "); Serial.println(pos);
    
    plm629pid->moveAccelerationVelocityPositionAbsolute('A', acc, vel, -pos);

    //plm629pid->loadFilter01('B');
    //plm629pid->loadFilter02('B');
    plm629pid->loadFilter('B', 1, 20, 4, 64, 4000);
    //plm629pid->loadFilter('B', 1, 20, 3, 150, 2000);
    //plm629pid->loadTrajectory01('B');
    //plm629pid->stopMotor01('B');
    //plm629pid->loopPhasing01('B');
    plm629->readSignalsRegister();
    //delay(1000);    
    //plm629pid->simpleMove01('B');

    //acc = 200000L;
    //vel = 7000000L;
    //pos = -2000L;
    
    plm629pid->moveAccelerationVelocityPositionAbsolute('B', acc, vel, pos);

    while(true)
    {
      plm629->chip_1_selected = true;
      Serial.print("Chip 1 real Pos: "); Serial.print(plm629->readRealPosition()); Serial.print(" of "); Serial.print(plm629->readDesiredPosition());
      Serial.print("   Velocity: "); Serial.print(plm629->readRealVelocity()); Serial.print(" of "); Serial.print(plm629->readDesiredVelocity());
      Serial.print("   Integration Sum: "); Serial.print(plm629->readIntegrationSum()); Serial.print("   ");
      plm629->readSignalsRegister();

      plm629->chip_1_selected = false;
      Serial.print("Chip 2 real Pos: "); Serial.print(plm629->readRealPosition()); Serial.print(" of "); Serial.print(plm629->readDesiredPosition());
      Serial.print("   Velocity: "); Serial.print(plm629->readRealVelocity()); Serial.print(" of "); Serial.print(plm629->readDesiredVelocity());
      Serial.print("   Integration Sum: "); Serial.print(plm629->readIntegrationSum()); Serial.print("   ");
      plm629->readSignalsRegister();

      blinkLED(1, 500);
      //delay(1000);
    }
    
    //plm629pid->simpleMove02('A');
    //delay(1000);    
    //plm629pid->madMove01('A');  
    //delay(1000);    
    //  hardwareSystemReset();  
    //  softwareSystemReset();
    //  softwareInterruptReset();    
    //  initMotionController();  
    blinkLED(1000, 100);

    Serial.println("*** Program finished ***");
    while (true)
      { ; }        
  }
  
