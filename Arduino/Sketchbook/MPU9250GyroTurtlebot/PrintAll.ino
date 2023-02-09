
#define PRINT_INTERVAL_MS 100

unsigned long lastPrintMillis = 0;

// warning: this takes significant time and disrupts control loop a bit
void printAll()
{
  if (millis() - lastPrintMillis > PRINT_INTERVAL_MS)
  {
    lastPrintMillis = millis();

    if (doTRACE)
    {
      Serial.print(yaw);
      Serial.print(" "); Serial.print(yawDelta);
      Serial.print(" "); Serial.print(yawc);
      //Serial.print(" "); Serial.print(adjustPotValue);
      //Serial.print("Yaw: "); Serial.print(yaw);
      //Serial.print(" PWM: "); Serial.print(yawc);
      //Serial.print(" Pitch: "); Serial.print(mympu.ypr[1]);
      //Serial.print(" Roll: "); Serial.print(mympu.ypr[2]);
      //      Serial.print("\tgy: "); Serial.print(mympu.gyro[0]);
      //      Serial.print(" gp: "); Serial.print(mympu.gyro[1]);
      //      Serial.print(" gr: "); Serial.print(mympu.gyro[2]);
      Serial.println();
    }
  }
}


void errorReporting() {
  if (ret != 0) {
    switch (ret) {
      case 0: c++; break;
      case 1: np++; return;
      case 2: err_o++; return;
      case 3: err_c++; return;
      default:
        Serial.print("READ ERROR!  ");
        Serial.println(ret);
    }
    Serial.print(np);
    Serial.print("  ");
    Serial.print(err_c);
    Serial.print(" ");
    Serial.print(err_o);
    Serial.print(" ");
  }
}

