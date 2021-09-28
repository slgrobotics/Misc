#include "MPU9250.h"

MPU9250 mpu;

void setup() 
{
    Serial.begin(115200);
    Wire.begin();
    delay(1000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    // load from eeprom
    loadCalibration();
}

void loop()
{
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            print_roll_pitch_yaw();
            prev_ms = millis();
        }
    }
}

void print_roll_pitch_yaw() 
{
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void loadCalibration() {
      mpu.setAccBias(-7.05, 38.97, -65.91);
      mpu.setGyroBias(-2.64, 0.31, -0.49);
      mpu.setMagBias(106.48, 263.52, -191.55);
      mpu.setMagScale(0.99, 0.97, 1.05);
}
