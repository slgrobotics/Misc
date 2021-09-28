
// MPU 9250/6500 in I2C mode, use RPi Pico pins GP6 = SDA, GP7 = SCL

// https://github.com/hideakitai/MPU9250

MPU9250 mpu;

void mpuInit() 
{
    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    // load calibration values (see calibration example)
    loadCalibration();
}

// result of prior calibration for this particular unit:
void loadCalibration() 
{
      mpu.setAccBias(-7.05, 38.97, -65.91);
      mpu.setGyroBias(-2.64, 0.31, -0.49);
      mpu.setMagBias(106.48, 263.52, -191.55);
      mpu.setMagScale(0.99, 0.97, 1.05);
}

void processMpu()
{
    if (mpu.update()) {
      yaw = mpu.getYaw(); // -180...180
      pitch = mpu.getPitch();
      roll = mpu.getRoll();
    }
}

#ifdef TRACE
void printYawPitchRoll() 
{
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(yaw, 2);
    Serial.print(", ");
    Serial.print(pitch, 2);
    Serial.print(", ");
    Serial.println(roll, 2);
}
#endif // TRACE
