#include "PCA9685Emulator.h"

#include <PWMServo.h>

//#define TRACE

// emulate the PCA9685 Raspberry Pi Hat using Teensy 4.0 microcontroller
// ensures "safe" positioning of all servos when the host RPi dies
// https://github.com/slgrobotics/Misc/Teensy_PCA9685
//
// Original code: https://github.com/jwatte/donkey_racing/tree/master
// Use Arduino IDE 2.x.x ("Windows app" version) with Teensy extensions

// Teensy 4.0 Wire:  SDA: 18 SCL: 19
//            Wire1: SDA: 17 SCL: 16

// IMPORTANT: increase Wire receive buffer size in the library, as PX4 sends 65 bytes for updateOutputs():
//   C:\Projects\Arduino\Sketchbook\libraries\teensy4_i2c\src\i2c_driver_wire.h : 23
//   static const size_t rx_buffer_length = 256;

//  PWM values are between US_MIN and US_MAX, with 1500 meaning "zero"

PCA9685Emulator pwmEmulation;

#define N_SERVOS 8
#define US_MIN 1000
#define US_MAX 2000

PWMServo* servos[N_SERVOS];

// I only needed 8 channels, so I use Wire (SCL0/SDA0).
// Teensy offers more PWM pins for 16-channel operation: 10, 11, 12, 14, 15, 22, 23  (note LED_BUILTIN=13).
// Use Wire1 (pins SDA1=17, SCL1=16) to free 18,19 pins for PWM

int servopins[N_SERVOS] = { 2, 3, 4, 5, 6, 7, 8, 9 };
int servo_safe_us[N_SERVOS] = { 1500, 1500, 1000, 1000, 1000, 1000, 1500, 1500 };

unsigned long last_step_ms = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);
  // Wait for Serial Comms or Timeout after 5 seconds
  while (!Serial && millis() < 5000)
    ;

  initServos();

  pwmEmulation.begin(PCA9685_I2C_ADDRESS);

#ifdef TRACE
  Serial.println("Teensy 4.0 PCA9685 Emulator - listening to I2C");
#endif  // TRACE
}

void loop() {

  if (pwmEmulation.step()) {

    // we've got data from the host

    digitalWrite(LED_BUILTIN, HIGH);  // lit the LED if all goes well

#ifdef TRACE
    Serial.println("");
    for (int i = 0; i < pwmEmulation.NUM_CHANNELS; i++) {
      int servo_us = pwmEmulation.readChannelUs(i);
      Serial.print("  SERVO ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(servo_us);
    }
    Serial.println("");
#endif  // TRACE

    for (int i = 0; i < N_SERVOS; i++) {
      servos[i]->write(usToAngle(pwmEmulation.readChannelUs(i)));
    }

    last_step_ms = millis();

  } else if (millis() - last_step_ms > 1000) {

    // host haven't sent data for more than 1 second.

    digitalWrite(LED_BUILTIN, LOW);

    safeServos();
  }

  delay(1);
}

void initServos() {
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i] = new PWMServo();
    servos[i]->attach(servopins[i], US_MIN, US_MAX);
  }
}

void safeServos() {
  // put all servos in safe positions:
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i]->write(usToAngle(servo_safe_us[i]));
  }
}

int usToAngle(int us) {
  // convert incoming microseconds to angle:
  return int(180 * (us - US_MIN) / (US_MAX - US_MIN));
}
