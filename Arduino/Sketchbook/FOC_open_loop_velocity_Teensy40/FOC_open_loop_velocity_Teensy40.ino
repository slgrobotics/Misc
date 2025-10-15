#include <SimpleFOC.h>

//
// Tested on Ninebot miniPRO Oct 15, 2025
//

#define POWER_SUPPLY_VOLTAGE 12
#define DRIVER_VOLTAGE_LIMIT 3
#define POLE_PAIRS 15

// BLDC motor & driver instance
// Adjust pole_pairs and GPIO pins for your setup
BLDCMotor motor = BLDCMotor(POLE_PAIRS);  // Ninebot miniPRO: 15 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 12);
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, -1); // Example Pico GPIOs for PWM A, B, C; -1 for no enable pin

// Target variable
float target_velocity = 0;  // rad/s
float voltage_factor = 1.2; // set voltage limit proportional to target velocity

// Instantiate the commander:
Commander command = Commander(Serial);

void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
  motor.voltage_limit = constrain(abs(target_velocity) * voltage_factor, 0.0, DRIVER_VOLTAGE_LIMIT);
}

void setup() {

  // driver config:
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;  // [V]
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT; // miniPRO wheels are very low resistance, "1" is usually fine for slow speed, "3" for fast
  // Configure a specific PWM frequency for the driver
  //driver.pwm_frequency = 1000; // 1 kHz
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  motor.voltage_limit = 0;  // [V] - Adjusted based on target_velocity

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  //_delay(2000);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s] using T<x>:");
  _delay(1000);
}

void loop() {
  // open loop velocity movement
  motor.move(target_velocity);

  // user communication
  command.run();
}
