// Open loop motor control example
#include <SimpleFOC.h>

//
// This is slightly modified example from simpleFOC linbrary.
// Works with either wheel of Segway Ninebot miniPRO,
// simpleFOC driver board - https://github.com/ChenDMLSY/FOC-SimpleFOC-MotorDriveDevelopmentBoard/tree/main
//
// see https://photos.app.goo.gl/yHXs7fP7u7ae8fa78
//

// miniPRO wheels are very low resistance, "1" is fine:
#define DRIVER_VOLTAGE_LIMIT 2
#define POWER_SUPPLY_VOLTAGE 12
#define POLE_PAIRS 15

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8); // original example, works
//BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6, 2); // try 1 - works
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8); // try 2 - works
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

//target variable
float target_velocity = 0;

// Commands over Serial (newline terminated): L1, T0, T0.5 T1, T2 ...
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
}

void setup() {

  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;

  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT; // miniPRO wheels are very low resistance, "1" is fine

  if (!driver.init()) {
    Serial.println("Error: Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 1;   // [V] miniPRO wheels are very low resistance. 0.5A at this setting

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if (!motor.init()) {
    Serial.println("Error: Motor init failed!");
    return;
  }

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('V', doLimit, "voltage limit");

  Serial.println("OK: Motor ready.");
  Serial.println("Set target velocity [rad/s] using T<x> or Voltage limit using V<x>");
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motor.move(target_velocity);

  // user communication
  command.run();
}
