// Open loop motor control example
#include <SimpleFOC.h>

//
// This is slightly modified example from simpleFOC linbrary.
// Works with either wheel of Segway Ninebot miniPRO,
// simpleFOC driver board - https://github.com/ChenDMLSY/FOC-SimpleFOC-MotorDriveDevelopmentBoard/tree/main
//
// see https://photos.app.goo.gl/yHXs7fP7u7ae8fa78
//
// Compiled file - look for something like "myBlink.ino.uf2" in:
//           1.8.19:  admin:///tmp/snap-private-tmp/snap.arduino/tmp/arduino_build_624141
//           2.3.6:   /home/sergei/.cache/arduino/sketches/CDE341FACC5958BD1FA063FDB3D2894A
//
// Does not work as of Oct 15, 2025
//

/*
Recommended groups for a 3PWM BLDC driver:
Group 1:
Phase A: GP8 (Slice 4, Channel A)
Phase B: GP9 (Slice 4, Channel B)
Phase C: GP10 (Slice 5, Channel A)
Group 2:
Phase A: GP6 (Slice 3, Channel A)
Phase B: GP7 (Slice 3, Channel B)
Phase C: GP8 (Slice 4, Channel A) 
*/

// miniPRO wheels are very low resistance, "1" is fine:
#define DRIVER_VOLTAGE_LIMIT 2
#define POWER_SUPPLY_VOLTAGE 12
#define POLE_PAIRS 15

bool init_done = false;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
//BLDCDriver3PWM driver = BLDCDriver3PWM(6, 7, 8, 9); // GP6...9
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 11, 12, 13); // GP10...13
//BLDCDriver3PWM driver = BLDCDriver3PWM(8, 9, 10, 7);

// target variable
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
  
  init_done = false;

  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  
  // use monitoring with serial
  Serial.begin(115200);
  delay(3000);
}

bool init_me()
{
  Serial.println("IP: init_me()");

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
  //driver.pwm_frequency = 1000; // 1 kHz

  if (!driver.init()) {
    Serial.println("Error: Driver init failed!");
    return false;
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
    return false;
  }

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('V', doLimit, "voltage limit");

  Serial.println("OK: Motor ready.");
  Serial.println("Set target velocity [rad/s] using T<x> or Voltage limit using V<x>");
  _delay(1000);

  init_done = true;
  
  return true;
}

void loop() {

  if(init_done != true) {
    delay(1000);
    Serial.println("IP: calling init_me()");
    if(!init_me()) {
      Serial.println("Error: init failed in loop()");
      delay(1000);
      return;
    } else {
      Serial.println("OK: init_me() successful");
    }
  }

  //Serial.println("..in loop()..");
  //delay(1000);

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motor.move(target_velocity);

  // user communication
  command.run();
}
