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

// BLDC motor & driver instances:
// BLDCMotor motor = BLDCMotor(pole pair number - 15 for miniPRO wheels with 30 magnets, 27 coils);
BLDCMotor motorL = BLDCMotor(POLE_PAIRS);
BLDCMotor motorR = BLDCMotor(POLE_PAIRS);

// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driverL = BLDCDriver3PWM(9, 10, 11, 8);
BLDCDriver3PWM driverR = BLDCDriver3PWM(3, 5, 6, 7);

//target velocity and voltage limit variables:
float target_velocity_L = 0;
float target_velocity_R = 0;
float volt_limit = 1; // [V] miniPRO wheels are very low resistance. 0.5A at this setting

// Commands over Serial (newline terminated): L1, T0, T0.5 T1, T2 ...
// instantiate the commander
Commander command = Commander(Serial);
void doTargetL(char* cmd) { command.scalar(&target_velocity_L, cmd); }
void doTargetR(char* cmd) { command.scalar(&target_velocity_R, cmd); }
void doLimit(char* cmd) { command.scalar(&volt_limit, cmd); motorL.voltage_limit = motorR.voltage_limit = driverL.voltage_limit = driverR.voltage_limit = volt_limit; }

void setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config - power supply voltage [V]
  driverL.voltage_power_supply = driverR.voltage_power_supply = POWER_SUPPLY_VOLTAGE;

  // limit the maximum dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driverL.voltage_limit = driverR.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  if(!driverL.init() || !driverR.init()){
    Serial.println("Driver init failed!");
    return;
  }

  // link motors and drivers:
  motorL.linkDriver(&driverL);
  motorR.linkDriver(&driverR);

  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motorL.voltage_limit = volt_limit;
  motorR.voltage_limit = volt_limit;
 
  // open loop control config
  motorL.controller = MotionControlType::velocity_openloop;
  motorR.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motorL.init() || !motorR.init()){
    Serial.println("Motors init failed!");
    return;
  }

  // add target commands: L and R for velocity, V for voltage limit:
  command.add('L', doTargetL, "target velocity_L");
  command.add('R', doTargetR, "target velocity_R");
  command.add('V', doLimit, "voltage limit");

  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  motorL.move(-target_velocity_L);
  motorR.move(target_velocity_R);

  // user communication
  command.run();
}
