//
// see https://github.com/slgrobotics/robots_bringup/tree/main/Docs/miniPRO
//
// Seggy ("Smart Table" robot is based on Segway Ninebot miniPRO
//

// Note: HallSensor's and POLE_PAIRS defined in Encoders.ino tab

#define POWER_SUPPLY_VOLTAGE 12
//#define DRIVER_VOLTAGE_LIMIT 6
#define MOTOR_VOLTAGE_LIMIT 8
#define MOTOR_VOLTAGE_ALIGN 2
//#define POLE_PAIRS 15

const float current_limit = 3.0;  // A, conservative

//target velocity and voltage limit variables:
float target_velocity_L = 0;
float target_velocity_R = 0;
float volt_limit = 1; // [V] miniPRO wheels are very low resistance. 0.5A at this setting

BLDCMotor motorL = BLDCMotor(POLE_PAIRS);
BLDCMotor motorR = BLDCMotor(POLE_PAIRS);

//BLDCDriver6PWM driver(A_H, B_H, C_H, A_L, B_L, C_L);
// Note: For 6PWM action GP 26...31 are available, if not used for LEDs

BLDCDriver3PWM driverL = BLDCDriver3PWM(5, 6, 7, 8);    // Left wheel
BLDCDriver3PWM driverR = BLDCDriver3PWM(9, 10, 11, 13); // Right wheel. 13 is also BUILTIN_LED

//HallSensor sensor(HALL1, HALL2, HALL3, POLE_PAIRS);
//HallSensor sensorL = HallSensor(2, 3, 4, POLE_PAIRS);    // Left wheel
//HallSensor sensorR = HallSensor(12, 14, 15, POLE_PAIRS); // Right wheel

// Note: A2 (16) and A9 (23) are used by joystick, as are A32 (Active) and A33 (Pressed). A10 (24) used by battery monitor.

// InlineCurrentSensor - https://docs.simplefoc.com/inline_current_sense
//  - shunt_resistor  - shunt resistor value (0.005 Ohm)
//  - gain  - current-sense op-amp gain (24 in this case)
//  Option: if measuring 2 out of 3 currents, put the flag _NC to the phase you are not using.
InlineCurrentSense current_senseL = InlineCurrentSense(0.005, 24, A6, A7, A8); // Left wheel
InlineCurrentSense current_senseR = InlineCurrentSense(0.005, 24, A3, A4, A5); // Right wheel

// -------- PID tuning values from last message --------
void setupPID() {

  float vel_P = 0.4;
  float vel_I = 2.0;
  float vel_D = 0.0;

  float curr_P = 1.0;
  float curr_I = 0.03;
  float curr_D = 0.001;

  // current q/d
  motorL.PID_current_q.P = curr_P;
  motorL.PID_current_q.I = curr_I;
  motorL.PID_current_q.D = curr_D;

  motorL.PID_current_d.P = curr_P;
  motorL.PID_current_d.I = curr_I;
  motorL.PID_current_d.D = curr_D;

  // velocity
  motorL.PID_velocity.P = vel_P;
  motorL.PID_velocity.I = vel_I;
  motorL.PID_velocity.D = vel_D;

  // position (optional)
  // motorL.P_angle.P = 18.0;
  // motorL.P_angle.I = 0.0;
  // motorL.P_angle.D = 0.0;

  // current q/d
  motorR.PID_current_q.P = curr_P;
  motorR.PID_current_q.I = curr_I;
  motorR.PID_current_q.D = curr_D;

  motorR.PID_current_d.P = curr_P;
  motorR.PID_current_d.I = curr_I;
  motorR.PID_current_d.D = curr_D;

  // velocity
  motorR.PID_velocity.P = vel_P;
  motorR.PID_velocity.I = vel_I;
  motorR.PID_velocity.D = vel_D;

  // position (optional)
  // motorR.P_angle.P = 18.0;
  // motorR.P_angle.I = 0.0;
  // motorR.P_angle.D = 0.0;
}

bool MotorsInit()
{
  target_velocity_L = target_velocity_R = 0.0f;

#ifdef TRACE
  // enable more verbose output for debugging
  // comment out if not needed. See https://docs.simplefoc.com/debugging
  SimpleFOCDebug::enable(&Serial);
#endif // TRACE

  // driver config - power supply voltage [V]
  driverL.voltage_power_supply = driverR.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  //driverL.voltage_limit = driverR.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driverL.pwm_frequency = driverR.pwm_frequency = 20000;
  // Force center-aligned PWM mode to avoid warning at startup (library version 2.4.x+):
  //driverL.center_aligned = driverR.center_aligned = true;

  // limit the maximum dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  if(!driverL.init() || !driverR.init()){
    Serial.println("Error: Drivers init failed!");
    return false;
  }

  // link current sense and driver
  current_senseL.linkDriver(&driverL);
  current_senseR.linkDriver(&driverR);
  if (!current_senseL.init() || !current_senseR.init()) {
    // also performs zero current offset
    Serial.println("Error: current_sense.init() failed");
    return false;
  }

  // link motors and drivers:
  motorL.linkDriver(&driverL);
  motorR.linkDriver(&driverR);

  // Link sensors to motors:
  motorL.linkSensor(&sensorL);
  motorR.linkSensor(&sensorR);
  sensorL.init();
  sensorR.init();

  EncodersInit();    // Initialize the encoder's variables, enable interrupt handlers

  // limiting motor movements
  // Note: enables basic anti-windup mechanism if you limit the voltage or current.
  // limit the voltage to be set to the motor. Start very low for high resistance motors.
  // current = voltage / resistance, so try to be well under 1Amp
  motorL.voltage_limit = motorR.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motorL.LPF_velocity = motorR.LPF_velocity = 0.2; // a low-pass filter to smooth out noisy velocity measurements, derived from position sensor.
  motorL.PID_velocity.output_ramp = motorR.PID_velocity.output_ramp = 300.0;
 
  // During the sensor align procedure, SimpleFOC moves the wheels and measures
  //     the direction of the sensor and the zero electrical angle offset.
  // set aligning voltage [Volts]:
  motorL.voltage_sensor_align = motorR.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;

  // open loop control config
  motorL.controller = MotionControlType::velocity;
  motorR.controller = MotionControlType::velocity;

  setupPID();

  // init motor hardware
  if(!motorL.init() || !motorR.init()){
    Serial.println("Error: Motors init failed!");
    return false;
  }

  if (!motorL.initFOC() || !motorR.initFOC()) {
    Serial.println("Error: FOC init failed!");
    return false;
  }

  set_motors();

#ifdef TRACE
  Serial.println("OK: FOC initialized. Starting aligning sequence...");
#endif // TRACE

  return true;
}

// ******************************************************************************************
//
//   Set motor power for both motors. Positive is forward.
//
// ******************************************************************************************
void set_motors()
{
  // convert setpoints to rad/s for FOC
  target_velocity_L = map(constrain(speedSetpointL, -100, 100), -100, 100, -MAX_SPEED_RAD_S, MAX_SPEED_RAD_S);
  target_velocity_R = map(constrain(speedSetpointR, -100, 100), -100, 100, -MAX_SPEED_RAD_S, MAX_SPEED_RAD_S);
}

void getFOCCurrents()
{
    // https://docs.simplefoc.com/inline_current_sense#standalone-current-sense
    current_dc_L = current_senseL.getDCCurrent();
    current_dc_R = current_senseR.getDCCurrent();
    current_ph_L = current_senseL.getPhaseCurrents();
    current_ph_R = current_senseR.getPhaseCurrents();
    current_dq_L = current_senseL.getFOCCurrents(motorL.electrical_angle);
    current_dq_R = current_senseR.getFOCCurrents(motorR.electrical_angle);
}

float prev_target_velocity_L = 0.0;
float prev_target_velocity_R = 0.0;

void motorLoop()
{
  // Execute FOC algorithm. Runs as often as possible.
  motorL.loopFOC();
  motorR.loopFOC();

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity

  if (abs(target_velocity_L) < 0.05 || (prev_target_velocity_L * target_velocity_L) < 0) {
      // anti-windup protection, reset accumulated integral on stops and direction reversals
      motorL.PID_velocity.reset();
  }
  prev_target_velocity_L = target_velocity_L;
  motorL.move(-target_velocity_L);

  if (abs(target_velocity_R) < 0.05 || (prev_target_velocity_R * target_velocity_R) < 0) {
      motorR.PID_velocity.reset();
  }
  prev_target_velocity_R = target_velocity_R;
  motorR.move(target_velocity_R);
}
