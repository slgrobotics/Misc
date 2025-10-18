// ======================================================
// SimpleFOC automated test for Teensy 4.0 + Ninebot wheel
// Uses Hall sensors, 12 V, safe PID values
//
// same as FOC_auto_test_Teensy_40_cs_chatgpt - but with inline current sensing
//
// ======================================================

#include <SimpleFOC.h>

#define POWER_SUPPLY_VOLTAGE 12
#define MOTOR_VOLTAGE_ALIGN 2
#define POLE_PAIRS 15

#define RIGHT_WHEEL

const float voltage_limit = 6.0;  // safe startup limit
const float current_limit = 3.0;  // A, conservative

//BLDCMotor motor(POLE_PAIRS);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);

//BLDCDriver6PWM driver(A_H, B_H, C_H, A_L, B_L, C_L);
// Note: For 6PWM action GP 24...31 are available

#ifdef RIGHT_WHEEL
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 13); // Right wheel. 13 is also BUILTIN_LED
#else
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 8);    // Left wheel
#endif // RIGHT_WHEEL

//HallSensor sensor(HALL1, HALL2, HALL3, POLE_PAIRS);
#ifdef RIGHT_WHEEL
HallSensor sensor = HallSensor(12, 14, 15, POLE_PAIRS); // Right wheel
#else
HallSensor sensor = HallSensor(2, 3, 4, POLE_PAIRS);    // Left wheel
#endif // RIGHT_WHEEL

// Note: A2 (16) and A9 (23) are left free for joystick, as are A32 (Active) and A33 (Pressed)

// InlineCurrentSensor - https://docs.simplefoc.com/inline_current_sense
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  Option: if measuring 2 out of 3 currents, put the flag _NC to the phase you are not using.
#ifdef RIGHT_WHEEL
InlineCurrentSense current_sense = InlineCurrentSense(0.005, 24, A3, A4, A5); // Right wheel
#else
InlineCurrentSense current_sense = InlineCurrentSense(0.005, 24, A6, A7, A8); // Left wheel
#endif // RIGHT_WHEEL

// -------- PID tuning values from last message --------
void setupPID() {
  // current q/d
  motor.PID_current_q.P = 1.00;
  motor.PID_current_q.I = 0.03;
  motor.PID_current_q.D = 0.00;

  motor.PID_current_d.P = 1.00;
  motor.PID_current_d.I = 0.03;
  motor.PID_current_d.D = 0.00;

  // velocity
  motor.PID_velocity.P = 0.08;   // 0.60;
  motor.PID_velocity.I = 0.75;   // 0.015;
  motor.PID_velocity.D = 0.004;  //0.0002;

  // position (optional)
  // motor.P_angle.P = 18.0;
  // motor.P_angle.I = 0.0;
  // motor.P_angle.D = 0.0;
}
// -----------------------------------------------------

// variables for test sequence
float t0;
int test_state = 0;
float target = 0;
bool init_ok = false;

void setup() {
  Serial.begin(115200);
  _delay(2000);

  // enable more verbose output for debugging
  // comment out if not needed. See https://docs.simplefoc.com/debugging
  SimpleFOCDebug::enable(&Serial);

  Serial.println("SimpleFOC automated test starting...");

  // driver setup
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.pwm_frequency = 20000;
  driver.init();

  // link current sense and driver
  current_sense.linkDriver(&driver);
  if (!current_sense.init()) {
    // also performs zero current offset
    Serial.println("Error: current_sense.init() failed");
    return;
  }

  // link sensor + driver + PID
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  sensor.init();

  // limits
  motor.voltage_limit = voltage_limit;
  motor.current_limit = current_limit;
  motor.LPF_velocity = 0.04;
  motor.PID_velocity.output_ramp = 300.0;

  // During the sensor align procedure, SimpleFOC moves the wheels and measures
  //     the direction of the sensor and the zero electrical angle offset.
  // set aligning voltage [Volts]:
  motor.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;

  // To skip alignment, supply known parameters:
  //motor.zero_electric_angle = 5.24; // rad
  //motor.sensor_direction = Direction::CCW; // CW or CCW

  setupPID();

  if (!motor.init()) {
    Serial.println("Error: motor.init() failed");
    return;
  }

  // Comment out the following link for "standalone" current sensor configuration
  // (sensing works for monitoring, but doesn't contribute to FOC):
  //motor.linkCurrentSense(&current_sense);

  // may take several tries, as current sensors are finicky:
  if (!motor.initFOC()) {
    Serial.println("Error: motor.initFOC() failed");
    _delay(2000);
    return;
  }

  Serial.println("OK: FOC initialized. Starting test sequence...");
  Serial.println("time_ms,target_vel,meas_vel,iq,voltage_q");

  t0 = millis();
  init_ok = true;
}

void loop() {

  if (!init_ok) {
    delay(1000);
    return;
  }

  motor.loopFOC();

  // --- simple automated sequence ---
  unsigned long t = millis() - t0;

  if (t < 4000) {
    // idle 0 → wait for stable startup
    target = 0;
  } else if (t < 8000) {
    // accelerate slowly forward
    target = 2.0;  // rad/s
  } else if (t < 12000) {
    // stop
    target = 0.0;
  } else if (t < 16000) {
    // accelerate reverse
    target = -2.0;
  } else if (t < 20000) {
    // stop
    target = 0.0;
  } else {
    // loop again
    t0 = millis();
  }

  motor.move(target);

  // log every 50 ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();

    // https://docs.simplefoc.com/inline_current_sense#standalone-current-sense
    PhaseCurrent_s current_ph = current_sense.getPhaseCurrents();
    float current_dc = current_sense.getDCCurrent();

    Serial.print("DC: ");
    Serial.print(current_dc);
    Serial.print(" A\tPhases:\t");
    Serial.print(current_ph.a);  // milli Amps
    Serial.print("\t");
    Serial.print(current_ph.b);
    Serial.print("\t");
    Serial.print(current_ph.c);

    DQCurrent_s current_dq = current_sense.getFOCCurrents(motor.electrical_angle);
    Serial.print("\tDQ:\t");
    Serial.print(current_dq.d);
    Serial.print("\t");
    Serial.println(current_dq.q);

    /*
    Serial.print(millis());
    Serial.print(",");
    Serial.print(target, 3);
    Serial.print(",");
    Serial.print(motor.shaft_velocity, 3);
    Serial.print(",");
    Serial.print(motor.current.q, 3);
    Serial.print(",");
    Serial.println(motor.voltage.q, 3);
    */
  }
}
