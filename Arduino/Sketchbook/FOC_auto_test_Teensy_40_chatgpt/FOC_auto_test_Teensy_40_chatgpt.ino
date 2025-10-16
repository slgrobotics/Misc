// ======================================================
// SimpleFOC automated test for Teensy 4.0 + Ninebot wheel
// Uses Hall sensors, 12 V, safe PID values
// ======================================================

#include <SimpleFOC.h>

#define POWER_SUPPLY_VOLTAGE 12
#define MOTOR_VOLTAGE_ALIGN 1
#define POLE_PAIRS 15

const float voltage_limit = 6.0; // safe startup limit
const float current_limit = 3.0;  // A, conservative

// PWM pins (example for Teensy 4.0, adapt as wired)
#define A_H 2
#define B_H 3
#define C_H 4
#define A_L 5
#define B_L 6
#define C_L 9

// Hall sensor pins (adapt to your wheel)
#define HALL1 14
#define HALL2 15
#define HALL3 16
// -----------------------------------

//BLDCMotor motor(POLE_PAIRS);
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
//BLDCDriver6PWM driver(A_H, B_H, C_H, A_L, B_L, C_L);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 12);
//HallSensor sensor(HALL1, HALL2, HALL3, POLE_PAIRS);
HallSensor sensor = HallSensor(14, 15, 16, POLE_PAIRS);

// -------- PID tuning values from last message --------
void setupPID()
{
  // current q/d
  motor.PID_current_q.P = 1.00;
  motor.PID_current_q.I = 0.03;
  motor.PID_current_q.D = 0.00;

  motor.PID_current_d.P = 1.00;
  motor.PID_current_d.I = 0.03;
  motor.PID_current_d.D = 0.00;

  // velocity
  motor.PID_velocity.P = 0.08;  // 0.60;
  motor.PID_velocity.I = 0.75;   // 0.015;
  motor.PID_velocity.D = 0.004; //0.0002;

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

void setup() {
  Serial.begin(115200);
  delay(1500);

  // enable more verbose output for debugging
  // comment out if not needed. See https://docs.simplefoc.com/debugging
  SimpleFOCDebug::enable(&Serial);

  Serial.println("SimpleFOC automated test starting...");

  // driver setup
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.pwm_frequency = 20000;
  driver.init();

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
  motor.zero_electric_angle = 5.24; // rad
  motor.sensor_direction = Direction::CCW; // CW or CCW

  setupPID();

  motor.init();
  motor.initFOC();

  Serial.println("FOC initialized. Starting test sequence...");
  Serial.println("time_ms,target_vel,meas_vel,iq,voltage_q");
  t0 = millis();
}

void loop() {
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
  if (millis() - lastPrint > 50) {
    lastPrint = millis();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(target, 3);
    Serial.print(",");
    Serial.print(motor.shaft_velocity, 3);
    Serial.print(",");
    Serial.print(motor.current.q, 3);
    Serial.print(",");
    Serial.println(motor.voltage.q, 3);
  }
}
