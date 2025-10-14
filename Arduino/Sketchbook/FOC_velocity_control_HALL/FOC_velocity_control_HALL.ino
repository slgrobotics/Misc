/**

   Velocity motion control example
   Steps:
   1) Configure the motor and sensor
   2) Run the code
   3) Set the target velocity (in radians per second) from serial terminal



   NOTE :
   > Specifically for Arduino UNO example code for running velocity motion control using a hall sensor
   > Since Arduino UNO doesn't have enough interrupt pins we have to use software interrupt library PciManager.

   > If running this code with Nucleo or Bluepill or any other board which has more than 2 interrupt pins
   > you can supply doC directly to the sensor.enableInterrupts(doA,doB,doC) and avoid using PciManger

*/
#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

// miniPRO wheels are very low resistance, "1" is fine:
#define DRIVER_VOLTAGE_LIMIT 3
#define POWER_SUPPLY_VOLTAGE 12
#define MOTOR_VOLTAGE_ALIGN 0.5
#define POLE_PAIRS 15

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// hall sensor instance
HallSensor sensor = HallSensor(2, 3, 4, POLE_PAIRS);

// Interrupt routine intialization
// channel A and B callbacks
void doA() {
  //Serial.println("doA");
  sensor.handleA();
}
void doB() {
  //Serial.println("doB");
  sensor.handleB();
}
void doC() {
  //Serial.println("doC");
  sensor.handleC();
}
// If no available hadware interrupt pins use the software interrupt
PciListenerImp listenerIndex(sensor.pinC, doC);

// velocity set point variable
float target_velocity = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}

void setup() {

  // use monitoring with serial
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed. See https://docs.simplefoc.com/debugging
  SimpleFOCDebug::enable(&Serial);

  // initialize HALL sensor hardware
  sensor.init(); // void sub

  sensor.enableInterrupts(doA, doB); //, doC);
  // third software interrupt for Arduino UNO:
  PciManager.registerListener(&listenerIndex);

  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
  driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;

  if (!driver.init()) {
    SIMPLEFOC_DEBUG("Driver init failed!");
    return;
  }

  // link the motor and the driver
  motor.linkDriver(&driver);

  // During the sensor align procedure, SimpleFOC moves the wheels and measures
  //     the direction of the sensor and the zero electrical angle offset.
  // set aligning voltage [Volts]:
  motor.voltage_sensor_align = MOTOR_VOLTAGE_ALIGN;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PID controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 2;
  motor.PID_velocity.D = 0;

  // initial motor voltage limit
  motor.voltage_limit = DRIVER_VOLTAGE_LIMIT;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;

  // comment out if not needed
  //motor.useMonitoring(Serial);

  SIMPLEFOC_DEBUG("IP: initializing motor and FOC...");

  // initialize motor
  if (!motor.init()) {
    SIMPLEFOC_DEBUG("Motor init failed!");
    return;
  }

  // align sensor and start FOC
  if (!motor.initFOC()) {
    SIMPLEFOC_DEBUG("FOC init failed!");
    return;
  }

  // add target velocity command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}


void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();

  // user communication
  command.run();
}
