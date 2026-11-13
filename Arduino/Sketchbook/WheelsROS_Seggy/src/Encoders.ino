// Note: Seggy robot does not have encoders. Hall sensors are used instead.

#define POLE_PAIRS 15

//HallSensor sensor(HALL1, HALL2, HALL3, POLE_PAIRS);
HallSensor sensorL = HallSensor(2, 3, 4, POLE_PAIRS);     // Left wheel
HallSensor sensorR = HallSensor(12, 14, 15, POLE_PAIRS);  // Right wheel

// **************************
//     Init the Encoders
// **************************
void EncodersInit() {
  // Left encoder:
  // Right encoder:

  sensorL.enableInterrupts(doA_L, doB_L, doC_L);
  sensorR.enableInterrupts(doA_R, doB_R, doC_R);

  EncodersReset();
}

void EncodersReset() {
  Ldistance = 0;
  Rdistance = 0;
  LdistancePrev = 0;
  RdistancePrev = 0;
}

// ********************************************************
// Read distance from the encoders
//
// one full wheel rotation on Seggy produces 60 ticks
// ********************************************************

bool prev_state_A_L = false;
bool prev_state_B_L = false;

bool prev_state_A_R = false;
bool prev_state_B_R = false;

// Lookup table for direction based on previous and current 2-bit states
static const int8_t dir_table[4][4] = {
  { 0, 1, -1, 0 },
  { -1, 0, 0, 1 },
  { 1, 0, 0, -1 },
  { 0, -1, 1, 0 }
};

// ChatGPT 5 version of the ISR:
// - Executes in constant time (no conditional branching),
// - Is extremely fast (a single table lookup)
void leftEncoder() {
  uint8_t A = digitalRead(sensorL.pinA);
  uint8_t B = digitalRead(sensorL.pinB);

  uint8_t curr = (A << 1) | B;
  uint8_t prev = (prev_state_A_L << 1) | prev_state_B_L;

  int8_t dir = dir_table[prev][curr];
  if (dir) Ldistance -= dir; // reversed

  prev_state_A_L = A;
  prev_state_B_L = B;
}

// Interrupt routine intialization
// channel A and B callbacks
void doA_L() {
  //Serial.println("doA");
  //uint8_t hall_state = sensor.readHallState(); // bitmask 0-2 bits; not in this library version (2.3) yet
  leftEncoder();
  sensorL.handleA();
}
void doB_L() {
  //Serial.println("doB");
  leftEncoder();
  sensorL.handleB();
}
void doC_L() {
  //Serial.println("doC");
  leftEncoder();
  sensorL.handleC();
}

void rightEncoder() {
  uint8_t A = digitalRead(sensorR.pinA);
  uint8_t B = digitalRead(sensorR.pinB);

  uint8_t curr = (A << 1) | B;
  uint8_t prev = (prev_state_A_R << 1) | prev_state_B_R;

  int8_t dir = dir_table[prev][curr];
  if (dir) Rdistance += dir;

  prev_state_A_R = A;
  prev_state_B_R = B;
}

// Interrupt routine intialization
// channel A and B callbacks
void doA_R() {
  //Serial.println("doA");
  //uint8_t hall_state = sensor.readHallState(); // bitmask 0-2 bits; not in this library version (2.3) yet
  rightEncoder();
  sensorR.handleA();
}
void doB_R() {
  //Serial.println("doB");
  rightEncoder();
  sensorR.handleB();
}
void doC_R() {
  //Serial.println("doC");
  rightEncoder();
  sensorR.handleC();
}
