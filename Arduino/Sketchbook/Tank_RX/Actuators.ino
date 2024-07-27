
// Track motors:
#define LF_PIN 6
#define LB_PIN 3
#define RF_PIN 9
#define RB_PIN 5

// Tower action:
#define L_TWR_PIN A2
#define R_TWR_PIN 4
#define L_MG_PIN 10
#define R_MG_PIN A1

// TBD functions:
#define FUNC_1_PIN A3
#define FUNC_2_PIN A4
#define GUN_SHOOT_PIN A5

void initActuators()
{
  pinMode(LF_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(RF_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);

  pinMode(L_TWR_PIN, OUTPUT);
  pinMode(R_TWR_PIN, OUTPUT);
  pinMode(L_MG_PIN, OUTPUT);
  pinMode(R_MG_PIN, OUTPUT);

  pinMode(FUNC_1_PIN, OUTPUT);
  pinMode(FUNC_2_PIN, OUTPUT);
  pinMode(GUN_SHOOT_PIN, OUTPUT);

  feather_all();
}

void feather_all()
{
  // stop/feather all actions, idle everything.
#ifdef TRACE
  Serial.println("feather_all()");
#endif // TRACE
  digitalWrite(LF_PIN, LOW);
  digitalWrite(LB_PIN, LOW);
  digitalWrite(RF_PIN, LOW);
  digitalWrite(RB_PIN, LOW);

  digitalWrite(L_TWR_PIN, LOW);
  digitalWrite(R_TWR_PIN, LOW);
  digitalWrite(L_MG_PIN, LOW);
  digitalWrite(R_MG_PIN, LOW);

  digitalWrite(FUNC_1_PIN, LOW);
  digitalWrite(FUNC_2_PIN, LOW);
  digitalWrite(GUN_SHOOT_PIN, LOW);
}

#define DEAD_ZONE 30

void workActuators()
{
  // Track driving motors.
  // received values go from -1020 to 1020, analogWrite values from 0 to 255

  int left_sp = controls[2] / 4;
  int right_sp = controls[1] / 4;

  if(abs(left_sp) < DEAD_ZONE) {
    digitalWrite(LF_PIN, LOW);
    digitalWrite(LB_PIN, LOW);
  } else if(left_sp > 0) {
    analogWrite(LF_PIN, left_sp);
    digitalWrite(LB_PIN, LOW);
  } else {
    digitalWrite(LF_PIN, LOW);
    analogWrite(LB_PIN, -left_sp);
  }

  if(abs(right_sp) < DEAD_ZONE) {
    digitalWrite(RF_PIN, LOW);
    digitalWrite(RB_PIN, LOW);
  } else if(right_sp > 0) {
    analogWrite(RF_PIN, right_sp);
    digitalWrite(RB_PIN, LOW);
  } else {
    digitalWrite(RF_PIN, LOW);
    analogWrite(RB_PIN, -right_sp);
  }

  // Tower turn controls - left and right bumpers.
  bool twr_left = controls[4] > 0;
  bool twr_right = controls[5] > 0;
  
  digitalWrite(L_TWR_PIN, twr_left ? HIGH : LOW);
  digitalWrite(R_TWR_PIN, twr_right ? HIGH : LOW);

  // Commander turn and machine gun controls - right stick horizontal or left trigger.
  bool mg_left = controls[0] < -500 || controls[7] > 500 ;
  bool mg_right = controls[0] > 500;
  
  digitalWrite(L_MG_PIN, mg_left ? HIGH : LOW);
  digitalWrite(R_MG_PIN, mg_right ? HIGH : LOW);

  // Gun shoot - right trigger
  digitalWrite(GUN_SHOOT_PIN, controls[6] > 500 ? HIGH : LOW);

  // No idea where these two are connected:
  //digitalWrite(FUNC_1_PIN, controls[4] > 0 ? HIGH : LOW);
  //digitalWrite(FUNC_2_PIN, controls[5] > 0 ? HIGH : LOW);
}
