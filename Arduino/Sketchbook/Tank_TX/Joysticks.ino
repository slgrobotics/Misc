#define VAL_MAX 1020

void readControls()
{
  // takes 2-3 ms
  unsigned long t0 = micros();

  controls[0] = analogRead(A0); // right stick H: l - 813 r - 178 n - 500
  controls[1] = analogRead(A1); // right stick V: u - 870 d - 232 n - 545
  controls[2] = analogRead(A2); // left  stick V: u - 853 d - 206 n - 508
  controls[3] = analogRead(A3); // left  stick H: l - 825 r - 183 n - 521
  controls[4] = analogRead(A4) < 512 ? 1 : 0; // left   bumper: pressed - 0  released - 1023
  controls[5] = analogRead(A5) < 512 ? 1 : 0; // right  bumper: pressed - 0  released - 1023
  controls[6] = analogRead(A6); // right trigger: pressed - 153 released - 810
  controls[7] = analogRead(A7); // left  trigger: pressed - 184 released - 880

  controls[0] = constrain(map(controls[0], 813, 178, -VAL_MAX, VAL_MAX), -VAL_MAX, VAL_MAX);
  controls[1] = constrain(map(controls[1], 878, 242, VAL_MAX, -VAL_MAX), -VAL_MAX, VAL_MAX);
  controls[2] = constrain(map(controls[2], 853, 206, VAL_MAX, -VAL_MAX), -VAL_MAX, VAL_MAX);
  controls[3] = constrain(map(controls[3], 815, 160, -VAL_MAX, VAL_MAX), -VAL_MAX, VAL_MAX);

  controls[6] = constrain(map(controls[6], 810, 153, 0, VAL_MAX), 0, VAL_MAX);
  controls[7] = constrain(map(controls[7], 880, 184, 0, VAL_MAX), 0, VAL_MAX);

#ifdef TRACE
  Serial.print(controls[0]); Serial.print(" ");
  Serial.print(controls[1]); Serial.print(" ");
  Serial.print(controls[2]); Serial.print(" ");
  Serial.print(controls[3]); Serial.print(" ");
  Serial.print(controls[4]); Serial.print(" ");
  Serial.print(controls[5]); Serial.print(" ");
  Serial.print(controls[6]); Serial.print(" ");
  Serial.print(controls[7]); Serial.print("   t: ");
  Serial.print(micros() - t0);  Serial.println(" us");
#endif // TRACE

}