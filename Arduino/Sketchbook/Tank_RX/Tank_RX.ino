
//
// Old toy Abrams tank, radio replaced with this.
//

//#define TRACE

void setup() {

#ifdef TRACE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
#endif // TRACE

  initBatteryMonitor();

#ifdef TRACE
  Serial.println(F("Tank Receiver (nRF24L01)"));
#endif // TRACE

  initRf24(false,  // true = TX role, false = RX role
           false); // false uses address[0] (pipe 0) to transmit, true uses address[1] (pipe 1) to transmit. Receiver listens to either pipe.

  initActuators();
}

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0f;
bool led_on = false;
int controls[8];
unsigned long last_rx = 0L; // ms

void loop() {

  if(workRf24()) {
    
    last_rx = millis();

#ifdef TRACE
    Serial.print(controls[0]); Serial.print(" ");
    Serial.print(controls[1]); Serial.print(" ");
    Serial.print(controls[2]); Serial.print(" ");
    Serial.print(controls[3]); Serial.print(" ");
    Serial.print(controls[4]); Serial.print(" ");
    Serial.print(controls[5]); Serial.print(" ");
    Serial.print(controls[6]); Serial.print(" ");
    Serial.print(controls[7]); Serial.println("");
#endif // TRACE

    workActuators();
  }

  // to make this example readable in the serial monitor
  //delay(1000);  // slow transmissions down by 1 second

  led_on = !led_on;

  //digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);

  if(millis() - last_rx > 1000L) {
    feather_all();
  }

  monitorBattery();
}
