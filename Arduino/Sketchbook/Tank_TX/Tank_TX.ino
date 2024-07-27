
#define RED_LED_PIN 3
#define GREEN_LED_PIN 4

//#define TRACE

int controls[8];

void setup() {

#ifdef TRACE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
#endif // TRACE

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);

  // can't use LED_BUILTIN pin 13 on Uno
  //pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, LOW);

#ifdef TRACE
  Serial.println(F("Tank Transmitter (nRF24L01)"));
#endif // TRACE

  initRf24(true,  // true = TX role, false = RX role
           true); // false uses address[0] (pipe 0) to transmit, true uses address[1] (pipe 1) to transmit

}

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
float payload = 0.0f;
bool led_on = false;

void loop() {

  bool goodTx = workRf24();

  readControls(); // takes 2-3 ms

  // to make this example readable in the serial monitor
  delay(50);  // slow transmissions down

  payload += 1.0f;
  led_on = !led_on;

  //digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);

  if(goodTx) {
    digitalWrite(GREEN_LED_PIN, led_on ? HIGH : LOW);
    digitalWrite(RED_LED_PIN, LOW);
  } else {
    digitalWrite(RED_LED_PIN, led_on ? HIGH : LOW);
    digitalWrite(GREEN_LED_PIN, LOW);
  }
}
