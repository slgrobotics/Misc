
// Raspberry Pi Pico blink test

int pinLED = 25;

void setup() {
  // put your setup code here, to run once:
  pinMode(pinLED, OUTPUT);
}

void loop() {
  digitalWrite(pinLED, HIGH);
  delay(50);
  digitalWrite(pinLED, LOW);
  delay(50);
}
