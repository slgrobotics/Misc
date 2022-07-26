// See https://randomnerdtutorials.com/esp8266-pwm-arduino-ide/

// NodeMCU has weird pin mapping.
// Pin numbers written on the board itself do not correspond to ESP8266 GPIO pin numbers.

#define D0 16 // Connected to LED. GPIO16 does support PWM.
#define D1 5  // I2C Bus SCL (clock)
#define D2 4  // I2C Bus SDA (data)
#define D3 0
#define D4 2  // Same as "LED_BUILTIN", but inverted logic
#define D5 14 // SPI Bus SCK (clock)
#define D6 12 // SPI Bus MISO 
#define D7 13 // SPI Bus MOSI
#define D8 15 // SPI Bus SS (CS) - use D0 instead
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

#define SD1 8   // SDD1
#define SD0 7   // SDD0, MISO
#define SD2 9   // SDD2 - do not use!
#define SD3 10  // SDD3

#define SDCLK 6  // SDCLK, CLK
#define SDCMD 11 // SDCMD, CMD

// See https://www.electronicwings.com/nodemcu/nodemcu-spi-with-arduino-ide
// Note: D8/HCS - boot fails if it is pulled high, see https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
// If you want to use NodeMCU pin 5, use D5 for pin number, and it will be translated to 'real' GPIO pin 14.

// Built in LED:
int pinLED = D0;  // HiLetgo, NodeMCU D0 
//int pinLED = 0;   // Adafruit Feather HUZZAH

const int interruptPin1 = D4;
const int interruptPin2 = D5;

// must have to avoid crashing:
void ICACHE_RAM_ATTR leftEncoder();
void ICACHE_RAM_ATTR rightEncoder();

void setup() 
{
  Serial.begin(9600);   // start serial 

  delay(1000);

  
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);

  // The ESP8266 supports interrupts in any GPIO, except GPIO16 (D0).
  pinMode(interruptPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), leftEncoder, FALLING);   // FALLING, RISING, CHANGE

  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), rightEncoder, FALLING);   // FALLING, RISING, CHANGE
}

void loop() 
{
  delay(100);
}

void leftEncoder()
{
  digitalWrite(pinLED, LOW);
}

void rightEncoder()
{
  digitalWrite(pinLED, HIGH);
}
