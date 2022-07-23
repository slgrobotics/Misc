
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
#define D8 15 // SPI Bus SS (CS)
#define D9 3  // RX0 (Serial console)
#define D10 1 // TX0 (Serial console)

//If you want to use NodeMCU pin 5, use D5 for pin number, and it will be translated to 'real' GPIO pin 14.

// Built in LED:
int pinLED = 16;  // HiLetgo 
//int pinLED = 0;   // Adafruit Feather HUZZAH 

int rightPwmPin = D6;
int rightDirPin = D8;

int leftPwmPin = D7;
int leftDirPin = D5;

int stopPin = D4;
int brakePin = D3;

int delayMs = 20;
bool dir = true;  // true - forward
int power = 0;    // max 255

void setup()
{
  pinMode(rightPwmPin, OUTPUT);
  pinMode(rightDirPin, OUTPUT);
  pinMode(leftPwmPin, OUTPUT);
  pinMode(leftDirPin, OUTPUT);
  pinMode(stopPin, OUTPUT);
  pinMode(brakePin, OUTPUT);

  digitalWrite(stopPin, LOW);
  digitalWrite(brakePin, LOW);
}

// Generated PWM frequency will be 1.00 kHz - quite precise.
// ESP devices are 3.3V levels

void loop()
{
  for(; power < 255; power++){   
    motorCycle(dir, power);
  }

  for(; power > 0; power--){
    motorCycle(dir, power);
  }

  dir = dir ? false : true;

  //digitalWrite(stopPin, HIGH);
  //digitalWrite(brakePin, HIGH);
  
}

void motorCycle(bool dir, int power)
{
  digitalWrite(rightDirPin, dir ? LOW : HIGH);
  digitalWrite(leftDirPin, dir ? HIGH : LOW);
  analogWrite(rightPwmPin, power);
  analogWrite(leftPwmPin, power);
  delay(delayMs);
}
