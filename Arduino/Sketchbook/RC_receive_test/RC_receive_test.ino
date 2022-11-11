
// https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb
// https://github.com/NicoHood/PinChangeInterrupt

#include <digitalWriteFast.h>

// choose a pair of valid interrupt pins of your Arduino board
#define PCINT_PIN_LEFT 18
#define PCINT_FUNCTION_LEFT rcPulseChange_LEFT
#define PCINT_PIN_RIGHT 19
#define PCINT_FUNCTION_RIGHT rcPulseChange_RIGHT

#define PCINT_MODE CHANGE

void setup()
{
  setup_pwmRead();
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

volatile int PW_LEFT;                      // pulse width measurements, microseconds
volatile unsigned long lastRisingUs_LEFT;  // to store the start time of each PWM pulse
volatile int PW_RIGHT;
volatile unsigned long lastRisingUs_RIGHT;

volatile boolean RC_data_rdy_LEFT;         // flag when RC receiver channel have received a new pulse
volatile boolean RC_data_rdy_RIGHT;

void rcPulseChange_LEFT()
{
  unsigned long pciTime = micros();        // Record the time of the PIN change in microseconds

  //DbgLed(HIGH);

  if (digitalReadFast(PCINT_PIN_LEFT))
  {
    lastRisingUs_LEFT = pciTime;           // record the start time of the current pulse
  }
  else
  {
    PW_LEFT = pciTime - lastRisingUs_LEFT; // calculate the duration of the current pulse
    RC_data_rdy_LEFT = true;
  }
}

void rcPulseChange_RIGHT()
{
  unsigned long pciTime = micros();        // Record the time of the PIN change in microseconds

  //DbgLed(HIGH);

  if (digitalReadFast(PCINT_PIN_RIGHT))
  {
    lastRisingUs_RIGHT = pciTime;           // record the start time of the current pulse
  }
  else
  {
    PW_RIGHT = pciTime - lastRisingUs_RIGHT; // calculate the duration of the current pulse
    RC_data_rdy_RIGHT = true;
  }
}

void setup_pwmRead()
{
  // set pins to input
  pinMode(PCINT_PIN_LEFT, INPUT_PULLUP);
  pinMode(PCINT_PIN_RIGHT, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PCINT_PIN_LEFT), PCINT_FUNCTION_LEFT, PCINT_MODE);
  attachInterrupt(digitalPinToInterrupt(PCINT_PIN_RIGHT), PCINT_FUNCTION_RIGHT, PCINT_MODE);
}

boolean RC_avail()
{
  // we assume that both channels are live more or less together,
  // so there are no stale values
  
  boolean avail = RC_data_rdy_LEFT || RC_data_rdy_RIGHT;
  RC_data_rdy_LEFT = RC_data_rdy_RIGHT = false; // reset the flags
  return avail;
}

void print_RCpwm()
{                             
  // display the raw RC Channel PWM Inputs
  Serial.print(PW_LEFT);
  Serial.print("   ");
  Serial.println(PW_RIGHT);
}

void loop()
{

  //DbgLed(HIGH);

  /*
    int ch2 = pulseIn(PCINT_PIN_LEFT, HIGH, 25000);
    if(ch2 > 0)
    {
    Serial.println(ch2);
    delay(100);
    }
  */

  if (RC_avail())
  {
    print_RCpwm();
  }
}

void DbgLed(bool on)
{
  digitalWriteFast(LED_BUILTIN, on);
}
