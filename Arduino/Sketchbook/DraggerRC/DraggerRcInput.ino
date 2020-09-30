
// https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb
// https://github.com/NicoHood/PinChangeInterrupt

// choose a pair of valid interrupt pins of your Arduino board
#define PCINT_PIN_LEFT 18
#define PCINT_PIN_RIGHT 19

// trim in case there's a timing discrepancy between RPi Hat and Arduino:
#define PW_TRIM 0

#define PCINT_FUNCTION_LEFT rcPulseChange_LEFT
#define PCINT_FUNCTION_RIGHT rcPulseChange_RIGHT

#define PCINT_MODE CHANGE

volatile unsigned long lastRisingUs_LEFT;  // to store the start time of each PWM pulse
volatile unsigned long lastRisingUs_RIGHT;

volatile boolean RC_data_rdy_LEFT;         // flag when RC receiver channel have received a new pulse
volatile boolean RC_data_rdy_RIGHT;

void InitRcInput()
{
  // set pins to input
  pinMode(PCINT_PIN_LEFT, INPUT_PULLUP);
  pinMode(PCINT_PIN_RIGHT, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PCINT_PIN_LEFT), PCINT_FUNCTION_LEFT, PCINT_MODE);
  attachInterrupt(digitalPinToInterrupt(PCINT_PIN_RIGHT), PCINT_FUNCTION_RIGHT, PCINT_MODE);
}

boolean RC_avail_LEFT()
{
  boolean avail = RC_data_rdy_LEFT;
  RC_data_rdy_LEFT = false; // reset the flag
  return avail;
}

boolean RC_avail_RIGHT()
{
  boolean avail = RC_data_rdy_RIGHT;
  RC_data_rdy_RIGHT = false; // reset the flag
  return avail;
}

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
    PW_LEFT = pciTime - lastRisingUs_LEFT + PW_TRIM; // calculate the duration of the current pulse
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
    PW_RIGHT = pciTime - lastRisingUs_RIGHT + PW_TRIM; // calculate the duration of the current pulse
    RC_data_rdy_RIGHT = true;
  }
}
