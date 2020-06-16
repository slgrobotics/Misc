
// https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb
// https://github.com/NicoHood/PinChangeInterrupt

#include "PinChangeInterrupt.h"

// choose a valid PinChangeInterrupt pin of your Arduino board
#define PCINT_PIN 6
#define PCINT_MODE CHANGE
#define PCINT_FUNCTION rcPulseChange

void setup()
{
  setup_pwmRead();
  Serial.begin(115200);
}

volatile int PW;                  // pulsewidth measurements
volatile boolean prev_pinState;   // used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;   // the time of the current pin change interrupt
volatile unsigned long pwmTimer;  // to store the start time of each PWM pulse

volatile boolean pwmFlag;         // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;     // flag when all RC receiver channels have received a new pulse
unsigned long pwmPeriod;          // period, mirco sec, between two pulses on each pin

void rcPulseChange()
{
  pciTime = micros();                                   // Record the time of the PIN change in microseconds

  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(PCINT_PIN));
  if (trigger == RISING)
  {
    prev_pinState = 1;                                  // record pin state
    pwmPeriod = pciTime - pwmTimer;                     // calculate the time period, micro sec, between the current and previous pulse
    pwmTimer = pciTime;                                 // record the start time of the current pulse
  }
  else if (trigger == FALLING)
  {
    prev_pinState = 0;                                  // record pin state
    PW = pciTime - pwmTimer;                            // calculate the duration of the current pulse
    pwmFlag = HIGH;                                     // flag that new data is available
    RC_data_rdy = HIGH;
  }
}

void setup_pwmRead()
{
  // set pins to input
  pinMode(PCINT_PIN, INPUT);
  //pinMode(LED_BUILTIN, OUTPUT);

  // Attach the new PinChangeInterrupt and enable event function below
  attachPCINT(digitalPinToPCINT(PCINT_PIN), rcPulseChange, CHANGE);
}

boolean RC_avail()
{
  boolean avail = RC_data_rdy;
  RC_data_rdy = LOW;                          // reset the flag
  return avail;
}

void print_RCpwm()
{                             
  // display the raw RC Channel PWM Inputs
  Serial.println(PW);
}

void loop()
{

  /*
    int ch2 = pulseIn(6, HIGH, 25000);
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
