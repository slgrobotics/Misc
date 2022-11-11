
// https://create.arduino.cc/projecthub/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-6b90eb
// https://github.com/NicoHood/PinChangeInterrupt

#include "PinChangeInterrupt.h"

// choose a valid PinChangeInterrupt pin of your Arduino board
#define PCINT_PIN 6
#define PCINT_MODE CHANGE
#define PCINT_FUNCTION rcPulseChange

volatile boolean RC_data_rdy;         // flag that new R/C data is available
unsigned long pwmTimer;               // to store pulse start time

void rcPulseChange()
{
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(PCINT_PIN));
  
  if (trigger == RISING)
  {
    pwmTimer = micros();              // record the start time of the current pulse
  }
  else if (trigger == FALLING)
  {
    // volatile!
    rcpwm = micros() - pwmTimer + RC_TRIM; // calculate the duration of the current pulse, about 4us precision at best

    if(rcpwm >= 800 && rcpwm <= 2200) // only accept valid RC values    
    {
      // volatile!
      rc_lastAvailable = millis();
      // volatile!
      RC_data_rdy = true;             // flag that new data is available
    }
  }
}

void setupRcReceiverRead()
{
  // set pins to input, have pullup to protect from open circuit noise:
  pinMode(PCINT_PIN, INPUT_PULLUP);
  //pinMode(PCINT_PIN, INPUT);

  // Attach the new PinChangeInterrupt and enable event function below
  attachPCINT(digitalPinToPCINT(PCINT_PIN), rcPulseChange, CHANGE);

  RC_data_rdy = false;
  rc_lastAvailable = 0;
}

// runs in main thread
boolean RC_avail()
{
  boolean avail = RC_data_rdy;
  RC_data_rdy = false;                          // reset the flag
  return avail;
}

void print_RCpwm()
{                             
  // display the raw RC Channel PWM Inputs
  unsigned int tmp = rcpwm;
  Serial.print(tmp);
}
