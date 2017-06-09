/* Parking Car Assist Ultrasound Sensor
 *------------------
 *
 * Reads values from a 4-sensor ultrasound unit (sold widely for around US $20 on Amazon)
 * and writes the values to the serial port.
 *
 * Copyright (c) 2012...2016..., Sergei Grichine   http://trackroamer.com, https://github.com/slgrobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *    
 * this is a no-warranty no-liability permissive license - you do not have to publish your changes,
 * although doing so, donating and contributing code and feedback is always appreciated
 * https://www.paypal.me/vitalbytes
 */
 

// Variables needed for state machine, time measurement, data accumulation, conversion.
// they are used only by ISR, therefore volatile is not needed:
unsigned long lastOnChangeMs = 0L;

byte psiState = 0;  // car parking sensor interpreter (state machine) state:
                        // 0 = looking for sync pulse 
                        // 1 = in the sync pulse (2ms high at the beginning of 40ms sequence)
                        // 2 = nded sync pulse
                        // 3 = reading the sequence bits (bit whacking)
                        // 4 = converting raw data into 4 bytes, preparing 
unsigned int psiIndex = 0;

#define maxPsiBytes 80    // real number of bits is 32 for 4 sensors (64 for 8 ?).
byte psiBytes[maxPsiBytes]; // raw bits' durations (in some units) go into this array - usually either "25" (1) or "50" (0).

// this is where we copy collected raw data:
unsigned int psiCount;
byte psiRawData[maxPsiBytes]; // at the end of the packet all psiBytes[] is copied here, for multitask o pass it to the PC
unsigned long usLastOnChangeDown = 0;
boolean isReversed;    // the signal is connected in a way that reverses logical level, i.e. through opto isolation chip.

void interrup_isr_onchange(void)
{
  //cli(); // disable global interrupts

  // we enter here about 8-9 us after the edge (signal change).
  unsigned long nowMs;
  unsigned long timespanMs;

  mLED_Red_Off();

  nowMs = millis();

  timespanMs = nowMs - lastOnChangeMs;
  lastOnChangeMs = nowMs;

  byte parkingSensorValue = digitalReadFast(mParkingSensorPin);

  if(timespanMs > BETWEEN_PACKETS_MS)
  {
    //mLED_Dbg_On();
    //mLED_Dbg_Off();

    isReversed = parkingSensorValue == 0;

    // 20+ ms since last change means we've likely got a sync pulse (which will be 2ms long high).
    // for PARKINGSENSOR_MODEL_1 actual interval between packets is 250 ms - much more than 20 anyway.
    psiState = (byte)1;
  }

  // the signal could be connected in a way that reverses logical level, i.e. through opto isolation chip.
  boolean isSignalHigh = isReversed ? (parkingSensorValue == 0) : (parkingSensorValue == 1);

  // see which state we are in and work the state machine to read sequence bits:
  switch (psiState)
  {
  case 0:   // in between pulses, ignoring noise, waiting for a long (>30ms) low on sensor pin.
    break;

  case 1:   // ended sync pulse?
    if(!isSignalHigh)
    {
      psiState = (byte)2;
    }
    break;

  case 2:   // ended sync pulse?
    if(isSignalHigh)
    {
      psiState = (byte)3;
      psiIndex = 0; // start byte array (bit whacking)
    }
    break;

  case 3:   // bit whacking - processed inside YourHighPriorityISRCode() in main.c for speed
    {
      
      // full bit pulse period is 1200us for PARKINGSENSOR_MODEL_1, 350us for PARKINGSENSOR_MODEL_2
      // Parking sensor can be connected to Arduino pin3 via a optical isolation chip, which also inverts the signal.
      // The voltage on mParkingSensorPin can be:
      //            - is normally 0V, comes to 5V when signal arrives (TTL active high logic) - in direct connection mode.
      //            - is normally 5V, comes to 0V when signal arrives (TTL active low logic) - in reverse mode.
      // we interrupt on the rising edge to reset time counter, and then measure 400us or 800us as "1" or "0" (for PARKINGSENSOR_MODEL_1), 100us or 200us for Model 2
      unsigned long usNow = micros();  // will overflow after approximately 70 minutes.

      if(!isSignalHigh)
      {
        //mLED_Red_On();
        //mLED_Red_Off();
        // on the rising edge - mark timer value
        usLastOnChangeDown = usNow;
      } 
      else {
        // on the falling edge - measure duration since last rise; scale it down to a byte:
        unsigned long usTimespan = (usNow - usLastOnChangeDown) >> 4;  // 25 for 400 us or 50 for 800 us; 8 for 100 us, 12 for 200 us

        // we expect all pulses to be between max and min values.
        if(usTimespan > PULSE_MAX || usTimespan < PULSE_MIN)  // we break if we know we skipped a beat or if timer rolled over
        {
          //mLED_Dbg_On();
          //mLED_Dbg_Off();
          psiState = (byte)0; // reset state machine
          psiIndex = 0;
        }
        else
        {
          // for PARKINGSENSOR_MODEL_1: a "1" is 400 mks, while a "0" is around 800. Push it in a byte by dividing by 16.
          // for PARKINGSENSOR_MODEL_2: a "1" is 100 mks, while a "0" is around 200. Push it in a byte by dividing by 16.
          // this way we will be getting a lot of "15" and "30" (or 6 and 12 for Model 2) and some out-of-whack numbers well above.
          // there is a lot of interrupting going on - USB and other timers, Ping CCP.
          // we may get delayed here - distorting the data 
          psiBytes[psiIndex++] = (byte)usTimespan;  // save the bit - 25 or 50; timespanMs is from the rising front of this pulse

          if(psiIndex >= (unsigned int)32)		// we reached the end of the transmission, 32 bytes - switch to state 4
          {
            psiState = (byte)4;
          }
        }
      }
    }
    break;

  case 4:     // we are almost finished - just send what's been collected - the 32 bytes in 
    // we come here on the falling edge of the last bit 

    if(psiIndex == (unsigned int)32)  // double-check just in case
    {
      //mLED_Dbg_On();
      //mLED_Dbg_Off();
      int i;
      boolean goodSample = true;
      psiCount = (unsigned int)0;   // prepare to mark it as invalid

      // when ready - copy and let main thread handle the data:
      for (i=0; (unsigned int)i < psiIndex ;i++)
      {
        // for PARKINGSENSOR_MODEL_1: we will be getting a lot of "25" and "50" and some out-of-whack numbers well above (because of lost time in interrupts).
        // for PARKINGSENSOR_MODEL_2: we will be getting a lot of "6" and "13" and some out-of-whack numbers well above (because of lost time in interrupts).
        if(psiBytes[i] > BADPULSE_DISCRIMINATOR)
        {
          goodSample = false;
          break;
        }
        psiRawData[i] = psiBytes[i];
      }
      if(goodSample)
      {
        psiCount = psiIndex;  // expect 32
        convertPsiData();
#ifdef USE_VOTING
        vote();
#endif //USE_VOTING
        psiChanged = true;
        mLED_Red_On();          // will stay on till the beginning of the next cycle
      }
    }
    psiState = (byte)0;  // we are done, reset state machine
    psiIndex = 0;
    break;

  default:  // cannot happen
    psiState = (byte)0; // reset state machine
    psiIndex = 0;
    break;
  }
  //sei(); // enable global interrupts
}

#ifdef USE_VOTING
// take 4 bytes from psiData (in decimeters) and save/combine it with rangeVotingMatrix voting out spikes.
// replace psiData with voted values
void vote(void)
{
  // for every channel
  for(int i=0; i < 4 ;i++)
  {
    // shift back two bytes - we keep a history of two previous readings
    for(int j=0; j < 2 ;j++)
    {
      rangeVotingMatrix[i][j] = rangeVotingMatrix[i][j+1];
    }
    rangeVotingMatrix[i][2] = psiData[i]; // current reading in the third position. Keep original value as history.

    // compute reading that we will report to caller:
    byte voted = psiData[i];

    if(voted <= 4)
    {
      // suspect "short distance" reading replaced, taking history into account.
      // as original reading becomes history, next short will be computed with it:
      if(rangeVotingMatrix[i][1] > 4)
      {
        // previous reading was not short. Report historical average for now:
        voted = (rangeVotingMatrix[i][0] + rangeVotingMatrix[i][1]) >> 1;
      }
    }

    psiData[i] = voted;
  }
}
#endif //USE_VOTING

// we run this conversion within the interrupt, as we don't care how much time we spend there.
// if you choose to run it in the loop, variables used by it should be declared volatile.
void convertPsiData(void)
{
  int i = 0;

  if(psiCount == (unsigned int)0)   // invalid sample (likely due to interrupts delays).
  {
    while (i < 8) // 4 psi + 4 extra
    {
      psiData[i++] = (byte)254;    // we signal this case with "254", as 255 is taken by "out of range".
    }
  }
  else
  {
    byte bt = 0;  // byte to be bitwhacked and shifted left
    int cnt = psiCount;   // must be 32
    volatile byte* pbt = psiData;

    // produce 4 bytes out of 32 numbers psiRawData[]:
    while((unsigned int)i < psiCount)
    {
      // the psiRawData contains 32 numbers around 25 ("1") and 50 ("0"). They represent bits in 4 resulting bytes (psiData[]).
      // the psiRawData contains 32 numbers around 6 ("1") and 12 ("0"). They represent bits in 4 resulting bytes (psiData[]).
      // I don't know if a 8-sensor model would have 64 bits in the sequence, will see when/if I get my hands on it.

      // use this for debugging to see what are typical values for "1" and "0"
      //Serial.print(":");
      //Serial.print(psiRawData[i]);  // we are getting "6", "7" and "12", "13" here
      
      if(psiRawData[i] < ONEZERO_DISCRIMINATOR)
      {
        // logical "1" - short pulse about 400us for Model 1, or 100us for Model 2
        bt = bt + 1;  // whack!
      }

      if((i & 0x7) == 0x7)
      {
        *pbt++ = bt;  // byte ready, save it - distance in decimeters
        bt = 0;
      }
      bt = bt << 1;
      i++;
    }
    *pbt++ = bt;  // save the last psiData byte - distance in decimeters

    // add 4 raw bytes for debugging, (these are "50" or "25"):
    for(i=0; i < 4; i++)
    {
      psiData[4+i] = psiRawData[i];
    }
    /*
    for(i=0; i < 4; i++)
    {
      psiData[4+i] = (byte)0;
    }
    */
    
    rangeFRcm = psiDataToCentimeters(psiData[0]);  // front right
    rangeFLcm = psiDataToCentimeters(psiData[1]);  // front left
    rangeBRcm = psiDataToCentimeters(psiData[3]);  // back right
    rangeBLcm = psiDataToCentimeters(psiData[2]);  // back left
  }
}

// we want output in centimeters (or change this to your favorite units).
int psiDataToCentimeters(byte dm)
{
  // roughly raw 7 is about 0.7 meters, 5 is about 0.5m...

  if(dm > 100)
  {
    return -1;  // a byte of 255 designates invalid data, beyond the range 0f 2.5 meters. 254 stands for error case.
  }

  return ((int)dm) * 10;
}

