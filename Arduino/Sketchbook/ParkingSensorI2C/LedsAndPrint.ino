
int redTreshold = 40;     // cm
int yellowTreshold = 80;

void setLeds()
{
  digitalWriteFast(ledFRYPin, LOW);
  digitalWriteFast(ledFRRPin, LOW);
  digitalWriteFast(ledFLYPin, LOW);
  digitalWriteFast(ledFLRPin, LOW);
  digitalWriteFast(ledBRYPin, LOW);
  digitalWriteFast(ledBRRPin, LOW);
  digitalWriteFast(ledBLYPin, LOW);
  digitalWriteFast(ledBLRPin, LOW);

  if(rangeFRcm >= 0)
  {
    if(rangeFRcm < redTreshold)
    {
      digitalWriteFast(ledFRRPin, HIGH);
    }
    else if(rangeFRcm < yellowTreshold)
    {
      digitalWriteFast(ledFRYPin, HIGH);
    }
  }
  
  if(rangeFLcm >= 0)
  {
    if(rangeFLcm < redTreshold)
    {
      digitalWriteFast(ledFLRPin, HIGH);
    }
    else if(rangeFLcm < yellowTreshold)
    {
      digitalWriteFast(ledFLYPin, HIGH);
    }
  }
    
  if(rangeBRcm >= 0)
  {
    if(rangeBRcm < redTreshold)
    {
      digitalWriteFast(ledBRRPin, HIGH);
    }
    else if(rangeBRcm < yellowTreshold)
    {
      digitalWriteFast(ledBRYPin, HIGH);
    }
  }
    
  if(rangeBLcm >= 0)
  {
    if(rangeBLcm < redTreshold)
    {
      digitalWriteFast(ledBLRPin, HIGH);
    }
    else if(rangeBLcm < yellowTreshold)
    {
      digitalWriteFast(ledBLYPin, HIGH);
    }
  }
}

// ===============================================================================
void test()
{
  for(int i=0; i < 5 ;i++)
  {
    testOne(ledFRYPin);
    testOne(ledFRRPin);
    testOne(ledFLYPin);
    testOne(ledFLRPin);
    
    testOne(ledBRYPin);
    testOne(ledBRRPin);
    testOne(ledBLYPin);
    testOne(ledBLRPin);
  }
}

void testOne(int led)
{
  digitalWriteFast(led, HIGH);
  digitalWriteFast(ledPin, HIGH);
  delay(100);
  digitalWriteFast(led, LOW);
  digitalWriteFast(ledPin, LOW);
  //delay(100);
}

// ===============================================================================
// nice to have a LED blinking when signal is captured OK. Good for debugging too.
void mLED_Red_On()
{
  digitalWriteFast(ledPin, HIGH);   // turn the LED on (HIGH is the voltage level)
}

void mLED_Red_Off()
{
  digitalWriteFast(ledPin, LOW);    // turn the LED off by making the voltage LOW
}

/*
// some debug related stuff:
void mLED_Dbg_On()
{
 digitalWriteFast(dbgPin, HIGH);   // turn the LED on (HIGH is the voltage level)
}
 
void mLED_Dbg_Off()
{
 digitalWriteFast(dbgPin, LOW);    // turn the LED off by making the voltage LOW
}
*/

#ifdef DO_SERIAL_OUTPUT

void printValues() 
{
  Serial.print("!SNR:");
  Serial.print(rangeFRcm);  // front right
  Serial.print(",");
  Serial.print(rangeFLcm);  // front left
  Serial.print(",");
  Serial.print(rangeBLcm);  // back left
  Serial.print(",");
  Serial.print(rangeBRcm);  // back right
  /*     Serial.print("\t");
   Serial.print(psiData[4]);  // raw byte
   Serial.print("\t");
   Serial.print(psiData[5]);  // raw byte
   Serial.print("\t");
   Serial.print(psiData[6]);  // raw byte
   Serial.print("\t");
   Serial.print(psiData[7]);  // raw byte
   */
  Serial.print("\n");
  /*
   Serial.print(psiRawData[0]);
   Serial.print("***");
   Serial.print(psiRawData[1]);
   Serial.print("***");
   Serial.print(psiRawData[2]);
   Serial.print("***");
   Serial.print(psiRawData[3]);
   Serial.print("\n");
   */
}

#endif // DO_SERIAL_OUTPUT


