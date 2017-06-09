/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  This example code is in the public domain.
 */
 
// Pin 13 has an LED connected on most Arduino boards. Give it a name:
const int led = 13;          // general purpose indicator
const int redLed = 8;        // low voltage indicator

const int voltagePin = A5;   // Analog input pin that the battery divider is attached to
const int analogInPin = A3;  // Analog input pin that the potentiometer is attached to

const int buttonPin1 = 9;    // the number of the pushbutton pin 1
const int buttonPin2 = 10;   // the number of the pushbutton pin 2
const int buttonPin3 = 11;   // the number of the pushbutton pin 3
const int buttonPin4 = 12;   // the number of the pushbutton pin 4

int buttonState1;            // the current reading from the input pin
int lastButtonState1 = LOW;  // the previous reading from the input pin
int buttonState2;            // the current reading from the input pin
int lastButtonState2 = LOW;  // the previous reading from the input pin
int buttonState3;            // the current reading from the input pin
int lastButtonState3 = LOW;  // the previous reading from the input pin
int buttonState4;            // the current reading from the input pin
int lastButtonState4 = LOW;  // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime1 = 0;  // the last time the output pin was toggled
long lastDebounceTime2 = 0;  // the last time the output pin was toggled
long lastDebounceTime3 = 0;  // the last time the output pin was toggled
long lastDebounceTime4 = 0;  // the last time the output pin was toggled
long debounceDelay = 50;     // the debounce time; increase if the output flickers

long lastSerialTime = 0;     // the last time the serial message was transmitted
long serialDelay = 20;      // the interval between serial messages (ms)

int batteryVoltage = 0;
long lastRedLed = 0;
boolean redLedState = false;

// the setup routine runs once when you press reset:
void setup()
{
  // warning: do not connect any voltage source to AREF! You can use AREF to power a 10K potentiometer.
  
  analogReference(INTERNAL);  // set AREF pin to 1.1V from internal source, independent of the Vcc. We can measure Vcc/Vbat now

  // initialize the LED digital pins as an output.
  pinMode(led, OUTPUT);
  pinMode(redLed, OUTPUT);
  digitalWrite(redLed, 1);

  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);
  
  Serial.begin(57600);
}

// the loop routine runs over and over again forever:
void loop()
{
  // ================ measure the battery and blink red light below 4.0V (3.3V considering 0.7V on the protecting diode) ===============
  
  if(batteryVoltage < 200)
  {
    batteryVoltage = 200;
  }
  
  if(millis() - lastRedLed > (long)((batteryVoltage-180)<<3))
  {
    lastRedLed = millis();

    batteryVoltage = analogRead(voltagePin);
    //batteryVoltage = analogRead(analogInPin);
    
    // with a 100K / 10K divider: 5V = 420;  3.81V = 331        
    //Serial.println(batteryVoltage);

    if(batteryVoltage < 300)
    {
      redLedState = !redLedState;
      digitalWrite(redLed, redLedState);
    }
    else
    {
         digitalWrite(redLed, 0);
    }
  }
  
  //return;

  // ============== end of battery watch ================================================
  
  // read the analog value:
  int sensorValue = analogRead(analogInPin);   // value read from the pot     

//----------------------------------------------------------
  int reading = digitalRead(buttonPin1);

  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState1) {
    // reset the debouncing timer
    lastDebounceTime1 = millis();
  } 
  
  if ((millis() - lastDebounceTime1) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    buttonState1 = reading;
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState1 = reading;

//----------------------------------------------------------
  reading = digitalRead(buttonPin2);

  if (reading != lastButtonState2) {
    lastDebounceTime2 = millis();
  } 
  
  if ((millis() - lastDebounceTime2) > debounceDelay) {
    buttonState2 = reading;
  }
  
  lastButtonState2 = reading;

//----------------------------------------------------------
  reading = digitalRead(buttonPin3);

  if (reading != lastButtonState3) {
    lastDebounceTime3 = millis();
  } 
  
  if ((millis() - lastDebounceTime3) > debounceDelay) {
    buttonState3 = reading;
  }

  lastButtonState3 = reading;

//----------------------------------------------------------
  reading = digitalRead(buttonPin4);

  if (reading != lastButtonState4) {
    lastDebounceTime4 = millis();
  } 
  
  if ((millis() - lastDebounceTime4) > debounceDelay) {
    buttonState4 = reading;
  }

  lastButtonState4 = reading;

//----------------------------------------------------------
  
  // set the LED using the state of the button:
  digitalWrite(led, !buttonState1 || !buttonState2 || !buttonState3 || !buttonState4);
  //digitalWrite(led, !buttonState4);

  if ((millis() - lastSerialTime) > serialDelay)
  {
    /*
    Serial.print(!buttonState1);
    Serial.print(" ");
    Serial.print(!buttonState2);
    Serial.print(" ");
    Serial.print(!buttonState3);
    Serial.print(" ");
    Serial.print(!buttonState4);
    Serial.print(" ");
    */

    // output like this:
    // *571 6 -577
    // *572 6 -578

    Serial.print("*");         // start of sentence
    Serial.print(sensorValue);
    
    int buttonMask = 0;
    if(!buttonState1)
    {
      buttonMask |= 1;
    }
    if(!buttonState2)
    {
      buttonMask |= 2;
    }
    if(!buttonState3)
    {
      buttonMask |= 4;
    }
    if(!buttonState4)
    {
      buttonMask |= 8;
    }
    
    Serial.print(" ");
    Serial.print(buttonMask);
    
    Serial.print(" ");
    Serial.println(-(buttonMask+sensorValue));  // checksum
    
    lastSerialTime = millis();
  }
}

