#include <Servo.h>
//#include <SoftwareSerial.h>

/*

http://arduino.cc/en/Tutorial/ReadASCIIString

   Reading a serial ASCII-encoded string.
  
  This sketch demonstrates the Serial parseInt() function.
  It looks for an ASCII string of comma-separated values.
  It parses them into ints, and uses those to fade an RGB LED.
  
  Circuit: Common-anode RGB LED wired like so:
  * Red cathode: digital pin 3
  * Green cathode: digital pin 5
  * blue cathode: digital pin 6
  * anode: +5V

 The circuit: 
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
  
  created 13 Apr 2012
  by Tom Igoe
  
  This example code is in the public domain.
  */

// standard LED
 const int led = 13;

// servo pin
 const int servoPin = 8;
 
// pins for the LEDs:
 const int led1 = 9;
 const int led2 = 10;
 const int led3 = 11;
 const int led4 = 12;
 
 Servo myservo;  // create servo object to control a servo; maximum of eight servo objects can be created 
 
 int servoPosLast = 0;    // variable to store the servo position 
 
 float servoFactor = 0.17; // sensor comes as 0...1024,  servo accepts 0...180

 int buttonState1;            // the current reading
 int buttonState2; 
 int buttonState3;
 int buttonState4;
 
 bool lastBt34 = false;
 int lastButtonState4 = 0;

 //SoftwareSerial mySerial(3, 2); // RX, TX

void setup() {
   // initialize serial:
   Serial.begin(57600);

   // set the data rate for the SoftwareSerial port
   //mySerial.begin(9600);
   
   // make the pins outputs:
   pinMode(led, OUTPUT); 
   digitalWrite(led, 1);    // set it up for now, will be reset on normal operation
   
   pinMode(led1, OUTPUT); 
   pinMode(led2, OUTPUT); 
   pinMode(led3, OUTPUT); 
   pinMode(led4, OUTPUT); 

   myservo.attach(servoPin);  // attaches the servo pin to the servo object 
}
 
void loop() {

  /*
    while(true)
    {
      for(servoPosLast = 30; servoPosLast < 150; servoPosLast += 1)  // goes from 0 degrees to 180 degrees 
      {                                  // in steps of 1 degree 
        myservo.write(servoPosLast);     // tell servo to go to position in variable 'servoPosLast' 
        delay(25);                       // waits 15ms for the servo to reach the position 
      } 
      delay(2000); // wait for a second
      for(servoPosLast = 150; servoPosLast>=30; servoPosLast-=1)     // goes from 180 degrees to 0 degrees 
      {                                
        myservo.write(servoPosLast);     // tell servo to go to position in variable 'servoPosLast' 
        delay(25);                       // waits 15ms for the servo to reach the position 
      } 
      delay(2000); // wait for a second

      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(led1, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(led2, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(500);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      digitalWrite(led1, LOW);    // turn the LED off by making the voltage LOW
      digitalWrite(led2, LOW);    // turn the LED off by making the voltage LOW
      digitalWrite(led3, LOW);    // turn the LED off by making the voltage LOW
      digitalWrite(led4, LOW);    // turn the LED off by making the voltage LOW
      Serial.print(".");
      delay(500);               // wait for a second
    }
    
   while (mySerial.available() > 0)
   {
       // look for the asterisk *. That's the beginning of your sentence:
       if (mySerial.read() != '*')
       {
         continue;
       }

       // look for the next three valid integers in the incoming serial stream:
       int sensorValue = mySerial.parseInt(); 
       int buttonMask = mySerial.parseInt(); 
       int checksum = mySerial.parseInt(); 
  
  */
  
  
   // if there's any serial available, read it:
   while (Serial.available() > 0)
   {
       // look for the asterisk *. That's the beginning of your sentence:
       if (Serial.read() != '*')
       {
         continue;
       }

       // look for the next three valid integers in the incoming serial stream:
       int sensorValue = Serial.parseInt(); 
       int buttonMask = Serial.parseInt(); 
       int checksum = Serial.parseInt(); 
  
        if(sensorValue + buttonMask + checksum != 0)
        {
          // bad transmission
           digitalWrite(led, 1);    // indicates trouble
           continue;
        }
       
        digitalWrite(led, 0);  // indicates normal operation
       
        int servoPos = constrain((int)((float)sensorValue * servoFactor), 0, 180);  // 0 to 180 degrees
       
        buttonState1 = ((buttonMask & 0x1) != 0);
        buttonState2 = ((buttonMask & 0x2) != 0);
        buttonState3 = ((buttonMask & 0x4) != 0);
        buttonState4 = ((buttonMask & 0x8) != 0);
        
        
  
        digitalWrite(led1, buttonState1);
        digitalWrite(led2, buttonState2);
        //digitalWrite(led3, buttonState3);
        //digitalWrite(led4, buttonState4);
        
        // ---------- implement on-button control for led3, led4 ----------------------------
        if(lastBt34)
        {
          digitalWrite(led3, buttonState4);
        }
        else
        {
          digitalWrite(led4, buttonState4);
        }
        
        if(buttonState4 != lastButtonState4)
        {
          lastButtonState4 = buttonState4;
          if(buttonState4 == 0)
          {
            lastBt34 = !lastBt34;
          }
        }
        // ---------- end of implement on-button control for led3, led4 --------------------
         
        //servoPos = buttonState1 ? 20 : 120;
         
        if(servoPosLast != servoPos)
        {
           servoPosLast = servoPos;
           myservo.write(servoPosLast);
        }
         
        /*
        // print the three numbers in one string as hexadecimal:
        Serial.print(servoPosLast);
        Serial.print(" ");
        Serial.print(buttonState1);
        Serial.print(" ");
        Serial.print(buttonState2);
        Serial.print(" ");
        Serial.print(buttonState3);
        Serial.print(" ");
        Serial.println(buttonState4);
        */
   }
 }
 
