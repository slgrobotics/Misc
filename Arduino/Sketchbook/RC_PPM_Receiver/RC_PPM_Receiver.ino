#include <PulsePosition.h>
#include <Wire.h>

// this is part of the PX4 Lawnmower project https://github.com/slgrobotics/PX4-Autopilot
// Teensy 3.2 runs RC_PPM_Receiver code

// take PPM signal from Ch1 of the FS-IA10B receiver via pin 10
// decode 8 channels and store values
// make values available to Master computer via I2C

PulsePositionInput myIn;

#define numChannels 8

int count = 0;
unsigned short values[numChannels + 1]; // one word reserved for time since last signal received, ms
unsigned long lastRcReceivedMs;

void setup() {
  myIn.begin(10);   // Pin 10 connected to Ch1 of the receiver in PPM mode
                    // Valid pins for Teensy 3.2, 3.1:  5, 6, 9, 10, 20, 21, 22, 23

  Wire.begin(0x48);              // join i2c bus with address #48. Use "sudo i2cdetect -y 1"
  Wire.onRequest(requestEvent);  // register event

  lastRcReceivedMs = millis();

  for(int i=0; i < numChannels; i++) {
    values[i] = 0;
  }
  values[numChannels] = 65535;  // flag it as data unavailable yet
}

void loop() {

  unsigned long nowMs = millis();

  // Every time new PPM data arrives, simply store it.
  // For debugging, print it to the Arduino Serial Monitor.
  int num = myIn.available();
  
  if (num > 0) {
    //Serial.print(sizeof(values));
    //Serial.print(" | ");
    count = 0;
    for (int i = 1; i <= num; i++) {
      float val = myIn.read(i); // channel number 1...
      //Serial.print(val);
      //Serial.print("  ");
      if (count < numChannels) {
        values[count] = (unsigned short)val;
      }
      count = count + 1;
    }
    lastRcReceivedMs = nowMs;
    //Serial.println();
  } else {
    delay(1);
  }

  // note: with transmitter off or out of range, the receiver in
  //       default mode will still send valid data.
  
  // fill the last slot with ms since R/C signal last received, max 1 minute:
  unsigned long since = nowMs - lastRcReceivedMs;
  values[numChannels] = since > 60000 ? 60000 : (unsigned short)since;
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  //Wire.write("hello "); // respond with message of 6 bytes
  // as expected by master
  Wire.write((byte*)values, sizeof(values));  // 16 bytes total
}
