
//#define DO_FULL_OUTPUT
#define DO_UPLINK_OUTPUT
//#define DO_TEST_OUTPUT

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

/*
   This sample code demonstrates just about every built-in operation of TinyGPS++ (TinyGPSPlus).
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 8(rx) and 3(tx).
   Not all pins on the Leonardo and Micro support change interrupts, so only the following 
   can be used for RX: 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
*/
static const int RXPin = 8, TXPin = 3;
static const uint32_t GPSBaud = 9600;
static const uint32_t UplinkBaud = 57600;

static const int GreenLedPin = 2, BlueLedPin = 5, YellowLedPin = 7, RedLedPin = 6, statusLedPin = 13; 


// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the uplink
SoftwareSerial ss(RXPin, TXPin);     // RX, TX

// For stats that happen every 5 seconds
unsigned long last = 0UL;

void setup()
{
#ifdef DO_FULL_OUTPUT
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only - Leonardo
  }
#endif // DO_FULL_OUTPUT

  Serial1.begin(GPSBaud); // connect GPS to pins 19 (RX) and 18 (TX)
  ss.begin(UplinkBaud);

#ifdef DO_FULL_OUTPUT
  Serial.println(F("KitchenSink.ino"));
  Serial.println(F("Demonstrating nearly every feature of TinyGPS++"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
#endif // DO_FULL_OUTPUT

  pinMode (statusLedPin, OUTPUT);  // Status LED
  pinMode (GreenLedPin, OUTPUT);
  pinMode (BlueLedPin, OUTPUT);
  pinMode (YellowLedPin, OUTPUT);
  pinMode (RedLedPin, OUTPUT);
}

String lastLastSentence = "";

void loop()
{
  // Dispatch incoming characters
  while (Serial1.available() > 0)
  {
    if(gps.encode(Serial1.read()))
    {
      digitalWrite(BlueLedPin, HIGH);
    }
  }

#ifdef DO_TEST_OUTPUT

  if (millis() - last > 1000)
  {
    last = millis();
    digitalWrite(BlueLedPin, HIGH);
    ss.println("Hello from GPS over SoftwareSerial");
  }
  
#endif // DO_TEST_OUTPUT

#ifdef DO_UPLINK_OUTPUT

  if (millis() - last > 3000)
  {
    last = millis();
    digitalWrite(BlueLedPin, LOW);
  }
  
  if(lastLastSentence != gps.lastSentence)
  {
    //ss.print(F("=== "));
    //ss.println(gps.lastSentence);
    lastLastSentence = gps.lastSentence;

    if(lastLastSentence == "GPGLL")
    {
      digitalWrite(GreenLedPin, LOW);
      digitalWrite(YellowLedPin, LOW);
      digitalWrite(RedLedPin, LOW);

      if (gps.satellites.isUpdated())
      {
        ss.print(F("FIX "));
        ss.println(gps.fixQuality);
        ss.print(F("SAT "));
        ss.println(gps.satellites.value());

        switch(gps.fixQuality)
        {
          case 0:
            digitalWrite(RedLedPin, HIGH);
            break;
          case 1:
            digitalWrite(YellowLedPin, HIGH);
            break;
          case 2:
            digitalWrite(GreenLedPin, HIGH);
            break;
        }
      }

      if (gps.hdop.isUpdated())
      {
        ss.print(F("HDP "));
        ss.println(gps.hdop.value());
      }

      if (gps.location.isUpdated())
      {
        ss.print(F("LOC "));
        ss.print(gps.location.rawLat().negative ? "-" : "");
        ss.print(gps.location.rawLat().deg);
        ss.print(".");
        ss.print(gps.location.rawLat().billionths);
        ss.print(F(" "));
        ss.print(gps.location.rawLng().negative ? "-" : "");
        ss.print(gps.location.rawLng().deg);
        ss.print(".");
        ss.println(gps.location.rawLng().billionths);
      }
    }
  }
  
#endif // DO_UPLINK_OUTPUT

#ifdef DO_FULL_OUTPUT

  if(lastLastSentence != gps.lastSentence)
  {
    Serial.print(F("=== "));
    Serial.println(gps.lastSentence);
    lastLastSentence = gps.lastSentence;
  }
  
  if (gps.location.isUpdated())
  {
    Serial.print(F("LOCATION   Fix Age="));
    Serial.print(gps.location.age());
    Serial.print(F("ms Raw Lat="));
    Serial.print(gps.location.rawLat().negative ? "-" : "+");
    Serial.print(gps.location.rawLat().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLat().billionths);
    Serial.print(F(" billionths],  Raw Long="));
    Serial.print(gps.location.rawLng().negative ? "-" : "+");
    Serial.print(gps.location.rawLng().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLng().billionths);
    Serial.print(F(" billionths],  Lat="));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" Long="));
    Serial.println(gps.location.lng(), 6);
  }

  else if (gps.date.isUpdated())
  {
    Serial.print(F("DATE       Fix Age="));
    Serial.print(gps.date.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.date.value());
    Serial.print(F(" Year="));
    Serial.print(gps.date.year());
    Serial.print(F(" Month="));
    Serial.print(gps.date.month());
    Serial.print(F(" Day="));
    Serial.println(gps.date.day());
  }

  else if (gps.time.isUpdated())
  {
    Serial.print(F("TIME       Fix Age="));
    Serial.print(gps.time.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.time.value());
    Serial.print(F(" Hour="));
    Serial.print(gps.time.hour());
    Serial.print(F(" Minute="));
    Serial.print(gps.time.minute());
    Serial.print(F(" Second="));
    Serial.print(gps.time.second());
    Serial.print(F(" Hundredths="));
    Serial.println(gps.time.centisecond());
  }

  else if (gps.speed.isUpdated())
  {
    Serial.print(F("SPEED      Fix Age="));
    Serial.print(gps.speed.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.speed.value());
    Serial.print(F(" Knots="));
    Serial.print(gps.speed.knots());
    Serial.print(F(" MPH="));
    Serial.print(gps.speed.mph());
    Serial.print(F(" m/s="));
    Serial.print(gps.speed.mps());
    Serial.print(F(" km/h="));
    Serial.println(gps.speed.kmph());
  }

  else if (gps.course.isUpdated())
  {
    Serial.print(F("COURSE     Fix Age="));
    Serial.print(gps.course.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.course.value());
    Serial.print(F(" Deg="));
    Serial.println(gps.course.deg());
  }

  else if (gps.altitude.isUpdated())
  {
    Serial.print(F("ALTITUDE   Fix Age="));
    Serial.print(gps.altitude.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.altitude.value());
    Serial.print(F(" Meters="));
    Serial.print(gps.altitude.meters());
    Serial.print(F(" Miles="));
    Serial.print(gps.altitude.miles());
    Serial.print(F(" KM="));
    Serial.print(gps.altitude.kilometers());
    Serial.print(F(" Feet="));
    Serial.println(gps.altitude.feet());
  }

  else if (gps.satellites.isUpdated())
  {
    Serial.print(F("\r\nFIX="));
    Serial.println(gps.fixQuality);
    Serial.print(F("SATELLITES Fix Age="));
    Serial.print(gps.satellites.age());
    Serial.print(F("ms Value="));
    Serial.println(gps.satellites.value());
  }

  else if (gps.hdop.isUpdated())
  {
    Serial.print(F("HDOP       Fix Age="));
    Serial.print(gps.hdop.age());
    Serial.print(F("ms Value="));
    Serial.println(gps.hdop.value());
  }

  else if (millis() - last > 5000)
  {
    Serial.println();
    if (gps.location.isValid())
    {
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      double distanceToLondon =
        TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
      double courseToLondon =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);

      Serial.print(F("LONDON     Distance="));
      Serial.print(distanceToLondon/1000, 6);
      Serial.print(F(" km Course-to="));
      Serial.print(courseToLondon, 6);
      Serial.print(F(" degrees ["));
      Serial.print(TinyGPSPlus::cardinal(courseToLondon));
      Serial.println(F("]"));
    }

    Serial.print(F("DIAGS      Chars="));
    Serial.print(gps.charsProcessed());
    Serial.print(F(" Sentences-with-Fix="));
    Serial.print(gps.sentencesWithFix());
    Serial.print(F(" Failed-checksum="));
    Serial.print(gps.failedChecksum());
    Serial.print(F(" Passed-checksum="));
    Serial.println(gps.passedChecksum());

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

    last = millis();
    Serial.println();
  }
#endif // DO_FULL_OUTPUT
}
