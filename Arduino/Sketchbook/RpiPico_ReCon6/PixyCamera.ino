// Reading PIXY -> Arduino Leonardo (PixyToSerial.ino) -> TX1 stream -> RPi Pico

#ifdef ARDUINO_ARCH_MBED_RP2040
// For the Arduino MBedOS Core, we must create Serial2
UART Serial2(8, 9, NC, NC);  // Pin 12, GP9 RX 3.3V TTL level!
#else
// For the Earl Philhower core ( https://github.com/earlephilhower )
// Serial2 is already defined.
#endif

void pixyCameraInit() 
{
  //Serial1.begin(115200);    // GP0,1
  Serial2.begin(115200);      // GP8,9
}

//
// actually, read Pixy and Sonars
//
int readPixyCamera()
{
  int blocksCount = 0;
  char buf[128];
  
  // if there's any serial available, read it:
  while (Serial2.available() > 0) {

    byte chrIn = Serial2.read();

    if (chrIn == '*')
    {
      // pixy block reported:
      pixyBlock.x = Serial2.parseInt();
      pixyBlock.y = Serial2.parseInt();
      pixyBlock.width = Serial2.parseInt();
      pixyBlock.height = Serial2.parseInt();
      pixyBlock.signature = Serial2.parseInt(); // one of the colors defined in training
      blocksCount++;
#ifdef TRACE
      sprintf(buf, "*%d: *%d %d %d %d %d\n", blocksCount, pixyBlock.x, pixyBlock.y, pixyBlock.width, pixyBlock.height, pixyBlock.signature);   
      Serial.print(buf);
#endif // TRACE
    } else if (chrIn == '#') {
      
      // sonar readings reported:
      sonarLF = Serial2.parseInt();
      sonarF  = Serial2.parseInt();
      sonarRF = Serial2.parseInt();
    }

  }

#ifdef TRACE
  if(blocksCount > 0)
  {
    Serial.print("Pixy Blocks: "); 
    Serial.println(blocksCount); 
  }
#endif // TRACE
  
  return blocksCount;
}
