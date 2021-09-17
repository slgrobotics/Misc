//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the 
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//
   
#include <SPI.h>  
#include <Pixy.h>

#define LED_PIN 13

// Adjusted to work for SmartLab ReCon 6 - see RpiPico_ReCon6  - Sep 2021

// This is the main Pixy object 
Pixy pixy;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);    // Leonardo TX1
  while (!Serial && !Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only - Leonardo
  }

  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{ 
  static int i = 0;
  int j = 0;
  uint16_t blocks;
  char buf[32]; 

  if (i%2000==0)
    digitalWrite(LED_PIN,0);
  
  // grab blocks!
  blocks = pixy.getBlocks();
  i++;
  
  // If there are detect blocks, print them!
  if (blocks)
  {
    digitalWrite(LED_PIN,1);

    // do this (print) every 10 frames because printing every
    // frame would bog down the Arduino
    //if (i%20==0)
    {
      /*
      sprintf(buf, "Detected %d:\n", blocks);
      Serial.print(buf);
      for (j=0; j<blocks; j++)
      {
        sprintf(buf, "  block %d: ", j);
        Serial.print(buf); 
        pixy.blocks[j].print();
      }
      */

      Block *block = &pixy.blocks[0];

      if(block->width > 30 && block->height > 30)
      {
        printBlock(block);
      }
    }
  }  
}

  // print block structure - see C:\Projects\Arduino\Sketchbook\libraries\Pixy\TPixy.h
  // Field of view:
  //     goal 45 degrees  left  x=0
  //                    middle  x=150
  //     goal 45 degrees right  x=300
  //
  //     goal 30 degrees  up    y=0
  //                    middle  y=85
  //     goal 30 degrees down   y=190
  //
  //    http://cmucam.org/projects/cmucam5/wiki/Arduino_API
  //
  //  pixy.blocks[i].signature The signature number of the detected object (1-7 for normal signatures)
  //  pixy.blocks[i].x The x location of the center of the detected object (0 to 319)
  //  pixy.blocks[i].y The y location of the center of the detected object (0 to 199)
  //  pixy.blocks[i].width The width of the detected object (1 to 320)
  //  pixy.blocks[i].height The height of the detected object (1 to 200)
  //  pixy.blocks[i].angle The angle of the object detected object if the detected object is a color code.
  //  pixy.blocks[i].print() A member function that prints the detected object information to the serial port

  void printBlock(Block *block)
  {
    int x = block->x - 160; // objects to the left - negative, to the right - positive (-150...150)
    int y = 100 - block->y; // objects below - negative above - positive (-90...90)
    
    char buf[128];
    //sprintf(buf, "sig: %d x: %d y: %d width: %d height: %d\n", signature, x, y, width, height);   
    sprintf(buf, "*%d %d %d %d %d\n", x, y, block->width, block->height, block->signature);   
    Serial.print(buf); 
    Serial1.print(buf); 
    //block->print();
  }
