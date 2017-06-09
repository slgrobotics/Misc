/*
 * Copyright (c) 2013..., Sergei Grichine   http://trackroamer.com
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
 * although doing so, donating and contributing is always appreciated
 */
 
/*
 * controld Trackroamer's Flying Monkey animatronic head.
 * see trackroamer.com
 */

#include <Servo.h> 
#include <AnimatronicsBase.h> 
#include <AnimatedLight.h> 
#include <AnimatedServo.h>

// Pin 13 has an LED connected on most Arduino boards.
const int led = 13;

// lights pins:
const int tongueLpin = 10;
const int tongueRpin = 11;

const int eyeRedLpin = 13;      // share with led
const int eyeRedLmostPin = 12;
const int eyeRedRpin = 8;
const int eyeRedRmostPin = 7;

const int eyeWhitePin = 9;

// servo pins:
const int panServoPin = 6;
const int tiltServoPin = 4;
const int jawServoPin = 5;

// servos 5K Ohm feedbak potentiometers attached to analog pins:
const int panFeedbackPin = A0;
const int tiltFeedbackPin = A1;
const int jawFeedbackPin = A2;

// timing parameters:
const int periodLightsMs = 20;  // ms; light control period

// work variables:
unsigned long currentMillis;
unsigned long lastRunMillis;
int onHigh = 30;  // percent
int onHighMs;

// actuators:
AnimatronicsBase animBase;
AnimatedLight lightTongueL;
AnimatedLight lightTongueR;
AnimatedLight lightEyeRedL;
AnimatedLight lightEyeRedLmost;
AnimatedLight lightEyeRedRmost;
AnimatedLight lightEyeRedR;
AnimatedLight lightEyeWhite;

// a maximum of eight servo objects can be created
AnimatedServo panServo;
AnimatedServo tiltServo;
AnimatedServo jawServo;

// actuators variables:
int panSetpointMks = 1500;
int tiltSetpointMks = 1500;
int jawSetpointMks = 1500;

// measured feedback voltages mapped to servo scale 800...2200 us
//int panFeedbackMks;
//int tiltFeedbackMks;
//int jawFeedbackMks;


// the setup routine runs once when you press reset:
void setup()
{                
  setupAnimationDevices();
  
  setupDefaultAnimations();

  Serial.begin(57600);
}

void setupAnimationDevices()
{
  animBase.init(7, 3);   // allocate arrays for lights and servos
  
  lightTongueL.attach(tongueLpin);
  lightTongueR.attach(tongueRpin);
  lightEyeRedL.attach(eyeRedLpin);
  lightEyeRedLmost.attach(eyeRedLmostPin);
  lightEyeRedRmost.attach(eyeRedRmostPin);
  lightEyeRedR.attach(eyeRedRpin);
  lightEyeWhite.attach(eyeWhitePin);
  
  panServo.attach(panServoPin);
  //panServo.setEmaPeriod(200);
  
  tiltServo.attach(tiltServoPin);
  
  jawServo.attach(jawServoPin);
  //jawServo.useEma = false;

  animBase.lights[0] = &lightTongueL;
  animBase.lights[1] = &lightTongueR;
  animBase.lights[2] = &lightEyeRedL;
  animBase.lights[3] = &lightEyeRedLmost;
  animBase.lights[4] = &lightEyeRedRmost;
  animBase.lights[5] = &lightEyeRedR;
  animBase.lights[6] = &lightEyeWhite;
  
  animBase.servos[0] = &panServo;
  animBase.servos[1] = &tiltServo;
  animBase.servos[2] = &jawServo;
}

// the loop routine runs over and over again forever:
void loop() {

  /*
  //Serial.println(sizeof(Frame));              // 16 bytes
  //Serial.println(sizeof(AnimatronicsBase));   // 80 bytes
  Serial.println(sizeof(AnimatedLight));        // 54 bytes
  delay(2000);
  return;
  */
  
  checkSerial();
  
  currentMillis = millis();

  if(currentMillis > lastRunMillis)
  {
    lastRunMillis = currentMillis;
    
    animBase.animate(currentMillis);
    
    //Timer_Tick();
  }
}

// command set for communications from PC:

#define CHANNEL_ANIMATIONS       0    // clear and set default
#define CHANNEL_PAN              1    // direct control
#define CHANNEL_TILT             2    // direct control
#define CHANNEL_JAW              3    // direct control

#define CHANNEL_ANIM_TONGUE_L    10
#define CHANNEL_ANIM_TONGUE_R    11
#define CHANNEL_ANIM_EYE_RED_L   12
#define CHANNEL_ANIM_EYE_RED_LM  13
#define CHANNEL_ANIM_EYE_RED_RM  14
#define CHANNEL_ANIM_EYE_RED_R   15
#define CHANNEL_ANIM_EYE_WHITE   16

#define CHANNEL_ANIM_SERVO_PAN   20
#define CHANNEL_ANIM_SERVO_TILT  21
#define CHANNEL_ANIM_SERVO_JAW   22

#define CMD_SET_VALUE            0    // up to 127 commands, as they are masked to 0x7F
#define CMD_ANIMATIONS_CLEAR     1
#define CMD_ANIMATIONS_DEFAULT   2
#define CMD_SET_FRAMES           3
#define CMD_IDENTIFY             127

void checkSerial()
{
  // if there's any serial available, read it:
  if (Serial.available() > 0)
  {
    // look for the asterisk *. That's the beginning of your sentence:
    if (Serial.read() != '*')
    {
      return;
    }

    // look for the next 3+ valid integers in the incoming serial stream:
    int checksum = 0;
    int channel = Serial.parseInt();      checksum += channel;
    int cmd = Serial.parseInt();          checksum += cmd;
    int cmdValue;    // used for single-value commands to save cmdFrames allocation
    
    // WARNING: the size of cmdFrames can cause RAM overfill in Atmel 328 based Arduinos, it is only 2KB there.
    int *cmdFrames = NULL;
    
    int nFrames = 0;
    boolean doRepeat = (cmd & 0x80) != 0;

    // we can have 0 to 10 values, depending on the command.   
    switch(cmd & 0x7F)
    {
      case CMD_IDENTIFY:
        // respond to "*0 127 -127"
        Serial.println("*ARD COMM HEAD V1.0*");
        break;
        
      case CMD_SET_VALUE:
        cmdValue = Serial.parseInt(); checksum += cmdValue;
        break;

      case CMD_SET_FRAMES:
        nFrames = cmd >> 8;
        cmdFrames = (int*)malloc((size_t)(nFrames * 3 * sizeof(int)));    // heap allocation, competes with frames and other objects for 2KB RAM
        for(int i=0; i < nFrames * 3 ;i++)
        {
          cmdFrames[i] = Serial.parseInt(); checksum += cmdFrames[i];
        }
        break;

      default:
        break;
    }
    int checksumPkg = Serial.parseInt();  checksum += checksumPkg;

    if(checksum != 0)
    {
      // bad transmission
      //digitalWrite(led, 1);    // indicates trouble
      return;
    }

    //digitalWrite(led, 0);  // indicates normal operation

    int mappedServoValue;

    switch (channel)
    {
     case CHANNEL_PAN:      // direct control
      animBase.clearServosFrames();
      panSetpointMks = cmdValue;
      //mappedServoValue = map(panSetpointMks, 1200, 1800, 0, 180);
      mappedServoValue = panSetpointMks;
      panServo.write(mappedServoValue);
      break;
      
     case CHANNEL_TILT:      // direct control
      animBase.clearServosFrames();
      tiltSetpointMks = cmdValue;
      mappedServoValue = tiltSetpointMks;
      tiltServo.write(mappedServoValue);
      break;
      
     case CHANNEL_JAW:      // direct control
      animBase.clearServosFrames();
      jawSetpointMks = cmdValue;
      mappedServoValue = jawSetpointMks;
      jawServo.write(mappedServoValue);
      break;
      
     case CHANNEL_ANIMATIONS:    // clear and set default animations
      switch(cmd)
      {
        case CMD_ANIMATIONS_CLEAR:
          animBase.clearAllFrames();
          break;
          
        case CMD_ANIMATIONS_DEFAULT:
          setupDefaultAnimations();
          break;
          
        default:
          break;
      }
      break;
      
     case CHANNEL_ANIM_TONGUE_L:
       setFrames(&lightTongueL, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_TONGUE_R:
       setFrames(&lightTongueR, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_EYE_RED_L:
       setFrames(&lightEyeRedL, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_EYE_RED_LM:
       setFrames(&lightEyeRedLmost, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_EYE_RED_RM:
       setFrames(&lightEyeRedRmost, cmdFrames, nFrames, doRepeat);
       break;

     case CHANNEL_ANIM_EYE_RED_R:
       setFrames(&lightEyeRedR, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_EYE_WHITE:
       setFrames(&lightEyeWhite, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_SERVO_PAN:
       setFrames(&panServo, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_SERVO_TILT:
       setFrames(&tiltServo, cmdFrames, nFrames, doRepeat);
       break;
      
     case CHANNEL_ANIM_SERVO_JAW:
       setFrames(&jawServo, cmdFrames, nFrames, doRepeat);
       break;
    }
    if(cmdFrames != NULL)
    {
      free(cmdFrames);
    }
  }
}

void setFrames(Animated *anim, int *cmdFrames, int nFrames, boolean doRepeat)
{
     anim->clearFrames();
     
     int j=0;
     for(int i=0; i < nFrames ;i++, j+=3)
     {
       anim->frames[i] = new Frame(cmdFrames[j], cmdFrames[j+1], cmdFrames[j+2]);
     }
     anim->repeat = doRepeat;
     anim->start();
}

/*
void measureFeedback()
{
  // read the feedback pot analog value:
  int feedbackPotValue = analogRead(panFeedbackPin);   // value read from the pot: 710 head turned all way to the left; 295 - to the right; 504 - middle

  panFeedbackMks = map(feedbackPotValue, 710, 295, 0, 180);    // map(value, fromLow, fromHigh, toLow, toHigh)          // Servo-like  800..2200 "us" scale

}
*/

void Timer_Tick()
{
}

void setupDefaultAnimations()
{
  // WARNING: allocating all frames here will cause RAM overfill, which is only 2KB in Atmel 328 based Arduinos.
  //          if you are having strange "freezes" - comment out some lines/frames here. 
  //          also see cmdFrames[] above.
  
  animBase.clearAllFrames();  // make sure all frames are properly removed and deleted
  
  int i = 0;
  lightTongueL.frames[i++] = new Frame(10, 30, 200);
  lightTongueL.frames[i++] = new Frame(-1, 10, 200);
  //lightTongueL.frames[i++] = new Frame(-1, 30, 1000);
  lightTongueL.frames[i++] = new Frame(-1, 5, 2000);
  lightTongueL.frames[i++] = new Frame(-1, 70, 200);
  lightTongueL.frames[i++] = new Frame(-1, 10, 200);
  lightTongueL.repeat = true;
  lightTongueL.start();
  
  i = 0;
  lightTongueR.frames[i++] = new Frame(10, 70, 200);
  lightTongueR.frames[i++] = new Frame(-1, 10, 200);
  //lightTongueR.frames[i++] = new Frame(-1, 30, 1000);
  lightTongueR.frames[i++] = new Frame(-1, 5, 2000);
  lightTongueR.frames[i++] = new Frame(-1, 30, 200);
  lightTongueR.frames[i++] = new Frame(-1, 10, 200);
  lightTongueR.repeat = true;
  lightTongueR.start();
  
  i = 0;
  lightEyeRedL.frames[i++] = new Frame(0, 100, 200);
  lightEyeRedL.frames[i++] = new Frame(-1, 0, 200);
  lightEyeRedL.frames[i++] = new Frame(-1, 0, 2000);
  lightEyeRedL.frames[i++] = new Frame(-1, 50, 8000);
  lightEyeRedL.frames[i++] = new Frame(-1, 0, 8000);
  lightEyeRedL.repeat = true;
  lightEyeRedL.start();
  
  i = 0;
  lightEyeRedLmost.frames[i++] = new Frame(0, 100, 200);
  lightEyeRedLmost.frames[i++] = new Frame(-1, 0, 200);
  lightEyeRedLmost.frames[i++] = new Frame(-1, 0, 2000);
  lightEyeRedLmost.frames[i++] = new Frame(-1, 50, 10000);
  lightEyeRedLmost.frames[i++] = new Frame(-1, 0, 10000);
  lightEyeRedLmost.repeat = true;
  lightEyeRedLmost.start();
  
  i = 0;
  lightEyeRedRmost.frames[i++] = new Frame(0, 100, 200);
  lightEyeRedRmost.frames[i++] = new Frame(-1, 0, 200);
  lightEyeRedRmost.frames[i++] = new Frame(-1, 0, 2000);
  lightEyeRedRmost.frames[i++] = new Frame(-1, 50, 10000);
  lightEyeRedRmost.frames[i++] = new Frame(-1, 0, 10000);
  lightEyeRedRmost.repeat = true;
  lightEyeRedRmost.start();
  
  i = 0;
  lightEyeRedR.frames[i++] = new Frame(0, 100, 200);
  lightEyeRedR.frames[i++] = new Frame(-1, 0, 200);
  lightEyeRedR.frames[i++] = new Frame(-1, 0, 2000);
  lightEyeRedR.frames[i++] = new Frame(-1, 50, 8000);
  lightEyeRedR.frames[i++] = new Frame(-1, 0, 8000);
  lightEyeRedR.repeat = true;
  lightEyeRedR.start();
  
  i = 0;
  lightEyeWhite.frames[i++] = new Frame(0, 100, 2000);
  lightEyeWhite.frames[i++] = new Frame(-1, 30, 2000);
  lightEyeWhite.frames[i++] = new Frame(-1, 60, 8000);
  lightEyeWhite.frames[i++] = new Frame(-1, 30, 8000);
  lightEyeWhite.frames[i++] = new Frame(-1, 100, 2000);
  lightEyeWhite.frames[i++] = new Frame(-1, 0, 2000);
  lightEyeWhite.frames[i++] = new Frame(-1, 0, 3000);
  lightEyeWhite.repeat = true;
  lightEyeWhite.start();
  
  i = 0;
  panServo.frames[i++] = new Frame(-1, 1200, 1000);
  panServo.frames[i++] = new Frame(-1, 1800, 3000);
  panServo.frames[i++] = new Frame(-1, 1200, 4000);
  panServo.frames[i++] = new Frame(-1, 1500, 1000);
  panServo.repeat = true;
  panServo.start();
  
  i = 0;
  tiltServo.frames[i++] = new Frame(-1, 1200, 1000);
  tiltServo.frames[i++] = new Frame(-1, 1800, 5000);
  tiltServo.frames[i++] = new Frame(-1, 1200, 7000);
  tiltServo.frames[i++] = new Frame(-1, 1500, 2000);
  tiltServo.repeat = true;
  tiltServo.start();
  
  i = 0;
  jawServo.frames[i++] = new Frame(-1, 1800, 200);
  jawServo.frames[i++] = new Frame(-1, 1200, 200);
  jawServo.frames[i++] = new Frame(-1, 1800, 1000);
  jawServo.frames[i++] = new Frame(-1, 1200, 1000);
  jawServo.frames[i++] = new Frame(-1, 1800, 200);
  jawServo.frames[i++] = new Frame(-1, 1200, 200);
  jawServo.repeat = true;
  jawServo.start();
}



