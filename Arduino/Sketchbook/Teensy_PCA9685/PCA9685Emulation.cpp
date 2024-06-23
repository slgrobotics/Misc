// Original code: https://github.com/jwatte/donkey_racing/tree/master

#include <Arduino.h>

// https://github.com/Richard-Gemmell/teensy4_i2c
// https://github.com/Richard-Gemmell/teensy4_i2c/blob/master/documentation/installation/arduino_installation.md
// just unpack the master zip into the library folder
#include <i2c_driver.h>
#include <i2c_driver_wire.h>

#include "PCA9685Emulator.h"

//#define TRACE

static const uint8_t MODE1 = 0x00;
static const uint8_t MODE2 = 0x01;
static const uint8_t SUBADR1 = 0x02;
static const uint8_t SUBADR2 = 0x03;
static const uint8_t SUBADR3 = 0x04;
static const uint8_t FAKE_PRESCALE = 0x05;
static const uint8_t LED0_ON_L = 0x06;
static const uint8_t LED0_ON_H = 0x07;
static const uint8_t LED0_OFF_L = 0x08;
static const uint8_t LED0_OFF_H = 0x09;
static const uint8_t ALL_LED_ON_L = 0xFA;
static const uint8_t ALL_LED_ON_H = 0xFB;
static const uint8_t ALL_LED_OFF_L = 0xFC;
static const uint8_t ALL_LED_OFF_H = 0xFD;
static const uint8_t PRESCALE = 0xFE;

static const uint8_t SWRESET = 0x06;

static const uint8_t RESTART = 0x80;
static const uint8_t SLEEP = 0x10;
static const uint8_t ALLCALL = 0x01;
static const uint8_t INVRT = 0x10;
static const uint8_t OUTDRV = 0x04;


PCA9685Emulator *PCA9685Emulator::active;

PCA9685Emulator::PCA9685Emulator() {
  wptr = 0;
  gotwrite = false;
}

void PCA9685Emulator::begin(uint8_t address) {
  active = this;
  Wire.begin(address);
  Wire.onRequest(onRequestIsr);
  Wire.onReceive(onReceiveIsr);
}

bool PCA9685Emulator::step() {
  // flag to pick data from interrupt service routine:
  bool ret;
  cli();
  ret = gotwrite;
  gotwrite = false;
  sei();
  return ret;
}

uint16_t PCA9685Emulator::readChannelUs(uint16_t ch) {
  // convert received data into particular channel's microseconds:

  if (ch >= NUM_CHANNELS) {
    return 0xffff;
  }
  uint16_t ret;
  cli();
  if ((mem[MODE1] & SLEEP) || !(mem[MODE2] & OUTDRV)) {
    ret = 0;
    sei();
  } else {
    uint32_t l_on_0 = mem[LED0_ON_L + 4 * ch];
    uint32_t h_on_0 = mem[LED0_ON_H + 4 * ch];
    uint32_t l_off_0 = mem[LED0_OFF_L + 4 * ch];
    uint32_t h_off_0 = mem[LED0_OFF_H + 4 * ch];
    uint32_t ps = mem[FAKE_PRESCALE] + 1;
    sei();
    ret = ((l_off_0 + (h_off_0 << 8)) - (l_on_0 + (h_on_0 << 8))) * ps / 25ul;
  }
  return ret;
}

void PCA9685Emulator::onRequestIsr() {
  // interrupt service routine
  active->onRequest();
}

void PCA9685Emulator::onRequest() {
#ifdef TRACE
  Serial.println("");
  Serial.print("onRequest(): sending back ");
  Serial.println(wptr);
#endif
  // Only support returning a single byte. That's what PX4 driver expects.
  // Other systems (ROS?) may expect different response, TBD.
  if (wptr < sizeof(mem)) {
    Wire.write(mem[wptr & 0xf]);
  } else {
    Wire.write(0xff);
  }
}

void PCA9685Emulator::onReceiveIsr(int received) {
  // interrupt service routine

  if (received != 0) {
    if (received == 1) {
      //  a command
      uint8_t a_command = Wire.read();
#ifdef TRACE
      Serial.print("command: ");
      Serial.println(a_command);
#endif
      if (a_command == SWRESET) {
        //  software reset -- I don't care?
      }
    } else {
      // crawl through the block:
      active->onReceiveBlock(received);
    }
  }
}

void PCA9685Emulator::onReceiveBlock(int received) {
  //  Empty the pipe within the interrupt request, to
  //  make sure I don't race with the step() function.

  // Teensy 4.0 PCA9685 Emulator - listening to I2C
  //
  // received: 2  wptr: 0
  // 0onRequest(): sending back 1
  //
  // received: 2  wptr: 0
  // 48
  // received: 2  wptr: 1
  // 4
  // received: 2  wptr: 254
  // 126 - PRESCALE
  //
  // received: 2  wptr: 0
  // 32
  // then block reads (64+1 bytes):
  // received: 65  wptr: 6
  // 0 0 52 1 |  0 0 52 1 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |  0 0 52 1 |  0 0 52 1 |  0 0 52 1 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |  0 0 205 0 |

#ifdef TRACE
  Serial.println("");
  Serial.print("received: ");
  Serial.print(received);
#endif
  gotwrite = true;
  wptr = Wire.read();
  --received;
#ifdef TRACE
  Serial.print("  wptr: ");
  Serial.println(wptr);
#endif

  while (received > 0) {
    unsigned char rb = Wire.read();
#ifdef TRACE
    Serial.print(" ");
    Serial.print(rb);
    if ((received - 1) % 4 == 0) {
      Serial.print(" | ");
    }
#endif
    if (wptr < sizeof(mem)) {
      mem[wptr] = rb;
    } else if (wptr == PRESCALE) {
#ifdef TRACE
      Serial.println(" - PRESCALE");
#endif
      mem[FAKE_PRESCALE] = rb;
    } else {
      (void)rb;
    }
    ++wptr;
    --received;
  }
}
