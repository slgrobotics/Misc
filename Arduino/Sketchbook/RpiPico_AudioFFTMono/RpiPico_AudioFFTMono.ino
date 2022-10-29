/* 
 *  Spectrum display of audio signal on 128x64 OLED 20220603_PiPicoFftAnalyzer.ino
 *  The board uses RP pico. The library is ArduinoFFT.h
 *  Original: 2022/06/03 Radio pliers http://radiopench.blog96.fc2.com/
 *  Modified Oct 2022 by Sergei Grichine for mono operation
 *  
 *  Calibration: https://onlinetonegenerator.com/
*/
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "arduinoFFT.h"
 
#define R_IN 26 // R-input pin
 
#define PX2 0  // screen origin (left side edge)
#define PY1 16 // Bottom edge of waveform screen
#define PY2 55 // lower edge of spectrum screen (-50db)
 
arduinoFFT FFT = arduinoFFT(); // Create FFT object
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
 
const uint16_t samples = 256;  // number of samples
double vReal[samples];         // Computational domain of FFT (actually maybe 32-bit float?)
double vImag[samples];
int16_t wave[samples];         // raw waveform data
 
void setup() {
  pinMode(16, OUTPUT);      // for execution time measurement
  pinMode(25, OUTPUT);      // Pico built-in LED
  // Wire.setClock(400000); // this has no effect
  Serial. begin(115200);
  analogReadResolution(12); // set ADC full scale to 12 bits
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();     // Upper left is the character position
  u8g2.clearBuffer();
  u8g2.drawStr( 0, 0, "Start FFT v0.4");
  u8g2.sendBuffer();
  delay(1000);
}

void loop() {
  // Read waveform (5ms)
  // digitalWrite(16, HIGH);
  digitalWrite(25, HIGH);         // Built-in LED lights during sampling
  for (int i = 0; i < samples; i++) {
    wave[i] = analogRead(R_IN); // Get waveform data (fs:4096)
    delayMicroseconds(42);        // Sampling period adjustment (1 cycle 39us)
  }                               // (5ms)
  digitalWrite(25, LOW);
  // digitalWrite(16, LOW);
 
  // Prepare FFT calculation data (1.7ms)
  for (int i = 0; i < samples; i++) {
    vReal[i] = (wave[i] - 2048) * 3.3 / 4096.0; // convert to voltage
    vImag[i] = 0;
  }
 
  // Calculate FFT (33ms)
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Apply window function (Hamming)
 
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); // FFT
 
  FFT.ComplexToMagnitude(vReal, vImag, samples); // Calculate absolute value
 
  u8g2.clearBuffer();   // Screen buffer clear (22us)
  showWaveform();       // Waveform display (6ms)
  showSpectrum();       // Spectrum display (9.4ms)
  showOthers();         // Show other gridlines (1.6ms)
  u8g2.sendBuffer();    // (30ms)
  delay(1);             // Magic against write failure
} // (Loop execution time: 82ms)

void showWaveform() {   // Input waveform display
  for (int i = 0; i < 128; i++) {
    u8g2.drawLine(PX2 + i, PY1 - (wave[i * 2]) / 256, PX2 + i + 1, 16 - (wave[i * 2 + 1] / 256)); // waveform plot
  }
}
 
void showSpectrum() {     // Spectrum display
  int d;
  static int peak[64];    // past peak value
 
  for (int xi = 1; xi < 120; xi++) {       // Display spectrum (skip 0)
    d = barLength(vReal[xi]);
    u8g2.drawVLine(xi + PX2, PY2 - d, d);         // spectrum
    u8g2.drawVLine(xi + PX2, PY2 - peak[xi], 1);  // peak
    if (peak[xi] < d) {   // if the latest value is greater than or equal to the peak value
      peak[xi] = d;       // Update peak value
    }
    if (peak[xi] > 0) {
      peak[xi] --; // Decay the peak value
    }
  }
}

//
// Calculate length of spectrum graph
//
int barLength(double d)
{
  float fy;
  int y;
  fy = 14.0 * (log10(d) + 1.5); // 14 pixels at 10x (20dB)
  y = fy;
  y = constrain(y, 0, 56);
  return y;
}

//
// Graph decoration (drawing scales, etc.)
//
void showOthers()
{
  //u8g2.drawVLine(PX2, 0, 64);         // screen origin line (vertical axis)
  u8g2.drawHLine(0, PY2, 128);          // Spectrum bottom line
 
  // frequency scale (horizontal axis)
  for (int xp = PX2; xp < 127; xp += 5) { // 1kHz interval scale
    u8g2.drawVLine(xp, PY2 + 1, 1);
  }
  u8g2.drawVLine(PX2 + 25, PY2 + 1, 2);   // 5k ticks
  u8g2.drawVLine(PX2 + 50, PY2 + 1, 2);   // 10k ticks
 
  u8g2.setFont(u8g2_font_micro_tr);       // Small font (3x5),
  u8g2.setCursor(0, 58); u8g2.print("0"); // R side frequency display
  u8g2.setCursor(62, 58); u8g2.print("5k");
  u8g2.setCursor(110, 58); u8g2.print("10k");
 
  // Spectrum level scale (vertical axis)
  for (int y = PY2 - 7; y > 16; y -= 14) { // dB scale (horizontal dotted line)
    for (int x = 13; x < 128; x += 5) {
      u8g2.drawHLine(x, y, 2);
    }
  }
  u8g2.setCursor(0, 17); u8g2.print("0dB"); // spectral sensitivity
  u8g2.setCursor(0, 30); u8g2.print("-20"); //
  u8g2.setCursor(0, 45); u8g2.print("-40"); //
 
  // LR display
  //u8g2.setFont(u8g2_font_crox1cb_tf); // With a slightly gorgeous font
  //u8g2.setCursor(0, 3); u8g2.print("R"); //
}
