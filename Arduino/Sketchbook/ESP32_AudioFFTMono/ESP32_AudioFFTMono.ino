/* 
 *  Spectrum display of audio signal on 128x64 OLED
 *  The board uses ESP32 DevKitC V4. The library is ArduinoFFT.h
 *  Original code: 2022/06/03 Radio pliers http://radiopench.blog96.fc2.com/
 *  Modified Oct 2022 by Sergei Grichine for mono operation and ESP32
 *  
 *  Calibration: https://onlinetonegenerator.com/
*/
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "arduinoFFT.h"

// attach a LED to GPIO 17
#define LED_PIN GPIO_NUM_17
#define DEBUG_PIN GPIO_NUM_16

arduinoFFT FFT = arduinoFFT(); // Create FFT object
 
#define SAMPLE_BUFFER_SIZE 512  // number of samples

double vReal[SAMPLE_BUFFER_SIZE];         // Computational domain of FFT (actually maybe 32-bit float?)
double vImag[SAMPLE_BUFFER_SIZE];
int32_t wave[SAMPLE_BUFFER_SIZE];         // raw waveform data
double maxWave = 0.0;  // scaling for wave display
 
void setup()
{
  pinMode(DEBUG_PIN, OUTPUT);   // for execution time measurement
  pinMode(LED_PIN, OUTPUT);     // LED

  // Wire.setClock(400000);     // this has no effect
  Serial. begin(115200);

  initMicrophone();

  initDisplay();

  delay(1000);
}

void loop()
{
  // Read waveform (5ms)
  // digitalWrite(DEBUG_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);         // LED lights during sampling

  if (getWave() != ESP_OK)
  {
    Serial.println("Error: i2s_read() failed");
    displayError("i2s_read() failed");
    delay(100);
    return;
  }

  digitalWrite(LED_PIN, LOW);
  // digitalWrite(DEBUG_PIN, LOW);
 
  performFFT();
 
  displayAll(); 
} // (Loop execution time: 82ms)

