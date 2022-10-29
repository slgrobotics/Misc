/*
 * Display module: 0.96 inch 128X64 OLED LCD Display I2C SSD1306 Driver (Blue and Yellow)
 *                 https://www.amazon.com/Hosyond-Display-Self-Luminous-Compatible-Raspberry/dp/B09C5K91H7
 * Connect:
 *    GND - GND
 *    VCC - 3.3V
 *    SCL - 22 
 *    SDA - 21
 *
 *  Original code credits: 2022/06/03 Radio pliers http://radiopench.blog96.fc2.com/
 */

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

void initDisplay()
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();     // Upper left is the character position
  u8g2.clearBuffer();
  u8g2.drawStr( 0, 0, "Start FFT v0.4");
  u8g2.sendBuffer();
}

void displayError(const char* msg)
{
  u8g2.clearBuffer();   // Screen buffer clear (22us)
  u8g2.setFont(u8g2_font_crox1cb_tf); // With a larger font
  u8g2.drawStr( 0, 0, "Error:");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr( 0, 20, msg);
  u8g2.sendBuffer();    // (30ms)
  delay(1);             // Magic against write failure
}

void displayAll()
{
  u8g2.clearBuffer();   // Screen buffer clear (22us)
  showWaveform();       // Waveform display (6ms)
  showSpectrum();       // Spectrum display (9.4ms)
  showOthers();         // Show other gridlines (1.6ms)
  u8g2.sendBuffer();    // (30ms)
  delay(1);             // Magic against write failure
}

// screen coordinates:
#define PX2 0  // screen origin (left side edge)
#define PY1 16 // Bottom edge of waveform screen (counting from the top edge) - also yellow zone
#define PY2 55 // lower edge of spectrum screen (-50db)

void showWaveform() {   // Input waveform display
  int offset = PY1/2;
  int factor = round(maxWave * 1.2 / offset);
  for (int i = 0; i < 128; i++) {
    int x = PX2 + i;
    int y1 = PY1 - wave[i] / factor - offset;
    int y2 = PY1 - wave[i + 1] / factor - offset;
    //u8g2.drawLine(PX2 + i, PY1 - (wave[i * 2]) / 256, PX2 + i + 1, 16 - (wave[i * 2 + 1] / 256)); // waveform plot
    u8g2.drawLine(x, y1, x + 1, y2); // waveform plot
  }
}
 
void showSpectrum() {     // Spectrum display
  int d;
  static int peak[128];    // past peak value
 
  for (int xi = 1; xi < 128; xi++) {       // Display spectrum (skip 0)
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
  u8g2.setCursor(62, 58); u8g2.print("1k");
  u8g2.setCursor(120, 58); u8g2.print("2k");
 
  // Spectrum level scale (vertical axis)
  for (int y = PY2 - 7; y > 16; y -= 14) { // dB scale (horizontal dotted line)
    for (int x = 0; x < 108; x += 5) {
      u8g2.drawHLine(x, y, 2);
    }
  }
  u8g2.setFont(u8g2_font_6x10_tf); // larger font
  u8g2.setCursor(110, 17); u8g2.print("0dB"); // spectral sensitivity
  u8g2.setCursor(110, 30); u8g2.print("-20"); //
  u8g2.setCursor(110, 45); u8g2.print("-40"); //
 
  // LR display
  //u8g2.setFont(u8g2_font_crox1cb_tf); // With a slightly gorgeous font
  //u8g2.setCursor(0, 3); u8g2.print("R"); //
}
