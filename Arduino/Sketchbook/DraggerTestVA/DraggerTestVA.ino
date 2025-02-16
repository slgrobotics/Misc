

#define COMM_SERIAL Serial
#define BAUDRATE_ARTICUBOTS 115200

const int batteryVoltageInPin = A4;  // Analog input pin that the battery 1/3 divider is attached to. "900" = 13.713V
const int batteryCurrentInPin = A5;  // Analog input pin that the current sensor is attached to.

char out_buf[128];

void setup() {
  COMM_SERIAL.begin(BAUDRATE_ARTICUBOTS);  // start serial for USB to Raspberry Pi
}

void loop() {

  long voltage_mv = analogRead(batteryVoltageInPin); // "900" here relates to battery voltage 13.713V
  voltage_mv = voltage_mv * 13713l / 900l; // millivolts, returns "12000" for 12V
  //long bat = 12000;
  long current_ma = analogRead(batteryCurrentInPin);
  //current_ma = 122l + 39l;
  current_ma = max(39l,current_ma);
  current_ma = (current_ma - 39l) * 4530l / 148l; // milliamperes, returns 148 for 4.53A
  //current_ma = (current_ma - 39l); // direct A/D reading, after offset
  sprintf(&out_buf[0], "%ld %ld\r", voltage_mv, current_ma);

  COMM_SERIAL.println(out_buf);

  delay(1000);
}

/*
 * Calibration (after offset -39):
 * 
 * 0.95A = 32
 * 2.03A = 66
 * 3.85A = 122
 * 4.53A = 148
 * 
Current sensor calibration - Dragger, pin A05
https://www.desmos.com/calculator
A/D  Amp
3.1, 0.95
6.6, 2.03
12.2, 3.85
14.8, 4.53
 */
