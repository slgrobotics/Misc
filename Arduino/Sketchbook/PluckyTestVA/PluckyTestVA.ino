

#define COMM_SERIAL Serial
#define BAUDRATE_ARTICUBOTS 115200

const int batteryVoltageInPin = A3;  // Analog input pin that the battery 1/3 divider is attached to. "900" = 13.713V
const int batteryCurrentInPin = A6;  // Analog input pin that the current sensor is attached to.

char out_buf[128];

void setup() {
  COMM_SERIAL.begin(BAUDRATE_ARTICUBOTS);  // start serial for USB to Raspberry Pi
}

void loop() {

  long voltage_mv = analogRead(batteryVoltageInPin); // "800" here relates to battery voltage 11.72V = 3.90V per cell
  //voltage_mv = voltage_mv * 39l / 8l; // millivolts, returns "4031" for 4.031V per cell
  voltage_mv = voltage_mv * 117l / 8l; // millivolts, returns "12000" for 12.0V
  long current_ma = analogRead(batteryCurrentInPin);
  
  current_ma = 529l + 51l;
  current_ma = max(51l,current_ma);
  current_ma = (current_ma - 51l) * 3730l / 529l; // milliamperes, returns 580 for 3.73A
  //current_ma = (current_ma - 51l); // direct A/D reading, after offset
  sprintf(&out_buf[0], "%ld %ld\r", voltage_mv, current_ma);

  COMM_SERIAL.println(out_buf);

  delay(1000);
}

/*
 * Calibration (after offset -51):
 * 
 * 0.51A = 0
 * 2.03A = 286
 * 3.73A = 529
 * 5.30A = 656
 * 
Current sensor calibration - Dragger, pin A05
https://www.desmos.com/calculator
A/D  Amp
2.86, 2.03
5.29, 3.73
6.56, 5.30
 */
