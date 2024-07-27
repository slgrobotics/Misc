
// battery voltage 1:3, red LED and Buzzer to warn of low voltage:
#define BAT_PIN A7
#define BAT_LED_PIN A0

// EMA period to smooth wheels movement. 100 is smooth but fast, 300 is slow.
const int EmaPeriod = 1000;

template<typename Type>
class Ema
{

public:
	inline Ema() {};
	inline ~Ema() {};

	inline void init(int period)
	{
		emaPeriod = period;
		multiplier = 2.0 / (1.0 + emaPeriod);
		valuePrev = NAN;
	};

	const inline Type Compute(Type val)
	{
		const Type valEma = emaPeriod <= 1 || isnan(valuePrev) ? val : ((val - valuePrev) * multiplier + valuePrev);
		valuePrev = valEma;
		return valEma;
	};

	const inline Type ValuePrev() { return valuePrev; };

	inline void Reset()	{ valuePrev = NAN; };

private:
	// variables :
	Type valuePrev{NAN};
	int emaPeriod{0};
	Type multiplier;
};

Ema<float> bat_ema;

void initBatteryMonitor()
{
  // can't use LED_BUILTIN pin 13 on Uno
  pinMode(BAT_LED_PIN, OUTPUT);
  digitalWrite(BAT_LED_PIN, LOW);

  bat_ema.init(EmaPeriod);

  analogReference(EXTERNAL);
}

void monitorBattery()
{
  int bat = analogRead(BAT_PIN);

  if(bat > 100) {
    // only if there is actual connection to the battery
    /*
    {
      float v_bat = bat * 8.54 / 739.0;

      Serial.print("Bat: ");
      Serial.print(bat);
      Serial.print("  VBat: ");
      Serial.println(v_bat);
    }
    */

    bat = (int)bat_ema.Compute((float)bat);

    /*
    {
      float v_bat = bat * 8.54 / 739.0;

      Serial.print("Bat: ");
      Serial.print(bat);
      Serial.print("  VBat: ");
      Serial.println(v_bat);
    }
    */

    // 517 is reading at 6.00V battery voltage: 
    if(millis() > 1000 && bat < 517) {
      feather_all();
      while(true) { // freeze. Battery too low.
        delay(200);
        digitalWrite(BAT_LED_PIN, HIGH);
        delay(200);
        digitalWrite(BAT_LED_PIN, LOW);
      }
    }
  }
}