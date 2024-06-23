#if !defined(PCA9685Emulator_h)
#define PCA9685Emulator_h

// Original code: https://github.com/jwatte/donkey_racing/tree/master

#define PCA9685_I2C_ADDRESS 0x40

class PCA9685Emulator {
public:
  PCA9685Emulator();

  void begin(uint8_t address);

  //  Return true if there are new values
  bool step();

  //  Return servo PWM value in microseconds
  uint16_t readChannelUs(uint16_t ch);

  void onRequest();
  void onReceiveBlock(int n);

  static void onRequestIsr();
  static void onReceiveIsr(int n);

  static PCA9685Emulator *active;  //  only one can be active at a time, "singleton" pattern

  enum {
    NUM_CHANNELS = 16
  };
  uint8_t mem[6 + NUM_CHANNELS * 4];
  uint8_t wptr;
  bool gotwrite;
};

#endif  //  PCA9685Emulator_h
