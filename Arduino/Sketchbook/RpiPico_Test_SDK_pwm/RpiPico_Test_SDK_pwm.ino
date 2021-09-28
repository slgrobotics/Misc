// Output PWM signals on pins 0 and 1
 
//#include <pico/stdio.h>
//#include <pico/stdlib.h>
//#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
 
void setup() {
  Serial.begin(115200);

  set_pwm_pin(1, 100, 5000);

 /*
  // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(0, GPIO_FUNC_PWM);
  gpio_set_function(1, GPIO_FUNC_PWM);
  
  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(0);
  
  // Set period of 4 cycles (0 to 3 inclusive)
  pwm_set_wrap(slice_num, 3);
  // Set channel A output high for one cycle before dropping
  pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
  // Set initial B output high for three cycles before dropping
  pwm_set_chan_level(slice_num, PWM_CHAN_B, 3);
  // Set the PWM running
  pwm_set_enabled(slice_num, true);
  
  // Note we could also use pwm_set_gpio_level(gpio, x) which looks up the
  // correct slice and channel for a given GPIO.
  */
}

void loop() {
  delay(1000);
  Serial.println("Test_SDK_pwm: loop()");
}

void set_pwm_pin(uint pin, uint freq, uint duty_c) { // duty_c between 0..10000, freq >=50
  gpio_set_function(pin, GPIO_FUNC_PWM);
  uint slice_num = pwm_gpio_to_slice_num(pin);
  pwm_config config = pwm_get_default_config();
  float div = (float)clock_get_hz(clk_sys) / (freq * 10000);
  pwm_config_set_clkdiv(&config, div);
  pwm_config_set_wrap(&config, 10000); 
  pwm_init(slice_num, &config, true);   // start the pwm running according to the config
  pwm_set_gpio_level(pin, duty_c);      //connect the pin to the pwm engine and set the on/off level. 
}
