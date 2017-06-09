
#include <AlfsTechHbridge.h>

#define LED_PIN 13

AlfsTechHbridge motors;

volatile long vrDist = 0;
volatile long vlDist = 0;
 
void setup()
{
  pinMode(LED_PIN, OUTPUT);
  
  // uncomment one or both of the following lines if your motors' directions need to be flipped
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
  
  motors.init();

  attachInterrupt(0, rencoder, FALLING);
  attachInterrupt(1, lencoder, FALLING);
}

int power_A = 0;
int power_B = 0;
boolean dir = true;
unsigned int counter=0;

void loop()
{
  
  if(dir)
  {
    power_A++;
    power_B++;
  }
  else
  {
    power_A--;
    power_B--;
  }
  
  if(power_A > 254 || power_A < -254)
  {
    dir = !dir;
  }
  
  
  if (counter % 50 == 0)  // print data at 1Hz... (50 loop runs)
  {
      Serial.print("    power_A: "); Serial.print(power_A);
      Serial.print("    power_B: "); Serial.print(power_B);
      Serial.print("    Distance: "); Serial.print(vlDist); Serial.print("  "); Serial.println(vrDist);
  }

  motors.setLeftSpeed(power_A);
  motors.setRightSpeed(power_B);
  
  delay(25);
  
  counter++;
}

// encoders are on interrupts:
void rencoder()
{
  if (digitalRead(motors.ENCODER_B_B) == LOW)
    vrDist ++;
  else
    vrDist --;
}

void lencoder()
{
  if (digitalRead(motors.ENCODER_B_A) == HIGH)
    vlDist ++;
  else
    vlDist --;
}
