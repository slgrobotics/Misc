
const int ENCODER_L_PIN = 10;    // side L (interrupt 0, Left encoder)
const int ENCODER_R_PIN = 11;    // side R (interrupt 1, Right encoder)

#define DBG1_PIN        14
#define DBG2_PIN        15

// debugging LEDs:
const int redLedPin   = 16; // GP16
const int greenLedPin = 17;
const int blueLedPin  = 18;

double pwm_R, pwm_L;    // pwm -255..255 accumulated here and ultimately sent to H-Bridge pins (will be constrained by set_motor()) 

volatile int Ldistance, Rdistance;    // encoders - distance traveled, used by balance calculator for increments and zeroed often
volatile int oLdistance, oRdistance;  // encoders - distance traveled, used by odometry calculator

void setup() 
{
  Serial.begin(115200);   // start serial for USB

  // DEBUG pins:
  pinMode(DBG1_PIN, OUTPUT);
  pinMode(DBG2_PIN, OUTPUT);

  // debugging LEDs:
  pinMode(redLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  // Left encoder:
  pinMode(ENCODER_L_PIN, INPUT); 

  // Right encoder:
  pinMode(ENCODER_R_PIN, INPUT); 
  
  attachInterrupt(ENCODER_L_PIN, leftEncoder, CHANGE);   // left encoder interrupt
  attachInterrupt(ENCODER_R_PIN, rightEncoder, CHANGE);  // right encoder interrupt

  pwm_R = 1.0;
  pwm_L = 1.0;
}

void loop() 
{
  delay(1000);
  digitalWrite(blueLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
}

// ************************************
//    Read distance from the encoders
// ************************************
void rightEncoder()
{
  digitalWrite(blueLedPin, HIGH);

  // we spend around 1.2us in the interrupt, at approx 30Hz frequency at pwm=80
  digitalWrite(DBG1_PIN, HIGH);
  
  //boolean vi = digitalRead(ENCODER_R_PIN);

  // as the direction is not known from encoder alone, we borrow that from motors:
  if((int)pwm_R < 0)
  {
    //digitalWrite(DBG2_PIN, LOW);
    Rdistance--;
    oRdistance--;
  } else if((int)pwm_R > 0) {
    //digitalWrite(DBG2_PIN, HIGH);
    Rdistance++;                // wheel moves forward, positive increase
    oRdistance++;
  }
  digitalWrite(DBG1_PIN, LOW);
}

void leftEncoder()
{
  digitalWrite(greenLedPin, HIGH);

  // we spend around 1.2us in the interrupt, at approx 30kHz frequency at pwm=80
  digitalWrite(DBG1_PIN, HIGH);
  
  //boolean vi = digitalRead(ENCODER_L_PIN);

  // as the direction is not known from encoder alone, we borrow that from motors:
  if((int)pwm_L < 0)
  {
    //digitalWrite(DBG2_PIN, LOW);
    Ldistance--;
    oLdistance--;
  } else if((int)pwm_L > 0) {
    //digitalWrite(DBG2_PIN, HIGH);
    Ldistance++;                // wheel moves forward, positive increase
    oLdistance++;
  }
  digitalWrite(DBG1_PIN, LOW);
}
