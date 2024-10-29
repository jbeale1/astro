 
 // Arduino code to measure pulsewidth on GPIO input pin
 // Note: gear has 32 teeth, so 32 cycles = 1 full turn
 // 27-Oct-2029 J.Beale

 
#define BUTTON_PIN 2  // on AtMega328p, only pins 2,3 can have interrupts
#define OUT1 8

// PWM on Arduino Uno: Pins 3, 5, 6, 9, 10, and 11

#define MD1 5  // motor drive 1 (PWM output, forward direction, to H-bridge)
#define MD2 6  // motor drive 2 (PWM output, reverse direction)

unsigned long pulseInTimeBegin;
unsigned long pulseInTimeEnd;
unsigned long pulseInPriorEnd;
bool newPulseDurationAvailable = false;
bool priorIn1 = false;
long steps = 0;  // how many full cycles have been counted 
int direction = 1;  // which direction we are turning

int stepMaxLimit = 20;  // maximum step count before reversing
int stepMinLimit = 0;

/// Set the current on a motor channel using PWM and directional logic.
/// \param pwm    PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN  pin number xIN1 for the given channel
/// \param IN2_PIN  pin number xIN2 for the given channel
void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm == 0) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }
  else if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("Input pulse width measure 27-Oct-2024");
    pinMode(BUTTON_PIN, INPUT);  // signal input
    pinMode(OUT1, OUTPUT);       // test signal output
    pinMode(LED_BUILTIN, OUTPUT);       // onboard LED indicator
    pinMode(MD1, OUTPUT);       // motor drive PWM output #1
    pinMode(MD2, OUTPUT);       // motor drive PWM output #2

    set_motor_pwm(0, MD1, MD2);

    pulseInPriorEnd = millis();
    // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPinInterrupt, CHANGE);
}

void buttonPinInterrupt() {
    if (digitalRead(BUTTON_PIN) == HIGH) {
      pulseInTimeBegin = micros();
    } else {
      pulseInPriorEnd = pulseInTimeEnd;
      pulseInTimeEnd = micros();
      newPulseDurationAvailable = true;
    }
}

int sign(int x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}
/*
void motorExcercise() {
  for (int j=0;j<4;j++) {
    for (int i=-(pMax-pTh); i<(pMax-pTh); i+=2) {
        pwm = pTh*sign(i) + i;
        set_motor_pwm(pwm, MD1, MD2);
        delay(pDelay);
    }
    for (int i=(pMax-pTh); i>-(pMax-pTh); i-=2) {
        pwm = pTh*sign(i) + i;
        set_motor_pwm(pwm, MD1, MD2);
        delay(pDelay);
    }
  }
}
*/

void loop() {

// PWM value of 70 is about right for existing gearmotor

int pwmRate = 100;
int pTh = 60;  // PWM threshold voltage for motor motion
int pMax = 80; // maximum PWM value for motor motion
int pDelay = 400;
int dbTime = 40;  // debounce time

int pwm = direction * pwmRate;
set_motor_pwm(pwm, MD1, MD2);


    bool newIn1 = digitalRead(BUTTON_PIN);
    if (newIn1 != priorIn1) {
      priorIn1 = newIn1;
      if (newIn1 == HIGH) {
        pulseInTimeBegin = millis();
        delay(dbTime); //debounce
      } else {
        pulseInPriorEnd = pulseInTimeEnd;
        pulseInTimeEnd = millis();
        delay(dbTime); //debounce
        // newPulseDurationAvailable = true;
        unsigned long pulseDuration1 = pulseInTimeEnd - pulseInTimeBegin;
        unsigned long pulseDuration2 = pulseInTimeBegin - pulseInPriorEnd;
        Serial.print(steps);
        Serial.print(",");
        Serial.print(pulseDuration1);
        Serial.print(",");
        Serial.println(pulseDuration2);
        steps += direction;
        if ((steps >= stepMaxLimit) || (steps <= stepMinLimit)) {
            direction = -direction;
            set_motor_pwm(0, MD1, MD2);
            delay(5000);
        }
      }

    }

}
