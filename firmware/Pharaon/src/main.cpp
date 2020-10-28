#include <APA102.h>

#define buttonUP 21
#define buttonDOWN 20
#define DIS 11
#define DIR 9
#define STP 8
#define stepSpeed 10
#define LedData 13
#define LedClock 12

uint32_t stepsForUp = 165000;
APA102<LedData, LedClock> ledStrip;
uint32_t currentSteps = 0;
int currentLed = 0;
const int ledCount = 8;
rgb_color colors[ledCount];
bool deployed = false;

void setup() {
  pinMode(buttonUP, INPUT);
  pinMode(buttonDOWN, INPUT);
  pinMode(DIS, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(STP, OUTPUT);
  digitalWrite(DIS, HIGH);
  Serial.begin(9600);
  Serial2.begin(9600);
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  // T = 250ms --> f = 4 Hz
  OCR1A = 15624; // = (16*10^6) / (f*256) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);
  // Set bits for 256 prescalar
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  sei();
  turn_off_all_leds();
}

void loop() {
  if (Serial2.available()) {
    String data = Serial2.readStringUntil(";");
    // Serial.println("Data:" + data);
    if (data == "deploy;") {
      deploy();
    }
  }
  if (Serial.available()) {
    String data2 = Serial.readStringUntil(";");
    // Serial.println("Data:" + data2);
    if (data2 == "deploy;\n") {
      deploy();
    }
  }
  if (digitalRead(buttonUP) == HIGH) {
    deployed = false;
    currentSteps = 0;
    step(0);
  }
  else if (digitalRead(buttonDOWN) == HIGH) {
    deployed = false;
    currentSteps = 0;
    step(1);
  }
  else if (currentSteps > 0) {
    currentSteps--;
    step(0);
    if (currentSteps == 0) {
      deployed = true;
      Serial2.write("deployed;");
      Serial.println("Finished deployment!");
    }
  } else {
    digitalWrite(DIS, HIGH);
  }
}

ISR(TIMER1_COMPA_vect){
  if (deployed) {
    light();
  } else {
    turn_off_all_leds();
  }
}

void deploy() {
  Serial.println("Starting deployment...");
  currentSteps = stepsForUp;
}

void light() {
  currentLed %= ledCount;
  turn_off_all_leds();
  colors[currentLed] = rgb_color(255,255,0);
  ledStrip.write(colors, ledCount, 16);
  currentLed++;
}

void step(int d) {
  digitalWrite(DIS, LOW);
  digitalWrite(DIR,d);
  digitalWrite(STP,HIGH);
  delayMicroseconds(stepSpeed);
  digitalWrite(STP,LOW);
  delayMicroseconds(stepSpeed);
}

void turn_off_all_leds() {
  for (int i = 0; i < ledCount; i++) {
    colors[i] = rgb_color(0, 0, 0);
  }
  ledStrip.write(colors, ledCount, 16);
}
