#include <APA102.h>

#define buttonUP 21
#define buttonDOWN 20
#define DIS 11
#define DIR 9
#define STP 8
#define stepSpeed 150
#define LedData 13
#define LedClock 12

APA102<LedData, LedClock> ledStrip;
int currentLed = 0;
const int ledCount = 5;
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
    if (data == "deploy") {
      deploy();
    }
    else if (data == "stop"){
      stop();
    }
  }
  if (Serial.available()) {
    String data2 = Serial.readStringUntil(";");
    // Serial.println("Data:" + data2);
    if (data2 == "deploy") {
      deploy();
    }
    else if (data2 == "stop"){
      stop();
    }
    
  }
  if (digitalRead(buttonUP) == HIGH) {
    deploy();
  }
  else if (digitalRead(buttonDOWN) == HIGH) {
    stop();
  }

  if (deployed) {
    step(LOW);
  }
}

void deploy() {
  Serial.println("Starting deployment...");
  deployed = true;
}

void stop() {
  Serial.println("Stopping deployment...");
  deployed=false;
}

ISR(TIMER1_COMPA_vect){
  if (deployed) {
    light();
  } else {
    turn_off_all_leds();
  }
}

void light() {
  uint8_t time = millis() >> 2;
  for(uint16_t i = 0; i < ledCount; i++)
  {
    uint8_t x = time - i * 8;
    colors[i].red = x;
    colors[i].green = 255 - x;
    colors[i].blue = x;
  }
  ledStrip.write(colors, ledCount, 16);
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