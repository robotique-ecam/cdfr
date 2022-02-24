#include <Wire.h>

#define READ_PIN A0

void setup(){
  Serial.begin(9600);
  pinMode(READ_PIN, INPUT);
  Wire.begin(6);
  Wire.onRequest(requestEvent);
}

void loop() {}

void requestEvent() {
  int R = 0;
  for (int i = 0; i < 10; i++) {
    int V = analogRead(READ_PIN);
    R += V;
    delay(10);
  }
  R /= 10;
  int res = map(R, 0, 1023, 0, 255); // I2C Range: (0, 255)
  Wire.write(res);
  //Serial.println("Resistance value: " + res);
}