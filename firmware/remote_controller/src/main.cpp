#include <Arduino.h>

const int pin_velocity = A5;
const int pin_direction = A6;
unsigned char buff[4] = {0x00, 0x01, 0xff, 0x01}; //dir '00', vel 'ff'

void setup() {
  Serial.begin(115200);
  pinMode(pin_velocity, INPUT);
  pinMode(pin_direction, INPUT);
}

void loop() {
  buff[1] = map(analogRead(pin_direction), 0, 1024, 1, 254);
  buff[3] = map(analogRead(pin_velocity), 0, 1024, 1, 254);
  Serial.write(buff, 4);
  delay(20);
}
