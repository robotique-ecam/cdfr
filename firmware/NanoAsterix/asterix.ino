#include <Wire.h>
#include <Servo.h>

#define READ_PIN A0
#define R1 2200

byte received = 0;
bool requested = false;
Servo servo;

void setup(){
  Serial.begin(9600);
  pinMode(READ_PIN, INPUT);
  Wire.begin(0x6);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  servo.attach(9);
}

void loop() {
  if (requested) {
    Serial.println("Requested resistance value");
    requested = false;
  }
  if (received != 0) {
    Serial.print("Setting servo value to: ");
    Serial.println(received);
    received = 0;
  }
  delay(100);
}

void receiveEvent(int howMany)
{
  received = Wire.read();
  servo.write(received);
}

void requestEvent() {
  requested = true;
  int R = 0;
  for (int i = 0; i < 10; i++) {
    int V = analogRead(READ_PIN);
    R += V;
    delay(10);
  }
  R /= 10;
  int res = map(R, 0, 1023, 0, 255); // I2C Range: (0, 255)
  Wire.write(res);
}