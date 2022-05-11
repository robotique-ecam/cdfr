#include <Wire.h>
#include <Servo.h>
#include <AX12A.h>

#define RESISTANCE_READ_PIN A0
#define DYNA_CUBE1_ID 1
#define DYNA_CUBE2_ID 2
#define DYNA_HEX_ID 3
#define RESISTANCE_SERVO_PIN 7

bool received, requested;
int received_val, requested_val;
Servo servo;

void setup(){
  Serial.begin(9600);
  pinMode(RESISTANCE_READ_PIN, INPUT);
  Wire.begin(0x6);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  servo.attach(RESISTANCE_SERVO_PIN);
  ax12a.begin(1000000, 10, &Serial);
  ax12a.setEndless(DYNA_CUBE1_ID, OFF);
  ax12a.setEndless(DYNA_CUBE2_ID, OFF);
  ax12a.setEndless(DYNA_HEX_ID, OFF);
}

void loop() {
  if (requested) {
    Serial.print("Requested resistance value: " );
    Serial.println(requested_val);
    requested = false;
  }
  if (received) {
    switch (received_val) {
      case 1:
        deploy_res_servo();
        break;
      case 2:
        retract_res_servo();
        break;
      case 3:
        deploy_cube1_dyna();
        break;
      case 4:
        retract_cube1_dyna();
        break;
      default:
        Serial.print("ERROR. Unknown I2C value received: ");
        Serial.println(received_val);
        break;
    }
    received = false;
  }
  delay(100);
}

void deploy_res_servo() {
  servo.write(10);
  delay(1000);
  Serial.println("Deployed resistance servo");
}

void retract_res_servo() {
  servo.write(110);
  delay(1000);
  Serial.println("Retracted resistance servo");
}

void deploy_cube1_dyna() {
  ax12a.moveSpeed(DYNA_CUBE1_ID, 600, 100);
  delay(1000);
  Serial.println("Deployed cube 1 dynamixel");
}

void retract_cube1_dyna() {
  ax12a.moveSpeed(DYNA_CUBE1_ID, 700, 100);
  delay(1000);
  Serial.println("Retracted cube 1 dynamixel");
}

void receiveEvent(int howMany)
{
  received_val = Wire.read();
  received = true;
}

void requestEvent() {
  int R = 0;
  for (int i = 0; i < 10; i++) {
    int V = analogRead(RESISTANCE_READ_PIN);
    R += V;
    delay(10);
  }
  R /= 10;
  requested_val = map(R, 0, 1023, 0, 255); // I2C Range: (0, 255)
  requested = true;
  Wire.write(requested_val);
}