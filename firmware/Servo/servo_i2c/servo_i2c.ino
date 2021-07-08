#include <Wire.h>
#include <Servo.h>

Servo servo[3];

void setup()
{
  Wire.begin(0x06);
  Wire.onReceive(receiveEvent);
  servo[0].attach(3);
  servo[1].attach(5);
  servo[2].attach(6);
  servo[0].write(164);
  servo[1].write(1); //1
  servo[2].write(179); //179
}

void receiveEvent(int howMany)
{
  while(1 < Wire.available())
  {
    char c = Wire.read();
  }
  int value = Wire.read();
  
  if (value>90){
    value -= 128;
    servo[1].write(value*2);
    servo[2].write(180 - value*2);
  } else {
    servo[0].write(value*2);
  }
}

void loop(){}
