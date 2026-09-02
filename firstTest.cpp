#include <ESP32Servo.h>

Servo myServo;
int servopin = 14;
int ledpin = 33;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledpin,OUTPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);
  myServo.attach(servopin,500,2400);
}

void loop() {
  float sinvalue = 0.0f;
  while (true)
  {
    sinvalue += 0.05f;
    digitalWrite(33,HIGH);
    myServo.write(pow(sin(sinvalue),2)*90.0f);
    delay(2);
  }
}
