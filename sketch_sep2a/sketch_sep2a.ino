#include <ESP32Servo.h>
#include <Wire.h>

#include "I2Cdev.h" 
#include "MPU6050_6Axis_MotionApps20.h"

Servo servo;
MPU6050 mpu;
Quaternion q;
VectorFloat gravity;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

void setup() {
  Wire.begin(4,15);
  ESP32PWM::allocateTimer(0);
  servo.attach(14, 500, 2400);
  
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
  fifoCount = mpu.getFIFOCount();

  if (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    if (fifoCount > packetSize * 2) {
      mpu.resetFIFO();
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);

    float angle = acos(constrain(-gravity.z, -1.0f, 1.0f)) * (180.0f);
    servo.write(angle);
  }
}