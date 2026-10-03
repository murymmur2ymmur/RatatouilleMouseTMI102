#include <Wire.h>
#include <ESP32Servo.h>

// MPU6050 / MPU6500 clone, read directly over I2C (no chip-ID check)
const uint8_t MPU_ADDR = 0x68;
const int SDA_PIN = 25;
const int SCL_PIN = 26;
const int SERVO_PIN = 32;

// Servo object
Servo myServo;

// Smoothing: 0 < ALPHA <= 1. Lower = smoother but slower.
const float ALPHA = 0.2f;

float rollSmooth = 0, pitchSmooth = 0;
bool firstSample = true;
unsigned long lastPrint = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool readAccel(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                                // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 6) != 6) return false;

  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();

  ax = rx / 16384.0f;                              // in g, at +/-2g range
  ay = ry / 16384.0f;
  az = rz / 16384.0f;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Attach servo to GPIO 32
  myServo.attach(SERVO_PIN);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  writeReg(0x6B, 0x00);   // wake up (chip starts in sleep mode)
  delay(100);
  writeReg(0x1C, 0x00);   // accel range +/-2g
  writeReg(0x1A, 0x04);   // gyro/general low-pass filter ~21 Hz
  writeReg(0x1D, 0x04);   // accel low-pass filter ~21 Hz (MPU6500 only, harmless otherwise)

  // Show the chip ID for reference (0x68 = MPU6050, 0x70 = MPU6500)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(true);
  if (Wire.requestFrom((int)MPU_ADDR, 1) == 1) {
    Serial.print("WHO_AM_I = 0x");
    Serial.println(Wire.read(), HEX);
  } else {
    Serial.println("WHO_AM_I read failed - check wiring");
  }

  Serial.println("Roll and Pitch are in degrees. Tilt is the angle away from flat.");
  Serial.println("--------------------------------------------------------------");
}

void loop() {
  float ax, ay, az;
  if (!readAccel(ax, ay, az)) {
    Serial.println("I2C read failed");
    delay(100);
    return;
  }

  // Roll: rotation around the X axis. Pitch: rotation around the Y axis.
  float roll  = atan2(ay, az) * 180.0f / M_PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

  // Total tilt away from lying flat (0 = flat face-up, 180 = flat face-down)
  float mag  = sqrt(ax * ax + ay * ay + az * az);
  float tilt = acos(constrain(az / mag, -1.0f, 1.0f)) * 180.0f / M_PI;

  // Exponential smoothing
  if (firstSample) {
    rollSmooth = roll;
    pitchSmooth = pitch;
    firstSample = false;
  } else {
    rollSmooth  += ALPHA * (roll  - rollSmooth);
    pitchSmooth += ALPHA * (pitch - pitchSmooth);
  }

  // Servo angle: 90 = level, +/-90 of tilt maps to 0..180
  int servoAngle = constrain((int)(90 + rollSmooth), 0, 180);

  // Drive the SG90 servo motor
  myServo.write(servoAngle);

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("Servo angle "); Serial.println(servoAngle);
  }

  delay(10);
}
