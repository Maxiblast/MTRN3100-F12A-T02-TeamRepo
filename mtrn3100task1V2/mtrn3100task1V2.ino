// Libraries
#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// Motor Pins
const int M1_PHASE = 12;
const int M1_ENABLE = 11;
const int M2_PHASE = 10;
const int M2_ENABLE = 9;

int motorSpeed = 50;

float refYaw = 0;

void setup() {
  Serial.begin(19200);
  //while (!Serial) {}

  // Motors
  pinMode(M1_PHASE, OUTPUT);
  pinMode(M1_ENABLE, OUTPUT);
  pinMode(M2_PHASE, OUTPUT);
  pinMode(M2_ENABLE, OUTPUT);

  // IMU
  Wire.begin();
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");
  Serial.println(status);
  Serial.println("Calibrating... keep robot still");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done");

  mpu.update();
  refYaw = mpu.getAngleZ();
}

// Main Loop
void loop() {
  static int state = 0;
  mpu.update();
  float currYaw = mpu.getAngleZ();

  switch (state) {
    case 0: // drive 200mm
      driveForward200mm();
      stop();
      delay(500);
      refYaw = currYaw;
      state = 1;
      break;
    
    case 1: { // turn 90 degrees left x 4
      for (int i = 1; i <= 4; i++) {
        float targetAngle = refYaw + 90.0 * i;
        turnTo(targetAngle, true);  // true = left
        delay(200);
      }
      state = 2;
      break;
    }

    case 2: { // turn 90 degrees right x 4
      for (int i = 1; i <= 4; i++) {
        float targetAngle = refYaw - 90.0 * i;
        turnTo(targetAngle, false);  // false = right
        delay(200);
      }
      state = 3;
      break;
    }

    case 3: // Stop
      stop();
      break;
  }
}

void driveForward200mm() {
  moveForward();
  delay(4000);
}

void turnTo(float targetYaw, bool leftTurn) {

  while (true) {
    mpu.update();
    float currentYaw = mpu.getAngleZ();
    float error = angleDiff(targetYaw, currentYaw);

    if (fabs(error) < 10.0) {
      break;
    }

    if (leftTurn) {
      turnLeft();
    } else {
      turnRight();
    }
  }
  stop();
}

float angleDiff(float targetAngle, float currentAngle) {
  float error = targetAngle - currentAngle;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  return error;
}

void moveForward() {
  digitalWrite(M1_PHASE, LOW);
  analogWrite(M1_ENABLE, motorSpeed);
  digitalWrite(M2_PHASE, HIGH);
  analogWrite(M2_ENABLE, motorSpeed);
}

void moveBackward() {
  digitalWrite(M1_PHASE, HIGH);
  analogWrite(M1_ENABLE, motorSpeed);
  digitalWrite(M2_PHASE, LOW);
  analogWrite(M2_ENABLE, motorSpeed);
}

void turnRight() {
  digitalWrite(M1_PHASE, LOW);
  analogWrite(M1_ENABLE, motorSpeed);
  digitalWrite(M2_PHASE, LOW);
  analogWrite(M2_ENABLE, motorSpeed);
}

void turnLeft() {
  digitalWrite(M1_PHASE, HIGH);
  analogWrite(M1_ENABLE, motorSpeed);
  digitalWrite(M2_PHASE, HIGH);
  analogWrite(M2_ENABLE, motorSpeed);
}

void stop() {
  analogWrite(M1_ENABLE, 0);
  analogWrite(M2_ENABLE, 0);
}
