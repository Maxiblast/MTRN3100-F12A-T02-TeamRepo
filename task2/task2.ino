#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "task2_missions.hpp"
#include <Wire.h>
#include <MPU6050_light.h>

//Motors 
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

//Encoders
#define MOT1ENCA 2 // Replaced EN_A PIN 2 is an interupt
#define MOT1ENCB 7 // Replaced EN_B
#define MOT2ENCA 3
#define MOT2ENCB 8
mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB, true);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

// IMU
MPU6050 mpu(Wire);

//Controllers
mtrn3100::PIDController controller(200,2,10);

//Constants
#define TARGET_RAD 6.28
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 85.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  Serial.println("Calibrating IMU, keep still...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done");
  delay(2000);
}

void loop() {
  turnImplementation();
}

