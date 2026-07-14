#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
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
mtrn3100::EncoderOdometry encoder_odometry(16,85);

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
  driveForward(200);
  turn(90);
}

void updateOdometry() {
  encoder_odometry.update(encoder1.getRotation(), encoder2.getRotation());
}

void serialPrint() {
  Serial.print("ODOM:\t\t X = ");
  Serial.print(encoder_odometry.getX());
  Serial.print(",\t\t Y = ");
  Serial.print(encoder_odometry.getY());
  Serial.print(",\t\t H = ");
  Serial.print(encoder_odometry.getH());
  Serial.print(",\t\t DeltaLeft = ");
  Serial.print(encoder_odometry.getDeltaLeft());
  Serial.print(",\t\t DeltaRight = ");
  Serial.print(encoder_odometry.getDeltaRight());
  Serial.println();
}

void driveForward(float distance) {
  encoder1.count = 0;
  encoder2.count = 0;
  controller.integral = 0;
  controller.prev_error = 0;

  float target = distance / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float leftRot = encoder1.getRotation();
    float rightRot = encoder2.getRotation();
    float avg = (leftRot + rightRot) / 2.0;

    updateOdometry();
    serialPrint();

    int pwm = controller.compute(avg);

    motor1.setPWM(-pwm);
    motor2.setPWM(pwm);

    if (abs(controller.error) < 0.1) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }

    delay(10);
  }
}

void turn(float angleDegrees) {
  encoder1.count = 0;
  encoder2.count = 0;
  controller.integral = 0;
  controller.prev_error = 0;

  float turn_angle = angleDegrees * PI / 180.0;
  float arc = (DISTANCE_BETWEEN_WHEELS / 2.0) * turn_angle;
  float target = arc / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float avg = (encoder2.getRotation() - encoder1.getRotation()) / 2.0;

    updateOdometry();
    serialPrint();

    int pwm = controller.compute(avg);

    motor1.setPWM(pwm);
    motor2.setPWM(pwm);

    if (abs(controller.error) < 0.1) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
  }
}
