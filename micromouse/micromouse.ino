#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include <Wire.h>
#include <MPU6050_light.h>


//Motors 
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
mtrn3100::EncoderOdometry encoder_odometry(16,100);

//Encoders
#define MOT1ENCA 2
#define MOT1ENCB 7
#define MOT2ENCA 3
#define MOT2ENCB 8
mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB, true);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

//Controllers
mtrn3100::PIDController controller(200,2,10);

// IMU
MPU6050 mpu(Wire);

//Constants
#define TARGET_RAD 6.28
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 100.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)



void setup() {
  delay(1000);
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
  navigateTo(180, 0, 0);
  navigateTo(180, 180, 0);
  navigateTo(360, 180, 180);
  while (true) {};
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
  float startLeft = encoder1.getRotation();
  float startRight = encoder2.getRotation();
  controller.integral = 0;
  controller.prev_error = 0;

  float target = distance / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float leftRot = encoder1.getRotation() - startLeft;
    float rightRot = encoder2.getRotation() - startRight;
    float avgRot = (leftRot + rightRot) / 2.0;

    updateOdometry();  // global pose
    serialPrint();

    int pwm = controller.compute(avgRot);

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

void turnToHeading(float target_deg) {
  controller.integral = 0;
  controller.prev_error = 0;

  float startLeft = encoder1.getRotation();
  float startRight = encoder2.getRotation();

  // get current heading from odometry in degrees
  float current_deg = encoder_odometry.getH() * 180.0 / PI;
  if (current_deg > 180.0) current_deg -= 360.0;
  if (current_deg < -180.0) current_deg += 360.0;
  
  float delta = target_deg - current_deg;
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;
  if (delta > 175.0) delta -= 360.0;
  if (delta < -175.0) delta += 360.0;

  Serial.print("turnToHeading: target="); Serial.print(target_deg);
  Serial.print(" current="); Serial.print(current_deg);
  Serial.print(" delta="); Serial.println(delta);

  if (abs(delta) < 2.0) return;  // already close enough

  float turn_angle = delta * PI / 180.0;
  float arc = (DISTANCE_BETWEEN_WHEELS / 2.0) * turn_angle;
  float target_rot = arc / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target_rot);

  while (true) {
    float leftRot = encoder1.getRotation() - startLeft;
    float rightRot = encoder2.getRotation() - startRight;
    float avg = (rightRot - leftRot) / 2.0;

    updateOdometry();

    int pwm = controller.compute(avg);

    Serial.print("avg="); Serial.print(avg);
    Serial.print(" error="); Serial.print(controller.error);
    Serial.print(" pwm="); Serial.println(pwm);

    if (delta > 0) {
      motor1.setPWM(abs(pwm));
      motor2.setPWM(abs(pwm));
    } else {
      motor1.setPWM(-abs(pwm));
      motor2.setPWM(-abs(pwm));
    }

    if (abs(controller.error) < 0.1) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
    delay(10);
  }
}

void navigateTo(float tx, float ty, float th_deg) {
  // rotate to face waypoint
  float dx = tx - encoder_odometry.getX();
  float dy = ty - encoder_odometry.getY();
  float angle_to_target_deg = atan2(dy, dx) * 180.0 / PI;
  while (th_deg > 180.0) th_deg -= 360.0;
  while (th_deg < -180.0) th_deg += 360.0;

  turnToHeading(angle_to_target_deg);

  // drive straight
  float distance = sqrt(dx * dx + dy * dy);
  driveForward(distance);

  // rotate to final heading
  turnToHeading(th_deg);
}

