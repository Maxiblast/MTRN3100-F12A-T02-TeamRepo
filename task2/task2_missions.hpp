#pragma once

#include <Arduino.h>
#include <MPU6050_light.h>

// void straightLineTracking(int targetDistance = 1) {

// /* Record initial IMU
//     Start loop until it reahces the target position
//     Keeping centre withing +/- 100mm
//     */

// }

// void maintainWallDistance(int targetDistanceMm = 100, int toleranceMm = 5) {

// /* Track lidar distance by a loop
//     Error = currentLidar - targetDistacneMm (not sure)
//     Drive forward if error is positive and backwards if error is negative
//     Make sure to reach the desired state in 10 seconds
//     */
// }

//------------------------ Turning Task ------------------------//

extern MPU6050 mpu;
extern mtrn3100::Motor motor1;
extern mtrn3100::Motor motor2;
extern mtrn3100::PIDController controller;

float targetYaw = 0;

// Calculate error = target - current angle
float angleError(float target, float current) {
  float e = target - current;

  // Bound between [-180, 180]
  while (e > 180.0) e -= 360.0;
  while (e < -180.0) e += 360.0;

  return e;
}

void turnIMU(float degrees) {
  mpu.update();
  targetYaw = mpu.getAngleZ() + degrees;
  while (targetYaw > 180.0) targetYaw -= 360.0;
  while (targetYaw < -180.0) targetYaw += 360.0;

  controller.integral = 0;
  controller.prev_error = 0;

  while (true) {
    mpu.update();
    float error = angleError(targetYaw, mpu.getAngleZ());
    int pwm = constrain((int)(controller.kp * error), -100, 100);
    Serial.print("Error: ");
    Serial.println(error);

    motor1.setPWM(pwm);
    motor2.setPWM(pwm);

    if (abs(error) < 2.0) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
  }
}

void holdHeading() {
  mpu.update();
  float error = angleError(targetYaw, mpu.getAngleZ());
  int pwm = constrain((int)(controller.kp * error), -100, 100);

  if (abs(error) > 5.0) {
    motor1.setPWM(pwm);
    motor2.setPWM(pwm);
  } else {
    motor1.setPWM(0);
    motor2.setPWM(0);
  }
}

void turnImplementation() {

// Challenge 1: 90 degree clockwise turn
  turnIMU(-90.0);

  // Challenge 2: hold heading indefinitely
  while (true) {
    holdHeading();
  }

}

//--------------------------------------------------------------//

// void chainMovements(const std::string commandLetter) {

// // Implement each letter logic for robot movement
// for (char command : commandLetter) {
//     switch (command) {
//         case 'f':

//             break;
        
//         case 'l':

//             break;

//         case 'r':
        
//             break;
//     }
// }  
// }
