#pragma once

#include <iostream>
#include <string>

void straightLineTracking(int targetDistance = 1) {

/* Record initial IMU
    Start loop until it reahces the target position
    Keeping centre withing +/- 100mm
    */

}

void maintainWallDistance(int targetDistanceMm = 100, int toleranceMm = 5) {

/* Track lidar distance by a loop
    Error = currentLidar - targetDistacneMm (not sure)
    Drive forward if error is positive and backwards if error is negative
    Make sure to reach the desired state in 10 seconds
    */
}

void turnImplementation() {

// Need to choose 1 method Lidar or IMU

}

void chainMovements(const std::string commandLetter) {
    for (char command : commandLetter) {
        switch (command) {
            case 'f':
                driveForward(200);   
                break;
            case 'l':
                turnLeft90();    // need to replace this with IMU + Encoder turning. 
                break;
            case 'r':
                turnRight90();
                break;
            case 'b':
                driveForward(-200);
                break;
        }
    }  
}

//HELPER FUNCTION

void driveForward(float distance) {
  encoder1.count = 0;
  encoder2.count = 0;
  controller.integral = 0;
  controller.prev_error = 0;

  float target = distance / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float avg = (encoder2.getRotation() + encoder1.getRotation()) / 2.0;
    int pwm = controller.compute(avg);

    Serial.print("enc1: "); Serial.print(encoder1.getRotation());
    Serial.print(" enc2: "); Serial.print(encoder2.getRotation());
    Serial.print(" avg: "); Serial.print(avg);
    Serial.print(" error: "); Serial.println(controller.error);

    motor1.setPWM(-pwm);
    motor2.setPWM(pwm);

    if (abs(controller.error) < 0.1) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
  }
}

void turnLeft90() {
  // Zero both encoders
  encoder1.count = 0;
  encoder2.count = 0;
  controller.integral = 0;
  controller.prev_error = 0;

  float turn_angle = PI / 2.0;                                     // 90 degrees
  float arc = (DISTANCE_BETWEEN_WHEELS / 2.0) * turn_angle;        // arc length formula (radius * angle)
  float target = arc / WHEEL_RADIUS;                               // divide arc length by radius to get radians
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float avg = (encoder2.getRotation() - encoder1.getRotation()) / 2.0;
    int pwm = controller.compute(avg);

    motor1.setPWM(pwm);
    motor2.setPWM(pwm);

    if (abs(controller.error) < 0.1) {  // within ~0.1 rad tolerance
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
  }
}

void turnRight90() {
  // Zero both encoders
  encoder1.count = 0;
  encoder2.count = 0;
  controller.integral = 0;
  controller.prev_error = 0;

  float turn_angle = -PI / 2.0;                                     // -90 degrees
  float arc = (DISTANCE_BETWEEN_WHEELS / 2.0) * turn_angle;        // arc length formula (radius * angle)
  float target = arc / WHEEL_RADIUS;                               // divide arc length by radius to get radians
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float avg = (encoder2.getRotation() - encoder1.getRotation()) / 2.0;
    int pwm = controller.compute(avg);

    motor1.setPWM(pwm);
    motor2.setPWM(pwm);

    if (abs(controller.error) < 0.1) {  // within ~0.1 rad tolerance
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
  }
}


