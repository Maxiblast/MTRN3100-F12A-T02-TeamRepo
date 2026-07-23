#pragma once
#include <Arduino.h>
#include <math.h>
#include "Motor.hpp"
#include "PIDController.hpp"
#include "VL6180X.h"

// Globals defined in task2.ino
extern mtrn3100::Motor motor1;
extern mtrn3100::Motor motor2;
extern VL6180X frontlidar;


// void straightLineTracking(int targetDistance = 1) {

// /* Record initial IMU
//     Start loop until it reahces the target position
//     Keeping centre withing +/- 100mm
//     */

// }

void maintainWallDistance(int targetDistanceMm = 100, int toleranceMm = 5) {

    static mtrn3100::PIDController wallPID(100,2,10);
    wallPID.zeroAndSetTarget(0, targetDistanceMm);

    while (true) {
        int currentDistance = frontlidar.readRangeSingleMillimeters();
        
        if (frontlidar.timeoutOccurred()) {
            Serial.println("Lidar timeout!");
            continue;
        };

        int pwm = -wallPID.compute(currentDistance);

        //debug
        // Serial.print("Lidar (mm): ");
        // Serial.print(currentDistance);
        // Serial.print(" | Error: ");
        // Serial.print(wallPID.getError());
        // Serial.print(" | PWM: ");
        // Serial.println(pwm);

        motor1.setPWM(-pwm);
        motor2.setPWM(pwm);
        
        if (abs(wallPID.getError()) <= toleranceMm) {
            motor1.setPWM(0);
            motor2.setPWM(0);
            continue;
        }

    }

}
// void turnImplementation() {

// // Need to choose 1 method Lidar or IMU

// }

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
