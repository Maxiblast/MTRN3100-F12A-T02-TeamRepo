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

    static mtrn3100::PIDController wallPID(1.5,0.0,0.3);
    wallPID.zeroAndSetTarget(0, targetDistanceMm);

    while (true) {
        if (frontlidar.timeoutOccured()) {
            continue;
        };

        int currentDistance = frontlidar.readRangeSingleMillimeters();

        int pwm = -wallPID.compute(currentDistance);

        motor1.setPWM(pwm);
        motor2.setPWM(pwm);

        if (fabs(wallPID.getError()) <= toleranceMm) {
            motor1.setPWM(0);
            motor2.setPWM(0);
        }

    }

}

void turnImplementation() {

// Need to choose 1 method Lidar or IMU

}

void chainMovements(const std::string commandLetter) {

// Implement each letter logic for robot movement
for (char command : commandLetter) {
    switch (command) {
        case 'f':

            break;
        
        case 'l':

            break;

        case 'r':
        
            break;
    }
}  
}
