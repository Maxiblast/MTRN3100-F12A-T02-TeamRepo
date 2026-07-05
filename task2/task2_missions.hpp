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
