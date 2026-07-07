#pragma once
#include <MPU6050_light.h>
MPU6050 mpu(Wire);

// COnstantssss
extern mtrn3100::Motor motor1;
extern mtrn3100::Motor motor2;
extern mtrn3100::Encoder encoder1;
extern mtrn3100::Encoder encoder2;
extern float refYaw;

const int baseSpeed = 140;
const int minSpeed = 0;
const int maxSpeed = 255;

const float WHEEL_DIAMETER_MM = 32.0;
const float WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2.0;

const int MOTOR1_SIGN = -1;
const int MOTOR2_SIGN = 1;
const float MOTOR2_TRIM = 1.0;  

float kp = 1.6;     
float ki = 0.02;    

float refYaw = 0;

// // HELPER FUNCTION ‼
float angleDiff(float target, float current) {
    float diff = target - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
}

void drive(int leftSpeed, int rightSpeed) {
    motor1.setPWM(MOTOR1_SIGN * leftSpeed);
    motor2.setPWM(int(MOTOR2_SIGN * rightSpeed * MOTOR2_TRIM));
}

void stop() {
    drive(0, 0);
}

// MISSIONS
void straightLineTracking() {
    const float targetDistance = 1000.0;  
    const unsigned long timeout = 30000;  

    unsigned long startTime = millis();
    float integralError = 0;

   
    float x = 0, y = 0;
    float lastAvgDist = 0;

    const float kLateral = 0.15;   

    while (millis() - startTime < timeout) {

        mpu.update();
        float currYaw = mpu.getAngleZ();

        float leftDist  = encoder1.getRotation() * WHEEL_RADIUS_MM;
        float rightDist = encoder2.getRotation() * WHEEL_RADIUS_MM;
        float avgDist = (leftDist + rightDist) / 2.0;

        float deltaDist = avgDist - lastAvgDist;
        lastAvgDist = avgDist;

        float headingRad = (currYaw - refYaw) * PI / 180.0; 
        x += deltaDist * cos(headingRad);
        y += deltaDist * sin(headingRad);

        float desiredHeadingOffset = -kLateral * y;
        desiredHeadingOffset = constrain(desiredHeadingOffset, -20.0, 20.0); 

        float error = angleDiff(refYaw + desiredHeadingOffset, currYaw);

        if (abs(error) < 1.0) error = 0;

        integralError += error * 0.01;
        integralError = constrain(integralError, -20, 20);

        float correction = kp * error + ki * integralError;
        correction = constrain(correction, -40, 40);

        int leftSpeed  = constrain(baseSpeed - correction, minSpeed, maxSpeed);
        int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);

        drive(leftSpeed, rightSpeed);

        if (avgDist >= targetDistance) break;

        delay(10);
    }

    stop();
}

// void maintainWallDistance(int targetDistanceMm = 100, int toleranceMm = 5) {
// /* Track lidar distance by a loop
//     Error = currentLidar - targetDistacneMm (not sure)
//     Drive forward if error is positive and backwards if error is negative
//     Make sure to reach the desired state in 10 seconds
//     */
// }

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

// // HELPER FUNCTION ‼

// void maintainWallDistance(int targetDistanceMm = 100, int toleranceMm = 5) {

// /* Track lidar distance by a loop
//     Error = currentLidar - targetDistacneMm (not sure)
//     Drive forward if error is positive and backwards if error is negative
//     Make sure to reach the desired state in 10 seconds
//     */
// }

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