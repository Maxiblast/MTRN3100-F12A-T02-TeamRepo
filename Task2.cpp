 #include <iostream>
 #include <string>
#include "mtrn3100task1V2.h"

//  int baseSpeed = 70;        
// float Kp = 3.0;            
// float Ki = 0.0;            
// int minSpeed = 0;
// int maxSpeed = 255;

float refYaw = 0;
float integralError = 0;
unsigned long startTime = 0;

void loop() {
  mpu.update();
  float currYaw = mpu.getAngleZ();
  float error = angleDiff(refYaw, currYaw);  // + = drifted one way, - = drifted other way

  unsigned long elapsed = millis() - startTime;

  if (elapsed >= driveTimeMs || elapsed >= maxRunTimeMs) {
    //stop();
    Serial.println("Finished straight-line run.");
        while (true) { 
        }
    }
    straightLineTracking(error);
    
     //delay(20); 
}


 void straightLineTracking(float error) {

    /* Record initial IMU
       Start loop until it reahces the target position
       Keeping centre withing +/- 100mm
       */

    integralError += error * 0.02; // 20ms maybe
    integralError = constrain(integralError, -50, 50); 

//    float correction = (Kp * error + Ki * integralError);

    int leftSpeed  = constrain(baseSpeed - correction, minSpeed, maxSpeed);
    int rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);

    // Forward direction
    digitalWrite(M1_PHASE, LOW);
    digitalWrite(M2_PHASE, HIGH);

    analogWrite(M1_ENABLE, leftSpeed);
    analogWrite(M2_ENABLE, rightSpeed);

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
