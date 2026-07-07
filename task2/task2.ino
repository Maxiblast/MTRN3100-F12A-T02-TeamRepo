#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "VL6180X.h"
#include "Wire.h"

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

//Controllers
// mtrn3100::PIDController controller(200,2,10);

//Constants
#define TARGET_RAD 6.28
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 85.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)

//lidar
VL6180X frontlidar;
#include "task2_missions.hpp"

//when more lidars added we need to also add a .setAddress 


void setup() {
  Serial.begin(9600);
  
  //lidar things
  Wire.begin();
  frontlidar.init();
  frontlidar.configureDefault();
  frontlidar.setTimeout(500);

  maintainWallDistance();
}

void loop() {
 
}

