#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"

//MODIFIED TO WORK ON THE MOUSE. 

#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
// mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define MOT1ENCA 2 // Replaced EN_A PIN 2 is an interupt
#define MOT1ENCB 7 // Replaced EN_B
#define MOT2ENCA 3
#define MOT2ENCB 8

#define TARGET_RAD 6.28
mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

// mtrn3100::BangBangController controller(40,1.0);  //speed 50 , deadband 10. 
mtrn3100::PIDController controller(200,0,0);


void setup() {
  Serial.begin(9600);
  // controller.zeroAndSetTarget(encoder.getRotation(), TARGET_RAD); // Set the target as 2 Radians
  delay(2000);
}

void loop() {
  // int val = controller.compute(encoder.getRotation());
  // motor1.setPWM(val);
  Serial.print("Count1: ");
  Serial.print(encoder1.count);
  Serial.println(" Count2: ");
  // Serial.println(encoder2.count);
  // Serial.print(" Rotation: ");
  // Serial.print(encoder.getRotation());
  // Serial.print(" Error: ");
  // Serial.print(controller.error);
  // Serial.print(" PWM Output: ");
  // Serial.println(val);
}
