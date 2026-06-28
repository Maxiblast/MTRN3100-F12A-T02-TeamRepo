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
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define MOT1ENCA 2 // Replaced EN_A PIN 2 is an interupt
#define MOT1ENCB 7 // Replaced EN_B
#define MOT2ENCA 3
#define MOT2ENCB 8
mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB, true);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

#define TARGET_RAD 6.28
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 85.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)


// mtrn3100::BangBangController controller(40,1.0);  //speed 50 , deadband 10. 
mtrn3100::PIDController controller(200,2,10);


void setup() {
  Serial.begin(9600);
  // controller.zeroAndSetTarget(encoder.getRotation(), TARGET_RAD); // Set the target as 2 Radians
  delay(2000);
}

void loop() {
  // int val = controller.compute(encoder.getRotation());
  // motor1.setPWM(val);
  // Serial.print("Count1: ");
  // Serial.print(encoder1.count);
  // Serial.print(" Count2: ");
  // Serial.println(encoder2.count);
  // Serial.print(" Rotation: ");
  // Serial.print(encoder.getRotation());
  // Serial.print(" Error: ");
  // Serial.print(controller.error);
  // Serial.print(" PWM Output: ");
  // Serial.println(val);
  
  driveForward(200); // input: distance in mm
  delay(500);
  for (int i = 0; i < 4; i++) {
    turnLeft90();
    delay(200);
  }
  for (int i = 0; i < 4; i++) {
    turnRight90();
    delay(200);
  }
  while (true) {};
}

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


