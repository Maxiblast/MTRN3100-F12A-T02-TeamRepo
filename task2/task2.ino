//Constants
#define MOVEDIST 180

//Motors 
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

//Encoders
#define MOT1ENCA 2 // Replaced EN_A PIN 2 is an interupt
#define MOT1ENCB 7 // Replaced EN_B
#define MOT2ENCA 3
#define MOT2ENCB 8

//Robot Dimensions
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 85.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)


#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB, true);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

//Controllers
mtrn3100::PIDController controller(200,2,10);

void chainMovements(String commandLetter);
void driveForward(float distance);
void turnLeft90();
void turnRight90();

void setup() {
  Serial.begin(9600);
  delay(3000);
  chainMovements("rrrr"); 
  chainMovements("llll"); 
}

void loop() {

}

void chainMovements(String commandLetter) {
    for (char command : commandLetter) {
        switch (command) {
            case 'f':
                driveForward(MOVEDIST);
                delay(1000);   
                break;
            case 'l':
                turnLeft90();
                delay(1000);    // need to replace this with IMU + Encoder turning. 
                break;
            case 'r':
                turnRight90();
                delay(1000);
                break;
            case 'b':
                driveForward(-MOVEDIST);
                delay(1000);
                break;
        }
    }  
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



