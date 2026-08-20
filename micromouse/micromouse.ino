#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "VelocityController.hpp"
#include <Wire.h>
#include <MPU6050_light.h>

//Motors
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
#define MAX_PWM 130
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
mtrn3100::EncoderOdometry encoder_odometry(16, 100);

//Encoders
#define MOT1ENCA 2
#define MOT1ENCB 7
#define MOT2ENCA 3
#define MOT2ENCB 8
mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB, true);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

//Controllers
mtrn3100::PIDController controller(200, 2, 10);
mtrn3100::VelocityController leftVelocityController(15, 0, 0, MAX_PWM);
mtrn3100::VelocityController rightVelocityController(15, 0, 0, MAX_PWM);

// IMU
MPU6050 mpu(Wire);

//Constants
#define TARGET_RAD 6.28
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 100.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)

struct Waypoint {
  float x, y;
};

void setup() {
  delay(1000);
  Serial.begin(115200);
  Wire.begin();
  mpu.begin();
  Serial.println("Calibrating IMU, keep still...");
  delay(1000);
  mpu.calcOffsets();
  Serial.println("Done");
  delay(2000);
}

// set to the actual provided starting orientation before each maze
#define START_HEADING_DEG -180.0

void loop() {
  // Waypoints are already given in the maze's global frame, use as-is
  Waypoint path[] = {
    {270, 270},
    {630, 270},
    {630, 90},
    {1170, 90},
    {1170, 270},
    {810, 270}
  };

  // int numberOfWaypoints = sizeof(path) / sizeof(path[0]);
  // int final_heading_deg = 90.0;

  // encoder_odometry.setPosition(path[0].x, path[0].y);    // start where this path says we start
  // resyncHeadingToKnown(START_HEADING_DEG * PI / 180.0);  // and facing where we're told we start
  // purePursuitPath(path, numberOfWaypoints, final_heading_deg);

  // for (int i = 0; i < sizeof(path); i++) { 
  //   navigateTo(path[i].x, path[i].y, encoder_odometry.getH());
  // }

  while (true) {};
}

float imuHeadingOffsetRad = 0;

void updateOdometry() {
  mpu.update();
  // NOTE: if fused heading drifts the opposite way to the encoder-only heading, this mounting
  // has gyro-Z reading CW-positive - negate it here.
  float imuHeadingRad = mpu.getAngleZ() * PI / 180.0 - imuHeadingOffsetRad;
  encoder_odometry.update(encoder1.getRotation(), encoder2.getRotation(), imuHeadingRad);
}

void resyncHeadingToKnown(float knownHeadingRad) {
  encoder_odometry.setHeading(knownHeadingRad);
  imuHeadingOffsetRad = mpu.getAngleZ() * PI / 180.0 - knownHeadingRad;
}

void serialPrint() {
  Serial.print("ODOM:\t\t X = ");
  Serial.print(encoder_odometry.getX());
  Serial.print(",\t\t Y = ");
  Serial.print(encoder_odometry.getY());
  Serial.print(",\t\t H = ");
  Serial.print(encoder_odometry.getH());
  Serial.print(",\t\t DeltaLeft = ");
  Serial.print(encoder_odometry.getDeltaLeft());
  Serial.print(",\t\t DeltaRight = ");
  Serial.print(encoder_odometry.getDeltaRight());
  Serial.println();
}

void driveForward(float distance) {
  float startLeft = encoder1.getRotation();
  float startRight = encoder2.getRotation();
  controller.integral = 0;
  controller.prev_error = 0;

  float target = distance / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float leftRot = encoder1.getRotation() - startLeft;
    float rightRot = encoder2.getRotation() - startRight;
    float avgRot = (leftRot + rightRot) / 2.0;

    updateOdometry();  // global pose
    serialPrint();

    int pwm = controller.compute(avgRot);

    motor1.setPWM(-pwm);
    motor2.setPWM(pwm);

    if (abs(controller.error) < 0.1) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }

    delay(10);
  }
}

void turnToHeading(float target_deg) {
  controller.integral = 0;
  controller.prev_error = 0;

  float startLeft = encoder1.getRotation();
  float startRight = encoder2.getRotation();

  // get current heading from odometry in degrees
  float current_deg = encoder_odometry.getH() * 180.0 / PI;
  if (current_deg > 180.0) current_deg -= 360.0;
  if (current_deg < -180.0) current_deg += 360.0;

  float delta = target_deg - current_deg;
  while (delta > 180.0) delta -= 360.0;
  while (delta < -180.0) delta += 360.0;
  if (delta > 175.0) delta -= 360.0;
  if (delta < -175.0) delta += 360.0;

  Serial.print("turnToHeading: target=");
  Serial.print(target_deg);
  Serial.print(" current=");
  Serial.print(current_deg);
  Serial.print(" delta=");
  Serial.println(delta);

  if (abs(delta) < 2.0) return;  // already close enough

  float turn_angle = delta * PI / 180.0;
  float arc = (DISTANCE_BETWEEN_WHEELS / 2.0) * turn_angle;
  float target_rot = arc / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target_rot);

  while (true) {
    float leftRot = encoder1.getRotation() - startLeft;
    float rightRot = encoder2.getRotation() - startRight;
    float avg = (rightRot - leftRot) / 2.0;

    updateOdometry();

    int pwm = controller.compute(avg);

    Serial.print("avg=");
    Serial.print(avg);
    Serial.print(" error=");
    Serial.print(controller.error);
    Serial.print(" pwm=");
    Serial.println(pwm);

    if (delta > 0) {
      motor1.setPWM(abs(pwm));
      motor2.setPWM(abs(pwm));
    } else {
      motor1.setPWM(-abs(pwm));
      motor2.setPWM(-abs(pwm));
    }

    if (abs(controller.error) < 0.1) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }
    delay(10);
  }
}

void navigateTo(float tx, float ty, float th_deg) {
  // rotate to face waypoint
  float dx = tx - encoder_odometry.getX();
  float dy = ty - encoder_odometry.getY();
  float angle_to_target_deg = atan2(dy, dx) * 180.0 / PI;
  while (th_deg > 180.0) th_deg -= 360.0;
  while (th_deg < -180.0) th_deg += 360.0;

  turnToHeading(angle_to_target_deg);

  // drive straight
  float distance = sqrt(dx * dx + dy * dy);
  driveForward(distance);

  // rotate to final heading
  turnToHeading(th_deg);
}

void purePursuitPath(Waypoint* waypoints, int num_waypoints, float final_heading_deg) {
  const float LOOKAHEAD = 10.0;
  const float TARGET_WHEEL_SPEED = 10.0;
  const float GOAL_TOLERANCE = 5.0;

  int current_segment = 0;  // which segment we're tracking
  leftVelocityController.reset(encoder1.getRotation());
  rightVelocityController.reset(encoder2.getRotation());

  while (true) {
    updateOdometry();

    float rx = encoder_odometry.getX();
    float ry = encoder_odometry.getY();
    float rh = encoder_odometry.getH();

    // final goal is the last waypoint
    float final_x = waypoints[num_waypoints - 1].x;
    float final_y = waypoints[num_waypoints - 1].y;
    float dist_to_goal = sqrt((final_x - rx) * (final_x - rx) + (final_y - ry) * (final_y - ry));
    if (dist_to_goal < GOAL_TOLERANCE) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }

    // advance segment if robot has passed current waypoint
    if (current_segment < num_waypoints - 2) {
      float wx = waypoints[current_segment + 1].x;
      float wy = waypoints[current_segment + 1].y;
      float dist_to_next = sqrt((wx - rx) * (wx - rx) + (wy - ry) * (wy - ry));
      if (dist_to_next < LOOKAHEAD) {
        current_segment++;
      }
    }

    // find lookahead point on current segment
    float sx = waypoints[current_segment].x;
    float sy = waypoints[current_segment].y;
    float ex = waypoints[current_segment + 1].x;
    float ey = waypoints[current_segment + 1].y;

    float seg_dx = ex - sx;
    float seg_dy = ey - sy;
    float seg_len = sqrt(seg_dx * seg_dx + seg_dy * seg_dy);

    // if heading change is too big for curvature to react to (e.g. <90)
    const float TURN_THRESHOLD_DEG = 45;
    float seg_heading_deg = atan2(seg_dy, seg_dx) * 180.0 / PI;
    float current_heading_deg = rh * 180.0 / PI;
    float heading_gap = seg_heading_deg - current_heading_deg;
    while (heading_gap > 180.0) heading_gap -= 360.0;
    while (heading_gap < -180.0) heading_gap += 360.0;
    if (fabs(heading_gap) > TURN_THRESHOLD_DEG) {
      turnToHeading(seg_heading_deg);
      leftVelocityController.reset(encoder1.getRotation());
      rightVelocityController.reset(encoder2.getRotation());
      continue;
    }

    // project robot position onto segment
    float t = ((rx - sx) * seg_dx + (ry - sy) * seg_dy) / (seg_len * seg_len);
    t = constrain(t, 0.0f, 1.0f);

    // lookahead point along segment
    float lookahead_t = t + LOOKAHEAD / seg_len;
    lookahead_t = constrain(lookahead_t, 0.0f, 1.0f);
    float lx = sx + lookahead_t * seg_dx;
    float ly = sy + lookahead_t * seg_dy;

    // transform to robot local frame
    float cos_h = cos(rh);
    float sin_h = sin(rh);
    float local_x = cos_h * (lx - rx) + sin_h * (ly - ry);
    float local_y = -sin_h * (lx - rx) + cos_h * (ly - ry);

    // curvature -> target wheel angular velocities (reference generator's job stops here)
    float l = sqrt(local_x * local_x + local_y * local_y);
    if (l < 1.0) l = 1.0;  // guard against the lookahead point collapsing onto the robot near a waypoint
    float curvature = 2.0 * local_y / (l * l);

    // clamp so the inner wheel target never flips sign (min turn radius = DISTANCE_BETWEEN_WHEELS/2)
    const float MAX_CURVATURE = 2.0 / DISTANCE_BETWEEN_WHEELS;
    curvature = constrain(curvature, -MAX_CURVATURE, MAX_CURVATURE);

    // slow down for corners
    const float CORNER_SLOWDOWN = 0.1;  // fraction of speed shed at max curvature (0=no slowdown, 1=full stop)
    float curvature_frac = fabs(curvature) / MAX_CURVATURE;
    float target_speed = TARGET_WHEEL_SPEED * (1.0 - CORNER_SLOWDOWN * curvature_frac);

    float left_wheel_speed  = target_speed * (1.0 - curvature * DISTANCE_BETWEEN_WHEELS / 2.0);
    float right_wheel_speed = target_speed * (1.0 + curvature * DISTANCE_BETWEEN_WHEELS / 2.0);

    if (fabs(curvature) == 0) {
      resyncHeadingToKnown(atan2(seg_dy, seg_dx));
    }

    // velocity controller's job: hit that target using measured wheel speed, not an open-loop guess
    int left_pwm = leftVelocityController.compute(encoder1.getRotation(), left_wheel_speed);
    int right_pwm = rightVelocityController.compute(encoder2.getRotation(), right_wheel_speed);

    motor1.setPWM(-left_pwm);
    motor2.setPWM(right_pwm);

    serialPrint();
    Serial.print("L: target=");
    Serial.print(left_wheel_speed);
    Serial.print(" measured=");
    Serial.print(leftVelocityController.getMeasuredVelocity());
    Serial.print(" pwm=");
    Serial.print(left_pwm);
    Serial.print("\t\tR: target=");
    Serial.print(right_wheel_speed);
    Serial.print(" measured=");
    Serial.print(rightVelocityController.getMeasuredVelocity());
    Serial.print(" pwm=");
    Serial.println(right_pwm);
    delay(10);
  }

  // final heading correction
  turnToHeading(final_heading_deg);
}
