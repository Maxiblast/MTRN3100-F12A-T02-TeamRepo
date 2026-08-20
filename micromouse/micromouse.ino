#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "VelocityController.hpp"
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>

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

// LiDARs: J14 -> A2, J15 -> A1, J16 -> A0 (from the kit schematic)
VL6180X lidar[3];
const uint8_t LIDAR_ENABLE_PINS[3] = {A2, A1, A0};
const uint8_t LIDAR_ADDRESSES[3] = {0x30, 0x31, 0x32};

// Change these numbers if your physical LiDAR plugs are different.
// This assumes J14 = left, J15 = front, J16 = right.
const int LEFT_LIDAR = 0;
const int FRONT_LIDAR = 1;
const int RIGHT_LIDAR = 2;

// ======================================================
//              LIDAR ODOM RESET TUNING

const float RESET_LEFT_CENTRED_MM = 50.0; //fix this measure tmr
const float RESET_FRONT_CENTRED_MM = 50.0;
const float RESET_RIGHT_CENTRED_MM = 50.0;

const float RESET_WALL_PRESENT_MAX_MM = 105.0;

// Reject a LiDAR correction larger than this.
const float RESET_MAX_OFFSET_MM = 35.0;
// ======================================================

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
  setupLidars();
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
    {1170, 450},
    {843, 483},
    {663, 597},
    {597, 777},
    {483, 957},
    {237, 1317},
    {90, 1350}
  };

  int numberOfWaypoints = sizeof(path) / sizeof(path[0]);
  int final_heading_deg = 90.0;

  encoder_odometry.setPosition(path[0].x, path[0].y);    // start where this path says we start
  resyncHeadingToKnown(START_HEADING_DEG * PI / 180.0);  // and facing where we're told we start

  // Face perpendicular to whichever wall is currently closest.
  centerToNearestWall();

  // Robot is stationary. Correct its position around the known starting-cell
  // centre while keeping the path in global coordinates.
  resetOdometryFromLidar(
    path[0].x,
    path[0].y,
    START_HEADING_DEG
  );

  purePursuitPath(path, numberOfWaypoints, final_heading_deg);

  while (true) {};
}

float imuHeadingOffsetRad = -180;

void setupLidars() {
  // Hold every VL6180X disabled so they do not all answer at address 0x29.
  for (int i = 0; i < 3; i++) {
    pinMode(LIDAR_ENABLE_PINS[i], OUTPUT);
    digitalWrite(LIDAR_ENABLE_PINS[i], LOW);
  }
  delay(20);

  // Enable one at a time and immediately give it a unique address.
  for (int i = 0; i < 3; i++) {
    digitalWrite(LIDAR_ENABLE_PINS[i], HIGH);
    delay(10);

    lidar[i].setTimeout(80);
    if (!lidar[i].init()) {
      Serial.print("LiDAR failed on J");
      Serial.println(14 + i);
      continue;
    }

    lidar[i].configureDefault();
    lidar[i].setAddress(LIDAR_ADDRESSES[i]);
  }
}

float filteredLidarReading(int sensor) {
  const int SAMPLE_COUNT = 5;
  float total = 0;
  int validSamples = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    uint16_t reading = lidar[sensor].readRangeSingleMillimeters();

    // VL6180X is most useful at short micromouse distances.
    if (!lidar[sensor].timeoutOccurred() && reading > 5 && reading < 250) {
      total += reading;
      validSamples++;
    }
    delay(4);
  }

  if (validSamples < 3) return 9999.0;
  return total / validSamples;
}

bool resetOdometryFromLidar(
  float cellCentreX,
  float cellCentreY,
  float knownHeadingDeg
) {
  // ASSUMPTION: the caller has already stopped and aligned the robot.
  float leftDistance = filteredLidarReading(LEFT_LIDAR);
  float frontDistance = filteredLidarReading(FRONT_LIDAR);
  float rightDistance = filteredLidarReading(RIGHT_LIDAR);

  bool leftWall = leftDistance < RESET_WALL_PRESENT_MAX_MM;
  bool frontWall = frontDistance < RESET_WALL_PRESENT_MAX_MM;
  bool rightWall = rightDistance < RESET_WALL_PRESENT_MAX_MM;

  float heading = knownHeadingDeg * PI / 180.0;

  // Find the current odometry offset relative to the known cell centre.
  // Keep this value on an axis where no usable wall is visible.
  float dx = encoder_odometry.getX() - cellCentreX;
  float dy = encoder_odometry.getY() - cellCentreY;

  float forwardOffset =
    dx * cos(heading) + dy * sin(heading);

  float leftOffset =
    -dx * sin(heading) + dy * cos(heading);

  bool forwardWasReset = false;
  bool sidewaysWasReset = false;

  // Use both side walls when available.
  if (leftWall && rightWall) {
    float offsetMeasuredFromLeft =
      RESET_LEFT_CENTRED_MM - leftDistance;

    float offsetMeasuredFromRight =
      rightDistance - RESET_RIGHT_CENTRED_MM;

    float measuredLeftOffset =
      (offsetMeasuredFromLeft + offsetMeasuredFromRight) / 2.0;

    if (fabs(measuredLeftOffset) <= RESET_MAX_OFFSET_MM) {
      leftOffset = measuredLeftOffset;
      sidewaysWasReset = true;
    }
  }
  // If one side is open, use the wall that still exists.
  else if (leftWall) {
    float measuredLeftOffset =
      RESET_LEFT_CENTRED_MM - leftDistance;

    if (fabs(measuredLeftOffset) <= RESET_MAX_OFFSET_MM) {
      leftOffset = measuredLeftOffset;
      sidewaysWasReset = true;
    }
  }

  else if (rightWall) {
    float measuredLeftOffset =
      rightDistance - RESET_RIGHT_CENTRED_MM;

    if (fabs(measuredLeftOffset) <= RESET_MAX_OFFSET_MM) {
      leftOffset = measuredLeftOffset;
      sidewaysWasReset = true;
    }
  }

  // Only reset the forward offset when the front wall of this cell exists.
  if (frontWall) {
    float measuredForwardOffset =
      RESET_FRONT_CENTRED_MM - frontDistance;

    if (fabs(measuredForwardOffset) <= RESET_MAX_OFFSET_MM) {
      forwardOffset = measuredForwardOffset;
      forwardWasReset = true;
    }
  }

  // Convert robot-local forward/left offsets back into global X/Y.
  float correctedX =
    cellCentreX +
    forwardOffset * cos(heading) -
    leftOffset * sin(heading);

  float correctedY =
    cellCentreY +
    forwardOffset * sin(heading) +
    leftOffset * cos(heading);

  encoder_odometry.setPosition(correctedX, correctedY);
  resyncHeadingToKnown(heading);

  Serial.println("----- LIDAR ODOM RESET -----");

  Serial.print("Left distance: ");
  if (leftWall) Serial.println(leftDistance);
  else Serial.println("OPEN");

  Serial.print("Front distance: ");
  if (frontWall) Serial.println(frontDistance);
  else Serial.println("OPEN");

  Serial.print("Right distance: ");
  if (rightWall) Serial.println(rightDistance);
  else Serial.println("OPEN");

  Serial.print("Forward offset: ");
  Serial.println(forwardOffset);

  Serial.print("Left offset: ");
  Serial.println(leftOffset);

  Serial.print("Corrected X: ");
  Serial.println(correctedX);

  Serial.print("Corrected Y: ");
  Serial.println(correctedY);

  Serial.println("----------------------------");

  return forwardWasReset || sidewaysWasReset;
}

float wallCenteringCurvature(float pathCurvature, float headingErrorDeg) {
  const float TARGET_SIDE_DISTANCE_MM = 50.0; // (180 mm cell - 80 mm robot) / 2
  const float WALL_PRESENT_MAX_MM = 105.0;    // larger means that side is an opening
  const float WALL_KP = 0.00040;              // curvature correction per mm
  const float MAX_WALL_CURVATURE = 0.0060;    // prevents aggressive steering
  const float ERROR_DEADBAND_MM = 2.0;

  // Do not wall-follow while pure pursuit is trying to turn.
  const float MAX_STRAIGHT_HEADING_ERROR_DEG = 8.0;
  const float MAX_STRAIGHT_PATH_CURVATURE = 0.0020;

  static float smoothedCorrection = 0.0;

  bool travellingStraight =
    fabs(headingErrorDeg) < MAX_STRAIGHT_HEADING_ERROR_DEG &&
    fabs(pathCurvature) < MAX_STRAIGHT_PATH_CURVATURE;

  if (!travellingStraight) {
    smoothedCorrection = 0.0;
    return 0.0;
  }

  float leftDistance = filteredLidarReading(LEFT_LIDAR);
  float rightDistance = filteredLidarReading(RIGHT_LIDAR);

  bool leftWall = leftDistance < WALL_PRESENT_MAX_MM;
  bool rightWall = rightDistance < WALL_PRESENT_MAX_MM;
  float sideError = 0.0;

  if (leftWall && rightWall) {
    // Positive means the robot is closer to the right wall, so steer left.
    sideError = (leftDistance - rightDistance) / 2.0;
  } else if (leftWall) {
    // Keep 50 mm from the left wall when the right wall is missing.
    sideError = leftDistance - TARGET_SIDE_DISTANCE_MM;
  } else if (rightWall) {
    // Keep 50 mm from the right wall when the left wall is missing.
    sideError = TARGET_SIDE_DISTANCE_MM - rightDistance;
  } else {
    // With no side wall, smoothly remove any previous correction.
    smoothedCorrection *= 0.6;
    return smoothedCorrection;
  }

  if (fabs(sideError) < ERROR_DEADBAND_MM) sideError = 0.0;

  float requestedCorrection = WALL_KP * sideError;
  requestedCorrection = constrain(
    requestedCorrection,
    -MAX_WALL_CURVATURE,
    MAX_WALL_CURVATURE
  );

  // Low-pass filtering prevents noisy readings or a wall opening from jerking
  // the steering from one loop to the next.
  smoothedCorrection = 0.70 * smoothedCorrection + 0.30 * requestedCorrection;

  Serial.print("Wall L=");
  if (leftWall) Serial.print(leftDistance); else Serial.print("OPEN");
  Serial.print(" R=");
  if (rightWall) Serial.print(rightDistance); else Serial.print("OPEN");
  Serial.print(" correction=");
  Serial.println(smoothedCorrection, 5);

  return smoothedCorrection;
}

void setCenteringTurn(int direction, int pwm) {
  // The motors are mounted in opposite directions, so equal signed PWM rotates
  // the robot on the spot (same convention as turnToHeading()).
  motor1.setPWM(direction * pwm);
  motor2.setPWM(direction * pwm);
}

bool centerToNearestWall() {
  const int TURN_PWM = 42;
  const float NOISE_MM = 2.0;
  const int REQUIRED_RISES = 3;
  const unsigned long MAX_TURN_TIME_MS = 7000;

  // First choose the LiDAR that can currently see the closest valid wall.
  int closestSensor = -1;
  float closestDistance = 9999.0;
  for (int i = 0; i < 3; i++) {
    float distance = filteredLidarReading(i);
    Serial.print("LiDAR ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(distance);

    if (distance < closestDistance) {
      closestDistance = distance;
      closestSensor = i;
    }
  }

  if (closestSensor < 0 || closestDistance >= 9999.0) {
    Serial.println("No valid wall found - centering skipped");
    return false;
  }

  Serial.print("Centering with LiDAR ");
  Serial.println(closestSensor);

  int direction = 1;
  bool directionWasSwitched = false;
  bool distanceHasDecreased = false;
  int initialRises = 0;
  int risesAfterMinimum = 0;
  float previous = closestDistance;
  float minimumDistance = closestDistance;
  unsigned long startTime = millis();

  setCenteringTurn(direction, TURN_PWM);

  while (millis() - startTime < MAX_TURN_TIME_MS) {
    float current = filteredLidarReading(closestSensor);
    updateOdometry();

    if (current >= 9999.0) {
      delay(10);
      continue;
    }

    Serial.print("Center distance = ");
    Serial.println(current);

    if (current < minimumDistance - NOISE_MM) {
      minimumDistance = current;
      distanceHasDecreased = true;
      risesAfterMinimum = 0;
    }

    if (!distanceHasDecreased) {
      // If the first chosen direction immediately moves away from the wall,
      // reverse once and continue searching for the minimum.
      if (current > previous + NOISE_MM) initialRises++;
      else initialRises = 0;

      if (initialRises >= 2 && !directionWasSwitched) {
        direction = -direction;
        directionWasSwitched = true;
        initialRises = 0;
        setCenteringTurn(direction, TURN_PWM);
        delay(80);  // allow the drivetrain to change direction
      }
    } else {
      // A few consecutive readings above the best value mean we have passed
      // the perpendicular point. Requiring three rejects single noisy samples.
      if (current > minimumDistance + NOISE_MM) risesAfterMinimum++;
      else risesAfterMinimum = 0;

      if (risesAfterMinimum >= REQUIRED_RISES) {
        motor1.setPWM(0);
        motor2.setPWM(0);
        updateOdometry();
        Serial.println("Wall centred");
        return true;
      }
    }

    previous = current;
    delay(10);
  }

  motor1.setPWM(0);
  motor2.setPWM(0);
  Serial.println("Centering timed out");
  return false;
}

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
  const float LOOKAHEAD = 50.0;
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
    const float TURN_THRESHOLD_DEG = 100.0;
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
    float path_curvature = 2.0 * local_y / (l * l);

    // Side-wall centering is added only while travelling almost straight.
    // During corners and turnToHeading(), this function returns zero.
    float wall_curvature = wallCenteringCurvature(path_curvature, heading_gap);
    float curvature = path_curvature + wall_curvature;

    // clamp so the inner wheel target never flips sign (min turn radius = DISTANCE_BETWEEN_WHEELS/2)
    const float MAX_CURVATURE = 2.0 / DISTANCE_BETWEEN_WHEELS;
    curvature = constrain(curvature, -MAX_CURVATURE, MAX_CURVATURE);

    // slow down for corners
    const float CORNER_SLOWDOWN = 0.1;  // fraction of speed shed at max curvature (0=no slowdown, 1=full stop)
    float curvature_frac = fabs(curvature) / MAX_CURVATURE;
    float target_speed = TARGET_WHEEL_SPEED * (1.0 - CORNER_SLOWDOWN * curvature_frac);

    float left_wheel_speed  = target_speed * (1.0 - curvature * DISTANCE_BETWEEN_WHEELS / 2.0);
    float right_wheel_speed = target_speed * (1.0 + curvature * DISTANCE_BETWEEN_WHEELS / 2.0);

    if (fabs(path_curvature) < 0.0001 && fabs(wall_curvature) < 0.0001) {
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