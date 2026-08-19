#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <Vector.h>  //gives static c arrays similar functionality to std::vector. Same for arrays. 
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "waypoint.hpp"

//Motors 
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
mtrn3100::EncoderOdometry encoder_odometry(16,100);

//Encoders
#define MOT1ENCA 2
#define MOT1ENCB 7
#define MOT2ENCA 3
#define MOT2ENCB 8
mtrn3100::Encoder encoder1(MOT1ENCA, MOT1ENCB, true);
mtrn3100::Encoder encoder2(MOT2ENCA, MOT2ENCB);

// Controllers
mtrn3100::PIDController controller(200, 2, 10);

// The centering controller receives distance derivative as its input.
// Target = 0 mm/sample, so only proportional gain is needed initially.
mtrn3100::PIDController centerController(25, 0, 0);

// IMU
MPU6050 mpu(Wire);

//Lidar 
#define LEFT_LIDAR_ENABLE   A0    
#define FRONT_LIDAR_ENABLE  A2    
#define RIGHT_LIDAR_ENABLE  A1    

// Unique I2C addresses
#define LEFT_LIDAR_ADDRESS   0x30
#define FRONT_LIDAR_ADDRESS  0x31
#define RIGHT_LIDAR_ADDRESS  0x32

//Display
#define DISPLAY_ADDRESS 0x3C
SSD1306AsciiWire oled;

VL6180X leftLidar;
VL6180X frontLidar;
VL6180X rightLidar;

enum LidarID {
  LIDAR_LEFT = 0,
  LIDAR_FRONT = 1,
  LIDAR_RIGHT = 2,
  LIDAR_NONE = -1
};

// The LiDAR chosen for centering is locked here for the whole attempt.
LidarID activeLidar = LIDAR_NONE;

//Constants
#define TARGET_RAD 6.28
#define DISTANCE_BETWEEN_WHEELS 100.0
#define WHEEL_RADIUS 16

//Floodfill stuff
const int8_t MAX_PATH_SIZE = 30; //if path gets bigger than this you are cooked. 
const int8_t MAZE_SIZE = 9; // assumes maze is square
#include "maze_data.h"
Waypoint storage[MAX_PATH_SIZE];
Vector<Waypoint> path(storage);
int currIndex = 0; //index of current cell in path
const int WALL_LENGTH = 180; // same as WALL_LENGTH. unit is mm.  
const int WALL_THRESHOLD = WALL_LENGTH + 90; //unit is mm. if there is an object within this distance then it is a wall. 

const float QUART_PI = PI / 4.0f;
const float SOUTH_RAD = 0.0;
const float EAST_RAD = HALF_PI;
const float NORTH_RAD = PI;
const float WEST_RAD = PI + HALF_PI;

//Floodfill user-variables
const int8_t FF_GOAL_TOLERANCE = 20; //unit is mm. 
const Waypoint startPoint = { 0 * WALL_LENGTH, 3 * WALL_LENGTH};
const Waypoint goalPoint = {0 * WALL_LENGTH, 4 * WALL_LENGTH};

// Function declarations
bool startLidar(VL6180X &lidar, uint8_t enablePin, uint8_t newAddress);
void initialiseLidars();
uint16_t readOneLidar(VL6180X &lidar);
uint16_t readRawLidar(LidarID lidar);
uint16_t readFilteredLidar(LidarID lidar);
void updateOdometry();
// void serialPrint();
void driveForward(int distance);
void turnToHeading(float target_deg);
void navigateTo(int tx, int ty, float th_deg);
void purePursuitPath(Waypoint* waypoints, int num_waypoints, float final_heading_deg);
const char* lidarName(LidarID lidar);
void stopRobot();
void setSpinPWM(int pwm);
float getHeadingDegrees();
int findCenterDirection(LidarID selectedLidar);
LidarID findClosestLidar();
bool centerOnClosestSurface();
bool isWallPresent(int r, int c, int nr, int nc);
void floodfillAppend(Vector<Waypoint>& path, int currIndex, Waypoint goalCell);
// void printMazeToTerminal();
void printPath(const Vector<Waypoint>& pathList);
bool isAtTarget(Waypoint curr_pos, Waypoint target_pos, int range);
void searchRun(const Waypoint& begin, const Waypoint endPoint);
void reversePath(Vector<Waypoint>& path);
void updateCellFromHeading(Waypoint robot_pos, float heading_rad);
void updateMazeUsingLidar(Waypoint robot_pos, float heading_rad);
void printToOLED(const Waypoint& robot_pos);

void setup() {
    delay(1000);
//   Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000L);
    
    oled.begin(&Adafruit128x64, DISPLAY_ADDRESS);

    oled.setFont(newbasic3x5);
    oled.clear();

  // Initialise all three LiDAR sensors
  initialiseLidars();

  // Initialise IMU
  mpu.begin();

  Serial.println(F("Calibrating IMU, keep still..."));
  delay(1000);

  mpu.calcOffsets();

  Serial.println(F("IMU calibration done"));
  // delay(2000);
    // float heading_rad = 180.0;
    // printToOLED(startPoint);
    // delay(2000);
    // updateCellFromHeading(startPoint, heading_rad);
    // printToOLED(startPoint);
    // delay(2000);
    // updateCellFromHeading(startPoint, heading_rad + HALF_PI);
    // printToOLED(startPoint);
    // delay(2000);
    // updateCellFromHeading(startPoint, heading_rad - HALF_PI);
    // printToOLED(startPoint);

    floodfillAppend(path, 0, goalPoint);
    Waypoint finalPath[path.size()];
    for (int i{0}; i < path.size(); i++) {
        finalPath[i] = path[i];
    }

    purePursuitPath(finalPath, path.size(),0.0);

    // //search run to Goal
    // searchRun(startPoint, goalPoint);

    // //search run back to start 
    // searchRun(goalPoint, startPoint);

    // reversePath(path);

    // float final_heading = 180.0;  //I arbitrarily picked NORTH_RAD. 
    // purePursuitPath(path, sizeof(final_path), final_heading);
}

void loop() {

}


// =========================================================
// START ONE LIDAR
// =========================================================

bool startLidar(
  VL6180X &lidar,
  uint8_t enablePin,
  uint8_t newAddress
) {
  /*
   * Release XSHUT/GPIO.
   * Do not actively drive it HIGH.
   */
  pinMode(enablePin, INPUT);
  delay(50);

  lidar.init();
  lidar.configureDefault();
  lidar.setAddress(newAddress);
  lidar.setTimeout(500);

  delay(20);
  return true;
}
// =========================================================
// INITIALISE ALL THREE LIDARS
// =========================================================

void initialiseLidars() {
  /*
   * Disable all three sensors.
   *
   * They must not all be active at address 0x29
   * at the same time.
   */

  pinMode(
    LEFT_LIDAR_ENABLE,
    OUTPUT
  );

  pinMode(
    FRONT_LIDAR_ENABLE,
    OUTPUT
  );

  pinMode(
    RIGHT_LIDAR_ENABLE,
    OUTPUT
  );

  digitalWrite(
    LEFT_LIDAR_ENABLE,
    LOW
  );

  digitalWrite(
    FRONT_LIDAR_ENABLE,
    LOW
  );

  digitalWrite(
    RIGHT_LIDAR_ENABLE,
    LOW
  );

  delay(100);


  // -------------------------------------------------------
  // Left LiDAR
  // -------------------------------------------------------

  if (!startLidar(
        leftLidar,
        LEFT_LIDAR_ENABLE,
        LEFT_LIDAR_ADDRESS
      )) {


    while (true) {
      stopRobot();
      delay(100);
    }
  }

  Serial.println(F("Left LiDAR address = 0x30"));


  // -------------------------------------------------------
  // Front LiDAR
  // -------------------------------------------------------

  if (!startLidar(
        frontLidar,
        FRONT_LIDAR_ENABLE,
        FRONT_LIDAR_ADDRESS
      )) {

    Serial.println(
      "ERROR: Front LiDAR failed"
    );

    while (true) {
      stopRobot();
      delay(100);
    }
  }

  Serial.println(
    "Front LiDAR address = 0x31"
  );


  // -------------------------------------------------------
  // Right LiDAR
  // -------------------------------------------------------

  if (!startLidar(
        rightLidar,
        RIGHT_LIDAR_ENABLE,
        RIGHT_LIDAR_ADDRESS
      )) {

    Serial.println(
      "ERROR: Right LiDAR failed"
    );

    while (true) {
      stopRobot();
      delay(100);
    }
  }

  Serial.println(
    "Right LiDAR address = 0x32"
  );

  Serial.println(
    "All LiDAR sensors initialised"
  );
}


// =========================================================
// READ ONE LIDAR
// =========================================================

uint16_t readOneLidar(VL6180X &lidar) {
  uint16_t distance =
    lidar.readRangeSingleMillimeters();

  if (lidar.timeoutOccurred()) {
    return NAN;
  }

  return distance;
}


// =========================================================
// READ SELECTED LIDAR
// =========================================================

uint16_t readRawLidar(LidarID lidar) {
  switch (lidar) {
    case LIDAR_LEFT:
      return readOneLidar(leftLidar);

    case LIDAR_FRONT:
      return readOneLidar(frontLidar);

    case LIDAR_RIGHT:
      return readOneLidar(rightLidar);

    default:
      return NAN;
  }
}


uint16_t readFilteredLidar(LidarID lidar) {
  const int SAMPLE_COUNT = 3;

  uint16_t samples[SAMPLE_COUNT];

  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    uint16_t distance =
      readRawLidar(lidar);

    /*
     * 255 normally indicates an invalid or
     * out-of-range VL6180X measurement.
     */

    if (
      !isnan(distance) &&
      distance >= 5 &&
      distance < 255
    ) {
      samples[validCount] = distance;
      validCount++;
    }

    delay(2);
  }

  if (validCount == 0) {
    return NAN;
  }

  // Sort samples
  for (int i = 1; i < validCount; i++) {
    uint16_t value = samples[i];

    int j = i - 1;

    while (
      j >= 0 &&
      samples[j] > value
    ) {
      samples[j + 1] =
        samples[j];

      j--;
    }

    samples[j + 1] = value;
  }

  // Return median
  return samples[validCount / 2];
}



void updateOdometry() {
  encoder_odometry.update(encoder1.getRotation(), encoder2.getRotation());
}

// void serialPrint() {
//   Serial.print("ODOM:\t\t X = ");
//   Serial.print(encoder_odometry.getX());
//   Serial.print(",\t\t Y = ");
//   Serial.print(encoder_odometry.getY());
//   Serial.print(",\t\t H = ");
//   Serial.print(encoder_odometry.getH());
//   Serial.print(",\t\t DeltaLeft = ");
//   Serial.print(encoder_odometry.getDeltaLeft());
//   Serial.print(",\t\t DeltaRight = ");
//   Serial.print(encoder_odometry.getDeltaRight());
//   Serial.println();
// }

void driveForward(int distance) {
  float startLeft = encoder1.getRotation();
  float startRight = encoder2.getRotation();
  controller.integral = 0;
  controller.prev_error = 0;

  float target = float(distance) / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float leftRot = encoder1.getRotation() - startLeft;
    float rightRot = encoder2.getRotation() - startRight;
    float avgRot = (leftRot + rightRot) / 2.0;

    updateOdometry();  // global pose
    // serialPrint();

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

//   Serial.print("turnToHeading: target="); Serial.print(target_deg);
//   Serial.print(" current="); Serial.print(current_deg);
//   Serial.print(" delta="); Serial.println(delta);

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

    // Serial.print("avg="); Serial.print(avg);
    // Serial.print(" error="); Serial.print(controller.error);
    // Serial.print(" pwm="); Serial.println(pwm);

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

void navigateTo(int tx, int ty, float th_deg) {
  // rotate to face waypoint
  int dx = tx - encoder_odometry.getX();
  int dy = ty - encoder_odometry.getY();
  float angle_to_target_deg = atan2(dy, dx) * 180.0 / PI;
  while (th_deg > 180.0) th_deg -= 360.0;
  while (th_deg < -180.0) th_deg += 360.0;

  turnToHeading(angle_to_target_deg);

  // drive straight
  int distance = sqrt(dx * dx + dy * dy);
  driveForward(distance);

  // rotate to final heading
  turnToHeading(th_deg);
}

void purePursuitPath(Waypoint* waypoints, int num_waypoints, float final_heading_deg) {
  const float LOOKAHEAD = 75.0;
  const float MAX_PWM = 130.0;
  const float BASE_SPEED = 60.0;
  const float GOAL_TOLERANCE = 10.0;

  // Wall correction settings.
  // A side reading above 160 mm is treated as no wall.
  const float WALL_MAX_DISTANCE = 160.0;
  const float WALL_KP = 0.35;
  const float MAX_WALL_CORRECTION = 30.0;

  int current_segment = 0;  // which segment we're tracking

  while (true) {
    updateOdometry();

    float rx = encoder_odometry.getX();
    float ry = encoder_odometry.getY();
    float rh = encoder_odometry.getH();

    // final goal is the last waypoint
    float final_x = waypoints[num_waypoints - 1].x;
    float final_y = waypoints[num_waypoints - 1].y;
    float dist_to_goal = sqrt((final_x-rx)*(final_x-rx) + (final_y-ry)*(final_y-ry));
    if (dist_to_goal < GOAL_TOLERANCE) {
      motor1.setPWM(0);
      motor2.setPWM(0);
      break;
    }

    // advance segment if robot has passed current waypoint
    if (current_segment < num_waypoints - 2) {
      float wx = waypoints[current_segment + 1].x;
      float wy = waypoints[current_segment + 1].y;
      float dist_to_next = sqrt((wx-rx)*(wx-rx) + (wy-ry)*(wy-ry));
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
    float seg_len = sqrt(seg_dx*seg_dx + seg_dy*seg_dy);

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
    float local_x =  cos_h * (lx - rx) + sin_h * (ly - ry);
    float local_y = -sin_h * (lx - rx) + cos_h * (ly - ry);

    // curvature and wheel speeds
    float l = sqrt(local_x*local_x + local_y*local_y);
    float curvature = 2.0 * local_y / (l * l);

    float left_pwm  = BASE_SPEED * (1.0 - curvature * DISTANCE_BETWEEN_WHEELS / 2.0);
    float right_pwm = BASE_SPEED * (1.0 + curvature * DISTANCE_BETWEEN_WHEELS / 2.0);

    // Read both side walls. Correction is applied only when BOTH
    // distances are valid and no greater than 160 mm.
    uint16_t leftDistance = readFilteredLidar(LIDAR_LEFT);
    uint16_t rightDistance = readFilteredLidar(LIDAR_RIGHT);

    bool leftValid =
      !isnan(leftDistance) &&
      leftDistance >= 5.0 &&
      leftDistance <= WALL_MAX_DISTANCE;

    bool rightValid =
      !isnan(rightDistance) &&
      rightDistance >= 5.0 &&
      rightDistance <= WALL_MAX_DISTANCE;

    if (leftValid && rightValid) {
      // Positive error means the left wall is closer, so steer right.
      float wallError = rightDistance - leftDistance;
      float wallCorrection = WALL_KP * wallError;

      wallCorrection = constrain(
        wallCorrection,
        -MAX_WALL_CORRECTION,
        MAX_WALL_CORRECTION
      );

      left_pwm += wallCorrection;
      right_pwm -= wallCorrection;

    //   Serial.print("WALL: L=");
    //   Serial.print(leftDistance);
    //   Serial.print(" R=");
    //   Serial.print(rightDistance);
    //   Serial.print(" correction=");
    //   Serial.println(wallCorrection);
    }

    left_pwm  = constrain(left_pwm,  -MAX_PWM, MAX_PWM);
    right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

    motor1.setPWM(-left_pwm);
    motor2.setPWM(right_pwm);

    // serialPrint();
    delay(10);
  }

  // final heading correction
  turnToHeading(final_heading_deg);
}

const char* lidarName(LidarID lidar) {
  switch (lidar) {
    case LIDAR_LEFT:
      return "LEFT";

    case LIDAR_FRONT:
      return "FRONT";

    case LIDAR_RIGHT:
      return "RIGHT";

    default:
      return "NONE";
  }
}

//lidar functions 
//read 
// Stop both drive motors.
void stopRobot() {
  motor1.setPWM(0);
  motor2.setPWM(0);
}

// Positive PWM uses the same turning convention as turnToHeading():
// positive = increasing odometry heading, negative = decreasing heading.
void setSpinPWM(int pwm) {
  pwm = constrain(pwm, -255, 255);
  motor1.setPWM(pwm);
  motor2.setPWM(pwm);
}

float getHeadingDegrees() {
  updateOdometry();

  float heading = encoder_odometry.getH() * 180.0 / PI;

  while (heading > 180.0) heading -= 360.0;
  while (heading < -180.0) heading += 360.0;

  return heading;
}

// Probe a small angle on both sides of the current heading and choose the
// direction that gives the smaller distance. Returning 0 means the current
// heading is already approximately at the local minimum.
int findCenterDirection(LidarID selectedLidar) {
  const float PROBE_ANGLE_DEG = 7.0;
  const float ALREADY_CENTRED_TOLERANCE_MM = 0.75;
  const unsigned long SETTLE_TIME_MS = 120;

  float originalHeading = getHeadingDegrees();
  uint16_t centreDistance = readFilteredLidar(selectedLidar);

  turnToHeading(originalHeading + PROBE_ANGLE_DEG);
  delay(SETTLE_TIME_MS);
  uint16_t positiveDistance = readFilteredLidar(selectedLidar);

  turnToHeading(originalHeading - PROBE_ANGLE_DEG);
  delay(SETTLE_TIME_MS);
  uint16_t negativeDistance = readFilteredLidar(selectedLidar);

  // Return to where the probe started before doing the real centering turn.
  turnToHeading(originalHeading);
  delay(SETTLE_TIME_MS);

//   Serial.print("Probe distances: negative=");
//   Serial.print(negativeDistance);
//   Serial.print(" mm, centre=");
//   Serial.print(centreDistance);
//   Serial.print(" mm, positive=");
//   Serial.print(positiveDistance);
//   Serial.println(" mm");

  if (isnan(centreDistance)) {
    return 0;
  }

  if (isnan(positiveDistance) && isnan(negativeDistance)) {
    return 0;
  }

  if (isnan(positiveDistance)) {
    return -1;
  }

  if (isnan(negativeDistance)) {
    return 1;
  }

  if (
    centreDistance <= positiveDistance + ALREADY_CENTRED_TOLERANCE_MM &&
    centreDistance <= negativeDistance + ALREADY_CENTRED_TOLERANCE_MM
  ) {
    return 0;
  }

  return (positiveDistance < negativeDistance) ? 1 : -1;
}


LidarID findClosestLidar() {
  uint16_t distances[3];

  distances[LIDAR_LEFT]  = readFilteredLidar(LIDAR_LEFT);
  distances[LIDAR_FRONT] = readFilteredLidar(LIDAR_FRONT);
  distances[LIDAR_RIGHT] = readFilteredLidar(LIDAR_RIGHT);

  LidarID closest = LIDAR_NONE;
  uint16_t smallestDistance = 9999;

  for (int i = 0; i < 3; i++) {
    // Serial.print(lidarName((LidarID)i));
    // Serial.print(": ");
    // Serial.println(distances[i]);

    if (!isnan(distances[i]) &&
        distances[i] >= 5.0 &&
        distances[i] < smallestDistance) {

      smallestDistance = distances[i];
      closest = (LidarID)i;
    }
  }

  return closest;
}

bool centerOnClosestSurface() {
  const int TURN_PWM = 40;

  const unsigned long FORCED_TURN_MS = 180;
  const unsigned long SEARCH_PULSE_MS = 35;
  const unsigned long SETTLE_TIME_MS = 35;
  const unsigned long SEARCH_TIMEOUT_MS = 6000;

  const float DIRECTION_THRESHOLD_MM = 1.5;
  const float NEW_MINIMUM_THRESHOLD_MM = 0.5;
  const float RISE_THRESHOLD_MM = 1.5;

  const int REQUIRED_RISES = 2;

  // =========================================================
  // 1. CHOOSE CLOSEST LIDAR ONCE
  // =========================================================
  activeLidar = findClosestLidar();

//   Serial.print("CENTER: selected ");
//   Serial.println(lidarName(activeLidar));

  uint16_t startDistance = readFilteredLidar(activeLidar);

  uint16_t startHeading = getHeadingDegrees();

//   Serial.print("CENTER: start distance = ");
//   Serial.print(startDistance);
//   Serial.print(" mm | heading = ");
//   Serial.println(startHeading);

  // =========================================================
  // 2. FORCE ROBOT TO TURN FIRST
  // =========================================================
  int direction = 1;

//   Serial.println("CENTER: forcing first turn");

  setSpinPWM(direction * TURN_PWM);
  delay(FORCED_TURN_MS);

  stopRobot();
  delay(SETTLE_TIME_MS);

  uint16_t probeDistance = readFilteredLidar(activeLidar);

  float probeHeading = getHeadingDegrees();

//   Serial.print("CENTER: probe distance = ");
//   Serial.print(probeDistance);
//   Serial.print(" mm | heading = ");
//   Serial.println(probeHeading);

  // If distance got bigger, reverse direction
  if (probeDistance > startDistance + DIRECTION_THRESHOLD_MM) {
    direction = -1;

    // Serial.println(
    //   "CENTER: distance increased -> reversing search"
    // );
  }
  else {
    // Serial.println(
    //   "CENTER: distance decreased -> continuing search"
    // );
  }

  // =========================================================
  // 3. SAVE BEST DISTANCE + HEADING
  // =========================================================
  float bestDistance;
  float bestHeading;

  if (probeDistance < startDistance) {
    bestDistance = probeDistance;
    bestHeading = probeHeading;
  }
  else {
    bestDistance = startDistance;
    bestHeading = startHeading;
  }

  float previousDistance = probeDistance;

  bool hasMovedTowardMinimum =
    probeDistance <
    startDistance - NEW_MINIMUM_THRESHOLD_MM;

  int risingCount = 0;

  unsigned long searchStart = millis();

  // =========================================================
  // 4. SEARCH FOR MINIMUM
  // =========================================================
  while (
    millis() - searchStart <
    SEARCH_TIMEOUT_MS
  ) {

    // Turn slightly
    setSpinPWM(direction * TURN_PWM);
    delay(SEARCH_PULSE_MS);

    stopRobot();
    delay(SETTLE_TIME_MS);

    uint16_t currentDistance =
      readFilteredLidar(activeLidar);

    if (isnan(currentDistance)) {
    //   Serial.println(
    //     "CENTER: invalid reading during search"
    //   );

      stopRobot();
      return false;
    }

    float currentHeading =
      getHeadingDegrees();

    // =====================================================
    // FOUND A NEW SMALLER DISTANCE
    // =====================================================
    if (
      currentDistance <
      bestDistance - NEW_MINIMUM_THRESHOLD_MM
    ) {

      bestDistance = currentDistance;
      bestHeading = currentHeading;

      hasMovedTowardMinimum = true;
      risingCount = 0;

    //   Serial.print("CENTER: new minimum = ");
    //   Serial.print(bestDistance);

    //   Serial.print(" mm @ ");
    //   Serial.print(bestHeading);

    //   Serial.println(" deg");
    }

    // =====================================================
    // CHECK IF WE PASSED THE MINIMUM
    // =====================================================
    if (
      hasMovedTowardMinimum &&
      currentDistance >
        bestDistance + RISE_THRESHOLD_MM &&
      currentDistance >
        previousDistance + 0.5
    ) {

      risingCount++;
    }
    else if (
      currentDistance <=
      previousDistance + 0.5
    ) {

      risingCount = 0;
    }

    // Serial.print("CENTER: prev=");
    // Serial.print(previousDistance);

    // Serial.print(" current=");
    // Serial.print(currentDistance);

    // Serial.print(" best=");
    // Serial.print(bestDistance);

    // Serial.print(" heading=");
    // Serial.print(currentHeading);

    // Serial.print(" rises=");
    // Serial.println(risingCount);

    // =====================================================
    // DISTANCE HAS STARTED INCREASING
    // =====================================================
    if (risingCount >= REQUIRED_RISES) {

      stopRobot();

    //   Serial.println(
    //     "CENTER: minimum passed"
    //   );

    //   Serial.print(
    //     "CENTER: returning to best heading "
    //   );

    //   Serial.println(bestHeading);

      // Return using odometry instead of guessing
      // how long to reverse the motors.
      turnToHeading(bestHeading);

      stopRobot();
      delay(100);

      uint16_t finalDistance = readFilteredLidar(activeLidar);

    //   Serial.print(
    //     "CENTER: final distance = "
    //   );

    //   Serial.print(finalDistance);
    //   Serial.println(" mm");

    //   Serial.println("CENTERED");

      return true;
    }

    previousDistance =
      currentDistance;
  }

  stopRobot();

//   Serial.println(
//     "CENTER: search timeout"
//   );

  return false;
}

inline int coordToGrid(float val) {
    return static_cast<int>(val / WALL_LENGTH + 0.5f);
}

bool isWallPresent(int r, int c, int nr, int nc) {
    if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) {
        return true; // Out of bounds is a wall
    }

    // Moving along X axis (Vertically Down/Up: Row boundaries)
    if (nr == r + 1) { // Moving Down (+X)
        return (MAZE_HORIZONTAL_WALLS[c] & (1 << (r + 1))) != 0;
    }
    if (nr == r - 1) { // Moving Up (-X)
        return (MAZE_HORIZONTAL_WALLS[c] & (1 << r)) != 0;
    }

    // Moving along Y axis (Horizontally Right/Left: Column boundaries)
    if (nc == c + 1) { // Moving Right (+Y)
        return (MAZE_VERTICAL_WALLS[r] & (1 << (c + 1))) != 0;
    }
    if (nc == c - 1) { // Moving Left (-Y)
        return (MAZE_VERTICAL_WALLS[r] & (1 << c)) != 0;
    }
    return true;
}

void floodfillAppend(Vector<Waypoint>& path, int currIndex, Waypoint goalCell) {
    // X maps to Row (down), Y maps to Col (right)
    int startR = coordToGrid(path[currIndex].x);
    int startC = coordToGrid(path[currIndex].y);
    int goalR  = coordToGrid(goalCell.x);
    int goalC  = coordToGrid(goalCell.y);

    if (startR == goalR && startC == goalC) return;

    uint8_t dist[MAZE_SIZE][MAZE_SIZE];
    for (int r = 0; r < MAZE_SIZE; ++r) {
        for (int c = 0; c < MAZE_SIZE; ++c) {
            dist[r][c] = 255;
        }
    }

    struct Cell { uint8_t r, c; };
    Cell queue[MAZE_SIZE * MAZE_SIZE];
    int head = 0;
    int tail = 0;

    dist[goalR][goalC] = 0;
    queue[tail++] = { (uint8_t)goalR, (uint8_t)goalC };

    const int dr[] = {-1, 1, 0, 0};
    const int dc[] = { 0, 0,-1, 1};

    while (head < tail) {
        Cell curr = queue[head++];
        uint8_t currentDist = dist[curr.r][curr.c];

        if (curr.r == startR && curr.c == startC) {
            break; 
        }

        for (int i = 0; i < 4; ++i) {
            int nr = curr.r + dr[i];
            int nc = curr.c + dc[i];

            if (nr >= 0 && nr < MAZE_SIZE && nc >= 0 && nc < MAZE_SIZE) {
                if (!isWallPresent(curr.r, curr.c, nr, nc) && dist[nr][nc] == 255) {
                    dist[nr][nc] = currentDist + 1;
                    queue[tail++] = { (uint8_t)nr, (uint8_t)nc };
                }
            }
        }
    }

    int currR = startR;
    int currC = startC;

    while (currR != goalR || currC != goalC) {
        int bestR = currR;
        int bestC = currC;
        uint8_t minDist = dist[currR][currC];

        for (int i = 0; i < 4; ++i) {
            int nr = currR + dr[i];
            int nc = currC + dc[i];

            if (nr >= 0 && nr < MAZE_SIZE && nc >= 0 && nc < MAZE_SIZE) {
                if (!isWallPresent(currR, currC, nr, nc) && dist[nr][nc] < minDist) {
                    minDist = dist[nr][nc];
                    bestR = nr;
                    bestC = nc;
                }
            }
        }

        if (bestR == currR && bestC == currC) {
            // Serial.println(F("ERROR: No reachable path found to goal!"));
            break; 
        }

        Waypoint nextWp;
        nextWp.x = bestR * WALL_LENGTH; // X = Row index * Cell Size
        nextWp.y = bestC * WALL_LENGTH; // Y = Column index * Cell Size
        path.push_back(nextWp);

        currR = bestR;
        currC = bestC;
    }
}

// void printMazeToTerminal()
// {
//     for (int wallRow = 0; wallRow <= MAZE_SIZE; wallRow++)
//     {
//         // Horizontal walls (separated across columns horizontally)
//         for (int col = 0; col < MAZE_SIZE; col++)
//         {
//             bool wall = (MAZE_HORIZONTAL_WALLS[col] >> wallRow) & 1;

//             Serial.print("+");
//             Serial.print(wall ? "---" : "   ");
//         }

//         Serial.println("+");

//         if (wallRow == MAZE_SIZE)
//             break;

//         // Vertical walls (separated across rows vertically)
//         int row = wallRow;

//         for (int boundary = 0; boundary <= MAZE_SIZE; boundary++)
//         {
//             bool wall = (MAZE_VERTICAL_WALLS[row] >> boundary) & 1;

//             Serial.print(wall ? "|" : " ");

//             if (boundary < MAZE_SIZE)
//                 Serial.print("   ");
//         }

//         Serial.println();
//     }
// }

void printPath(const Vector<Waypoint>& pathList) {
    Serial.print(F("Total Elements: "));
    Serial.println(pathList.size());
    Serial.println(F("Generated Waypoint Path:"));

    for (size_t i = 0; i < pathList.size(); ++i) {
        Serial.print("(");
        Serial.print(pathList[i].x);
        Serial.print(",");
        Serial.print(pathList[i].y);
        Serial.print(")");
    }
    Serial.println();
}

bool isAtTarget(Waypoint curr_pos, Waypoint target_pos, int range) {
    int dx = target_pos.x - curr_pos.x;
    int dy = target_pos.y - curr_pos.y;
    
    int distance = sqrt(dx * dx + dy * dy);
    
    return distance <= range;
}

void searchRun(const Waypoint& begin, const Waypoint endPoint) {
// search run back to start 
  // Serial.println(F("test1"));
  currIndex = 0;
  path.empty();
  path.push_back(begin);
  floodfillAppend(path, currIndex, endPoint);
  // Serial.println(F("test2"));
  while (isAtTarget(path[currIndex], endPoint, FF_GOAL_TOLERANCE) == false) {
    // Serial.println(F("test3"));
    // centerOnClosestSurface();
    //updateOdomUsingLidar
    updateMazeUsingLidar(path[currIndex], encoder_odometry.getH());
    printToOLED(path[currIndex]);
    int8_t r = static_cast<int>(path[currIndex].x / WALL_LENGTH);
    int8_t c = static_cast<int>(path[currIndex].y / WALL_LENGTH);
    int8_t nr = static_cast<int>(path[currIndex + 1].x / WALL_LENGTH);
    int8_t nc = static_cast<int>(path[currIndex + 1].y / WALL_LENGTH);

    //if wall then recalculate else keep moving.
    if (isWallPresent(r,c,nr,nc)) {  //will the lidar detect step at the begining be enough ?????? this check assumes walls have already been detected.
        int lastIndex = path.size() - 1; //index of last elem in path. 
        for (int i{lastIndex}; i > currIndex; i--) {  //removes all cells after current cell. 
            path.pop_back();     
        }
        floodfillAppend(path, currIndex, begin); //appends next steps to get to goal
    }

    // Discrete direction calculation without atan2() or float math
    int dx = nr - r;
    int dy = nc - c;
    float heading_rad = 0.0f;

    if (dx > 0)       heading_rad = 0.0f;          // East
    else if (dx < 0)  heading_rad = PI;            // West
    else if (dy > 0)  heading_rad = HALF_PI;       // North
    else if (dy < 0)  heading_rad = -HALF_PI;      // South
    
    navigateTo(path[currIndex + 1].x, path[currIndex + 1].y, heading_rad);  
    currIndex++;
  }
}

// //takes path and reverses it. Or should I make a function that just drives path in reverse for less mem?
void reversePath(Vector<Waypoint>& path){
  int tailIndex = path.size() - 1;
  int middleIndex = (path.size() + 1) / 2; //the + 1 is to round up the integer division to get the average rounded up.
  for (int i{0}; i < middleIndex; i++) {
    Waypoint curr = path[i];
    path[i] = path[tailIndex];
    path[tailIndex] = curr;
    tailIndex--;
  }
}

// void updateOdomUsingLidar() {

// }

//returns cell that robot is poting to.
void updateCellFromHeading(Waypoint robot_pos, float heading_rad) {
    while (heading_rad < 0) {
        heading_rad += 2*M_PI;
    }
    while (heading_rad >= 2*M_PI) {
        heading_rad -= 2*M_PI;
    }
    //calc curr row and column of robot_pos;
    int r = static_cast<int>(robot_pos.x / WALL_LENGTH);
    int c = static_cast<int>(robot_pos.y / WALL_LENGTH);

    if ( SOUTH_RAD - QUART_PI <= heading_rad && heading_rad < SOUTH_RAD + QUART_PI) {
        MAZE_HORIZONTAL_WALLS[c] |= (1 << (r + 1));
    } else if (EAST_RAD - QUART_PI <= heading_rad && heading_rad < EAST_RAD + QUART_PI) {
        MAZE_VERTICAL_WALLS[r] |= (1 << (c + 1));
    } else if (NORTH_RAD - QUART_PI <= heading_rad && heading_rad < NORTH_RAD + QUART_PI) {
        MAZE_HORIZONTAL_WALLS[c] |= (1 << r);
    } else if (WEST_RAD - QUART_PI <= heading_rad && heading_rad < WEST_RAD + QUART_PI) { 
        MAZE_VERTICAL_WALLS[r] |= (1 << c);
    } 
}

void updateMazeUsingLidar(Waypoint robot_pos, float heading_rad) {
    uint16_t frontDist = readFilteredLidar(LIDAR_FRONT);
    uint16_t leftDist = readFilteredLidar(LIDAR_LEFT);
    uint16_t rightDist = readFilteredLidar(LIDAR_RIGHT);
    if (frontDist < WALL_THRESHOLD) {
      updateCellFromHeading(robot_pos, heading_rad);
    }
    if (leftDist < WALL_THRESHOLD) {
      updateCellFromHeading(robot_pos, heading_rad + HALF_PI);
    }
    if (rightDist < WALL_THRESHOLD) {
      updateCellFromHeading(robot_pos, heading_rad - HALF_PI);
    }
}

// void printToOLED(const Waypoint& robot_pos) {
//     int robotRow = static_cast<int>(robot_pos.x / WALL_LENGTH);
//     int robotCol = static_cast<int>(robot_pos.y / WALL_LENGTH);

//     oled.clear();


//     // Iterate through all rows (0 to MAZE_SIZE)
//     for (int r = 0; r <= MAZE_SIZE; ++r) {

//         // 1. Render Top Horizontal Walls for row 'r'
//         display.setCaret(0, r * 2 * charHeight);
//         for (int c = 0; c < MAZE_SIZE; ++c) {
//             // Check if the bit at column 'c' is set in MAZE_HORIZONTAL_WALLS[c] for level 'r'
//             bool wallPresent = (MAZE_HORIZONTAL_WALLS[c] & (1 << r)) != 0;
            
//             display.print(' ');
//             display.print(wallPresent ? '_' : ' ');
//         }
//         display.print(' '); // End boundary corner

//         // Render cell contents and vertical walls (only up to MAZE_SIZE)
//         if (r < MAZE_SIZE) {
//             display.setCaret(0, (r * 2 + 1) * charHeight);

//             for (int c = 0; c <= MAZE_SIZE; ++c) {
//                 // Check vertical wall at bit position 'c' in row 'r'
//                 bool wallPresent = (MAZE_VERTICAL_WALLS[r] & (1 << c)) != 0;
                
//                 display.print(wallPresent ? '|' : ' ');

//                 // Render robot position or empty cell space
//                 if (c < MAZE_SIZE) {
//                     if (robotRow == r && robotCol == c) {
//                         display.print('.');
//                     } else {
//                         display.print(' ');
//                     }
//                 }
//             }
//         }
//     }

//     display.update();
// }


//Gen Ai used to bugfix
void printToOLED(const Waypoint& robotPos) {
    oled.clear();
    int initRow = (robotPos.x >= 6) ? 2 : 0; // Start at row 2 if robot is further down

    // Print top row
    if (initRow == 0) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            oled.print(" ");
            bool horiPresent = (MAZE_HORIZONTAL_WALLS[c] & (1 << 0));
            oled.print(horiPresent ? "_" : " ");
        }
        oled.print("%");
        oled.println();
    }

    // Limit visible rows so total output lines never exceed 8
    int maxRows = (initRow == 0) ? 7 : 8; 
    int endRow = min((int)MAZE_SIZE, initRow + maxRows);  // lowk is the min necessary Gemini? lol...
    bool statsPrinted = false;
    for (int r = initRow; r < endRow; r++) {
        for (int c = 0; c <= MAZE_SIZE; c++) {
            bool vertPresent = (MAZE_VERTICAL_WALLS[r] & (1 << c));
            bool horiPresent = (MAZE_HORIZONTAL_WALLS[c] & (1 << (r + 1)));
            oled.print(vertPresent ? "|" : " ");
            oled.print(horiPresent ? "_" : " ");
        }
        if (!statsPrinted) {
            oled.print("   %");
            statsPrinted = true;
        }
        oled.println();
    }
}

void printStats() {
    oled.clear();
    
    oled.print(F("x: "))
    oled.print();
    oled.print(F("y: "))
    oled.print();
    oled.print(F("r: "))
    oled.print();
    oled.print(F("c: "))
    oled.print();
    oled.println();
    
    oled.print(F("l: "))
    oled.print();
    oled.print(F("f: "))
    oled.print();
    oled.print(F("r: "))
    oled.print();
    oled.println();
}