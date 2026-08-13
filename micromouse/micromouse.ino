#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "maze.hpp"
#include "maze_planner.hpp"
#include "waypoint.hpp"
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SSD1306.h>

//Display
SSD1306 display;

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
#define WHEEL_DIAMETER 32.0
#define DISTANCE_BETWEEN_WHEELS 100.0
#define WHEEL_RADIUS (WHEEL_DIAMETER / 2.0)
const int WALL_LENGTH = 180;

// Autonomous mapping start / goal (hard-coded per competition rules).
const int START_I = 0;
const int START_J = 0;
const int GOAL_I = 4;
const int GOAL_J = 7;
const direction START_HEADING = SOUTH;

Maze maze;
MazePlanner mazePlanner(maze);
Waypoint shortest_path[MAX_SHORTEST_PATH];
int shortest_path_len = 0;

const float WALL_DETECT_MM = 120.0f;

// Function declarations
bool startLidar(VL6180X &lidar, uint8_t enablePin, uint8_t newAddress);
void initialiseLidars();
float readOneLidar(VL6180X &lidar);
float readRawLidar(LidarID lidar);
float readFilteredLidar(LidarID lidar);
void updateOdometry();
void serialPrint();
void driveForward(float distance);
void turnToHeading(float target_deg);
void navigateTo(float tx, float ty, float th_deg);
void purePursuitPath(Waypoint* waypoints, int num_waypoints, float final_heading_deg);
void stopRobot();
void setSpinPWM(int pwm);
float getHeadingDegrees();
int findCenterDirection(LidarID selectedLidar);
LidarID findClosestLidar();
bool centerOnClosestSurface();
const char* lidarName(LidarID lidar);
direction turnLeft(direction dir);
direction turnRight(direction dir);
void senseAndUpdateWalls(int ci, int cj, direction heading);
void driveOneCell(direction dir);
bool runAutonomousMapping();
void printShortestPath();

void setup() {
  delay(1000);

  Serial.begin(9600);
  Wire.begin();
  // Initialise all three LiDAR sensors
  initialiseLidars();

  // Initialise IMU
  mpu.begin();

  Serial.println("Calibrating IMU, keep still...");
  delay(1000);

  mpu.calcOffsets();

  Serial.println("IMU calibration done");
  delay(2000);

  //updatePosUsingLidar   << placeholder for trysthan's centering code. 

    Maze maze();
    maze.mazeLoad();
    printToOLED(maze, std::make_pair(0,0));
}

void loop() {
//   static bool missionDone = false;
//   if (!missionDone) {
//     missionDone = runAutonomousMapping();
//   }
}

direction turnLeft(direction dir) {
  switch (dir) {
    case NORTH: return WEST;
    case EAST: return NORTH;
    case SOUTH: return EAST;
    case WEST: return SOUTH;
  }
  return NORTH;
}

direction turnRight(direction dir) {
  switch (dir) {
    case NORTH: return EAST;
    case EAST: return SOUTH;
    case SOUTH: return WEST;
    case WEST: return NORTH;
  }
  return NORTH;
}

void senseAndUpdateWalls(int ci, int cj, direction heading) {
  const direction absDirs[3] = {
    heading,
    turnRight(heading),
    turnLeft(heading)
  };
  const LidarID lidars[3] = {
    LIDAR_FRONT,
    LIDAR_RIGHT,
    LIDAR_LEFT
  };

  for (int k = 0; k < 3; ++k) {
    if (readFilteredLidar(lidars[k]) < WALL_DETECT_MM) {
      maze.addWall({ci, cj}, absDirs[k]);
    }
  }
}

void driveOneCell(direction dir) {
  turnToHeading(MazePlanner::directionToHeading(dir));
  driveForward(static_cast<float>(WALL_LENGTH));
}

void printShortestPath() {
  Serial.print("Shortest path (");
  Serial.print(shortest_path_len);
  Serial.println(" waypoints):");
  for (int k = 0; k < shortest_path_len; ++k) {
    Serial.print("  ");
    Serial.print(k);
    Serial.print(": (");
    Serial.print(shortest_path[k].x);
    Serial.print(", ");
    Serial.print(shortest_path[k].y);
    Serial.println(")");
  }
}

bool runAutonomousMapping() {
  maze.mazeLoad();
  mazePlanner.resetExplore();

  int ci = START_I;
  int cj = START_J;
  direction heading = START_HEADING;

  mazePlanner.markVisited(ci, cj);
  senseAndUpdateWalls(ci, cj, heading);

  Serial.println("Starting DFS exploration...");
  while (true) {
    direction moveDir = NORTH;
    const ExploreStep step = mazePlanner.getNextExploreMove(ci, cj, moveDir);

    Serial.print("Map ");
    Serial.print(mazePlanner.visitPercent());
    Serial.println("% complete");

    if (step == ExploreStep::DONE) {
      Serial.println("DFS exploration complete.");
      break;
    }

    driveOneCell(moveDir);
    heading = moveDir;

    if (!MazePlanner::neighbour(ci, cj, moveDir, ci, cj)) {
      Serial.println("ERROR: move left maze bounds.");
      return true;
    }

    senseAndUpdateWalls(ci, cj, heading);

    if (ci == GOAL_I && cj == GOAL_J) {
      Serial.println("Goal cell reached during exploration.");
    }
  }

  Serial.println("Running BFS for shortest path...");
  if (!mazePlanner.computeShortestPath(
        {START_I, START_J},
        {GOAL_I, GOAL_J},
        shortest_path,
        shortest_path_len,
        MAX_SHORTEST_PATH,
        static_cast<float>(WALL_LENGTH))) {
    Serial.println("ERROR: BFS could not find a path to the goal.");
    shortest_path_len = 0;
    return true;
  }

  printShortestPath();
  maze.printMaze();
  return true;
}

// =========================================================
// INITIALISE ALL THREE LIDARS
// =========================================================

void initialiseLidars() {
















//   static bool pathCompleted = false;

//   if (!pathCompleted) {
//     pathCompleted = true;

//     // Replace these coordinates with your required waypoints (mm).
//     Waypoint path[] = {
//       {0.0,   0.0},
//       {180.0, 0.0},
//       {360.0, 0.0},
//       {360.0, -180.0}
//     };

//     int numberOfWaypoints =
//       sizeof(path) / sizeof(path[0]);

//     purePursuitPath(
//       path,
//       numberOfWaypoints,
//       90.0
//     );

//     stopRobot();
//     Serial.println("Waypoint path complete");
//   }
// }


// // =========================================================
// // START ONE LIDAR
// // =========================================================

// bool startLidar(
//   VL6180X &lidar,
//   uint8_t enablePin,
//   uint8_t newAddress
// ) {
//   /*
//    * Release XSHUT/GPIO.
//    * Do not actively drive it HIGH.
//    */
//   pinMode(enablePin, INPUT);
//   delay(50);

//   lidar.init();
//   lidar.configureDefault();
//   lidar.setAddress(newAddress);
//   lidar.setTimeout(500);

//   delay(20);
//   return true;
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

  Serial.println(
    "Left LiDAR address = 0x30"
  );


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

float readOneLidar(VL6180X &lidar) {
  uint16_t distance =
    lidar.readRangeSingleMillimeters();

  if (lidar.timeoutOccurred()) {
    return NAN;
  }

  return (float)distance;
}


// =========================================================
// READ SELECTED LIDAR
// =========================================================

float readRawLidar(LidarID lidar) {
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


float readFilteredLidar(LidarID lidar) {
  const int SAMPLE_COUNT = 3;

  float samples[SAMPLE_COUNT];

  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float distance =
      readRawLidar(lidar);

    /*
     * 255 normally indicates an invalid or
     * out-of-range VL6180X measurement.
     */

    if (
      !isnan(distance) &&
      distance >= 5.0 &&
      distance < 255.0
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
    float value = samples[i];

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

  Serial.print("turnToHeading: target="); Serial.print(target_deg);
  Serial.print(" current="); Serial.print(current_deg);
  Serial.print(" delta="); Serial.println(delta);

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

    Serial.print("avg="); Serial.print(avg);
    Serial.print(" error="); Serial.print(controller.error);
    Serial.print(" pwm="); Serial.println(pwm);

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
  const float LOOKAHEAD = 75.0;
  const float MAX_PWM = 130.0;
  const float BASE_SPEED = 60.0;
  const float GOAL_TOLERANCE = 5.0;

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
    float leftDistance = readFilteredLidar(LIDAR_LEFT);
    float rightDistance = readFilteredLidar(LIDAR_RIGHT);

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

      Serial.print("WALL: L=");
      Serial.print(leftDistance);
      Serial.print(" R=");
      Serial.print(rightDistance);
      Serial.print(" correction=");
      Serial.println(wallCorrection);
    }

    left_pwm  = constrain(left_pwm,  -MAX_PWM, MAX_PWM);
    right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

    motor1.setPWM(-left_pwm);
    motor2.setPWM(right_pwm);

    serialPrint();
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
  float centreDistance = readFilteredLidar(selectedLidar);

  turnToHeading(originalHeading + PROBE_ANGLE_DEG);
  delay(SETTLE_TIME_MS);
  float positiveDistance = readFilteredLidar(selectedLidar);

  turnToHeading(originalHeading - PROBE_ANGLE_DEG);
  delay(SETTLE_TIME_MS);
  float negativeDistance = readFilteredLidar(selectedLidar);

  // Return to where the probe started before doing the real centering turn.
  turnToHeading(originalHeading);
  delay(SETTLE_TIME_MS);

  Serial.print("Probe distances: negative=");
  Serial.print(negativeDistance);
  Serial.print(" mm, centre=");
  Serial.print(centreDistance);
  Serial.print(" mm, positive=");
  Serial.print(positiveDistance);
  Serial.println(" mm");

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
  float distances[3];

  distances[LIDAR_LEFT]  = readFilteredLidar(LIDAR_LEFT);
  distances[LIDAR_FRONT] = readFilteredLidar(LIDAR_FRONT);
  distances[LIDAR_RIGHT] = readFilteredLidar(LIDAR_RIGHT);

  LidarID closest = LIDAR_NONE;
  float smallestDistance = 9999.0;

  for (int i = 0; i < 3; i++) {
    Serial.print(lidarName((LidarID)i));
    Serial.print(": ");
    Serial.println(distances[i]);

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

  Serial.print("CENTER: selected ");
  Serial.println(lidarName(activeLidar));

  float startDistance = readFilteredLidar(activeLidar);

  float startHeading = getHeadingDegrees();

  Serial.print("CENTER: start distance = ");
  Serial.print(startDistance);
  Serial.print(" mm | heading = ");
  Serial.println(startHeading);

  // =========================================================
  // 2. FORCE ROBOT TO TURN FIRST
  // =========================================================
  int direction = 1;

  Serial.println("CENTER: forcing first turn");

  setSpinPWM(direction * TURN_PWM);
  delay(FORCED_TURN_MS);

  stopRobot();
  delay(SETTLE_TIME_MS);

  float probeDistance = readFilteredLidar(activeLidar);

  float probeHeading = getHeadingDegrees();

  Serial.print("CENTER: probe distance = ");
  Serial.print(probeDistance);
  Serial.print(" mm | heading = ");
  Serial.println(probeHeading);

  // If distance got bigger, reverse direction
  if (probeDistance > startDistance + DIRECTION_THRESHOLD_MM) {
    direction = -1;

    Serial.println(
      "CENTER: distance increased -> reversing search"
    );
  }
  else {
    Serial.println(
      "CENTER: distance decreased -> continuing search"
    );
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

    float currentDistance =
      readFilteredLidar(activeLidar);

    if (isnan(currentDistance)) {
      Serial.println(
        "CENTER: invalid reading during search"
      );

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

      Serial.print("CENTER: new minimum = ");
      Serial.print(bestDistance);

      Serial.print(" mm @ ");
      Serial.print(bestHeading);

      Serial.println(" deg");
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

    Serial.print("CENTER: prev=");
    Serial.print(previousDistance);

    Serial.print(" current=");
    Serial.print(currentDistance);

    Serial.print(" best=");
    Serial.print(bestDistance);

    Serial.print(" heading=");
    Serial.print(currentHeading);

    Serial.print(" rises=");
    Serial.println(risingCount);

    // =====================================================
    // DISTANCE HAS STARTED INCREASING
    // =====================================================
    if (risingCount >= REQUIRED_RISES) {

      stopRobot();

      Serial.println(
        "CENTER: minimum passed"
      );

      Serial.print(
        "CENTER: returning to best heading "
      );

      Serial.println(bestHeading);

      // Return using odometry instead of guessing
      // how long to reverse the motors.
      turnToHeading(bestHeading);

      stopRobot();
      delay(100);

      float finalDistance =
        readFilteredLidar(activeLidar);

      Serial.print(
        "CENTER: final distance = "
      );

      Serial.print(finalDistance);
      Serial.println(" mm");

      Serial.println("CENTERED");

      return true;
    }

    previousDistance =
      currentDistance;
  }

  stopRobot();

  Serial.println(
    "CENTER: search timeout"
  );

  return false;
}

//Gen Ai//
void printToOLED(Maze maze, std::pair<int, int> robot_pos) {
    display.clear();
    display.setScale(1);

    int charWidth = 6;   // Default width of a font character in pixels (scale 1)
    int charHeight = 8;  // Default height of a font character in pixels (scale 1)

    // Iterate through all rows (0 to MAZE_SIZE)
    for (int r = 0; r <= MAZE_SIZE; ++r) {

        // 1. Render Top Horizontal Walls for row 'r'
        display.setCaret(0, r * 2 * charHeight);
        for (int c = 0; c < MAZE_SIZE; ++c) {
            // Check if the bit at column 'c' is set in MAZE_HORIZONTAL_WALLS[c] for level 'r'
            bool wallPresent = (MAZE_HORIZONTAL_WALLS[c] & (1 << r)) != 0;
            
            display.print('+');
            display.print(wallPresent ? '_' : ' ');
        }
        display.print('+'); // End boundary corner

        // Render cell contents and vertical walls (only up to MAZE_SIZE)
        if (r < MAZE_SIZE) {
            display.setCaret(0, (r * 2 + 1) * charHeight);

            for (int c = 0; c <= MAZE_SIZE; ++c) {
                // Check vertical wall at bit position 'c' in row 'r'
                bool wallPresent = (MAZE_VERTICAL_WALLS[r] & (1 << c)) != 0;
                
                display.print(wallPresent ? '|' : ' ');

                // Render robot position or empty cell space
                if (c < MAZE_SIZE) {
                    if (robot_pos.first == r && robot_pos.second == c) {
                        display.print('.');
                    } else {
                        display.print(' ');
                    }
                }
            }
        }
    }

    display.update();
}