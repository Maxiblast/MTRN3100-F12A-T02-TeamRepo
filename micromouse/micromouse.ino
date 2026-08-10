#include "Encoder.hpp"
#include "EncoderOdometry.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include <Wire.h>
#include <MPU6050_light.h>
#include <VL6180X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


//Motors 
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
mtrn3100::EncoderOdometry encoder_odometry(16,100);
float odomOriginX = 0.0;
float odomOriginY = 0.0;
float odomOriginH = 0.0;

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

struct Waypoint {
  float x;
  float y;
  float headingDeg;
};

// Replace these example coordinates with your required cell-centre waypoints.
Waypoint missionWaypoints[] = {
  {720.0,   0.0},
};

const int NUM_MISSION_WAYPOINTS =
  sizeof(missionWaypoints) / sizeof(missionWaypoints[0]);

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
float getRobotX();
float getRobotY();
float getRobotH();
int findCenterDirection(LidarID selectedLidar);
LidarID findClosestLidar();
bool centerOnClosestSurface();
bool centerInCell();
bool shiftSideways(float lateralErrorMm);
const char* lidarName(LidarID lidar);

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

}

void loop() {
  static bool missionComplete = false;

  if (missionComplete) {
    stopRobot();
    return;
  }

  // Ensure the route only runs once.
  missionComplete = true;

  // 1. Centre in the starting cell.
  Serial.println("MISSION: centering in start cell");
  centerInCell();

  // 2. Make the centred starting pose the odometry origin (0,0,0).
  updateOdometry();
  odomOriginX = encoder_odometry.getX();
  odomOriginY = encoder_odometry.getY();
  odomOriginH = encoder_odometry.getH();
  Serial.println("MISSION: start cell set to (0,0,0)");

  // 3. Navigate to each waypoint. navigateTo() centres after each arrival.
  for (int i = 0; i < NUM_MISSION_WAYPOINTS; i++) {
    Serial.print("MISSION: navigating to waypoint ");
    Serial.println(i + 1);

    navigateTo(
      missionWaypoints[i].x,
      missionWaypoints[i].y,
      missionWaypoints[i].headingDeg
    );

    Serial.print("MISSION: waypoint ");
    Serial.print(i + 1);
    Serial.println(" reached and centred");
  }

  // The final navigateTo() has already centred at the last waypoint.
  stopRobot();
  Serial.println("MISSION COMPLETE");

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

    Serial.println(
      "ERROR: Left LiDAR failed"
    );

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
  Serial.print(getRobotX());
  Serial.print(",\t\t Y = ");
  Serial.print(getRobotY());
  Serial.print(",\t\t H = ");
  Serial.print(getRobotH());
  Serial.print(",\t\t DeltaLeft = ");
  Serial.print(encoder_odometry.getDeltaLeft());
  Serial.print(",\t\t DeltaRight = ");
  Serial.print(encoder_odometry.getDeltaRight());
  Serial.println();
}

void driveForward(float distance) {
  // Wall-following tuning. Increase WALL_KP if correction is too weak.
  // Increase WALL_KD if it oscillates from side to side.
  const float SIDE_TARGET_MM = 40.0;
  const float WALL_VALID_MAX_MM = 160.0;
  const float WALL_KP = 1.2;
  const float WALL_KD = 0.8;
  const float ENCODER_STEER_KP = 18.0;
  const float MAX_STEERING_PWM = 35.0;
  const unsigned long LIDAR_PERIOD_MS = 35;

  float startLeft = encoder1.getRotation();
  float startRight = encoder2.getRotation();
  controller.integral = 0;
  controller.prev_error = 0;

  float previousWallError = 0.0;
  float steeringCorrection = 0.0;
  unsigned long previousLidarTime = 0;

  float target = distance / WHEEL_RADIUS;
  controller.zeroAndSetTarget(0, target);

  while (true) {
    float leftRot = encoder1.getRotation() - startLeft;
    float rightRot = encoder2.getRotation() - startRight;
    float avgRot = (leftRot + rightRot) / 2.0;

    updateOdometry();  // global pose
    serialPrint();

    int forwardPWM = controller.compute(avgRot);

    // Wall following is only used while driving forwards. Reverse movements
    // used by cell centering remain encoder controlled.
    if (distance > 0.0 && millis() - previousLidarTime >= LIDAR_PERIOD_MS) {
      previousLidarTime = millis();

      float leftDistance = readFilteredLidar(LIDAR_LEFT);
      float rightDistance = readFilteredLidar(LIDAR_RIGHT);

      bool leftWall = !isnan(leftDistance) &&
                      leftDistance >= 5.0 &&
                      leftDistance < WALL_VALID_MAX_MM;
      bool rightWall = !isnan(rightDistance) &&
                       rightDistance >= 5.0 &&
                       rightDistance < WALL_VALID_MAX_MM;

      bool wallAvailable = leftWall || rightWall;

      if (wallAvailable) {
        float wallError;

        if (leftWall && rightWall) {
          // Positive means the robot is closer to the right wall and should
          // steer left.
          wallError = leftDistance - rightDistance;
        } else if (leftWall) {
          wallError = leftDistance - SIDE_TARGET_MM;
        } else {
          wallError = SIDE_TARGET_MM - rightDistance;
        }

        float derivative = wallError - previousWallError;

        // Positive steering makes the robot turn right. Therefore the wall
        // correction uses the negative of the measured wall error.
        steeringCorrection =
          -(WALL_KP * wallError + WALL_KD * derivative);

        previousWallError = wallError;

        Serial.print("WALL FOLLOW | L=");
        Serial.print(leftDistance);
        Serial.print(" R=");
        Serial.print(rightDistance);
        Serial.print(" error=");
        Serial.print(wallError);
        Serial.print(" steer=");
        Serial.println(steeringCorrection);
      } else {
        // In an opening with no side wall, use the encoders to prevent one
        // wheel from gradually travelling farther than the other.
        float encoderError = leftRot - rightRot;
        steeringCorrection = -ENCODER_STEER_KP * encoderError;
        previousWallError = 0.0;
      }

      steeringCorrection = constrain(
        steeringCorrection,
        -MAX_STEERING_PWM,
        MAX_STEERING_PWM
      );
    }

    // Do not let steering reverse a wheel as the distance PID slows down.
    float availableSteering = min(
      MAX_STEERING_PWM,
      abs((float)forwardPWM) * 0.45
    );

    float appliedSteering = constrain(
      steeringCorrection,
      -availableSteering,
      availableSteering
    );

    float leftPWM = forwardPWM + appliedSteering;
    float rightPWM = forwardPWM - appliedSteering;

    leftPWM = constrain(leftPWM, -130.0f, 130.0f);
    rightPWM = constrain(rightPWM, -130.0f, 130.0f);

    motor1.setPWM(-leftPWM);
    motor2.setPWM(rightPWM);

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
  float current_deg = getRobotH() * 180.0 / PI;
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
  float dx = tx - getRobotX();
  float dy = ty - getRobotY();
  float angle_to_target_deg = atan2(dy, dx) * 180.0 / PI;
  while (th_deg > 180.0) th_deg -= 360.0;
  while (th_deg < -180.0) th_deg += 360.0;

  turnToHeading(angle_to_target_deg);

  // drive straight
  float distance = sqrt(dx * dx + dy * dy);
  driveForward(distance);

  // rotate to final heading
  turnToHeading(th_deg);

  // Every call to navigateTo represents arrival at one cell waypoint.
  centerInCell();
}

void purePursuitPath(Waypoint* waypoints, int num_waypoints, float final_heading_deg) {
  const float LOOKAHEAD = 75.0;
  const float MAX_PWM = 130.0;
  const float BASE_SPEED = 60.0;
  const float GOAL_TOLERANCE = 5.0;

  int current_segment = 0;  // which segment we're tracking

  while (true) {
    updateOdometry();

    float rx = getRobotX();
    float ry = getRobotY();
    float rh = getRobotH();

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

    left_pwm  = constrain(left_pwm,  -MAX_PWM, MAX_PWM);
    right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

    motor1.setPWM(-left_pwm);
    motor2.setPWM(right_pwm);

    serialPrint();
    delay(10);
  }

  // final heading correction
  turnToHeading(final_heading_deg);
  centerInCell();
}

// =========================================================
// CENTRE THE ROBOT'S POSITION INSIDE A MAZE CELL
// =========================================================
// Adjust these four values to match the physical robot and maze.
const float CELL_SIZE_MM = 180.0;
const float ROBOT_WIDTH_MM = 100.0;
const float FRONT_SENSOR_TO_ROBOT_CENTRE_MM = 50.0;
const float WALL_READING_LIMIT_MM = 160.0;

// A differential-drive robot cannot strafe. This manoeuvre creates an
// approximately sideways displacement while returning to the same heading:
// turn, move diagonally, restore heading, then cancel the forward displacement.
bool shiftSideways(float lateralErrorMm) {
  const float SHIFT_ANGLE_DEG = 25.0;
  const float MAX_SHIFT_PER_PASS_MM = 25.0;

  lateralErrorMm = constrain(
    lateralErrorMm,
    -MAX_SHIFT_PER_PASS_MM,
    MAX_SHIFT_PER_PASS_MM
  );

  if (abs(lateralErrorMm) < 2.0) return true;

  float originalHeading = getHeadingDegrees();
  float signedAngle = (lateralErrorMm > 0.0)
                        ? SHIFT_ANGLE_DEG
                        : -SHIFT_ANGLE_DEG;
  float diagonalDistance =
    abs(lateralErrorMm) / sin(SHIFT_ANGLE_DEG * PI / 180.0);
  float forwardComponent =
    diagonalDistance * cos(SHIFT_ANGLE_DEG * PI / 180.0);

  turnToHeading(originalHeading + signedAngle);
  driveForward(diagonalDistance);
  turnToHeading(originalHeading);
  driveForward(-forwardComponent);
  turnToHeading(originalHeading);
  stopRobot();

  return true;
}

bool centerInCell() {
  const float SIDE_TARGET_MM =
    (CELL_SIZE_MM - ROBOT_WIDTH_MM) / 2.0;
  const float FRONT_TARGET_MM =
    CELL_SIZE_MM / 2.0 - FRONT_SENSOR_TO_ROBOT_CENTRE_MM;

  const float SIDE_TOLERANCE_MM = 3.0;
  const float FRONT_TOLERANCE_MM = 3.0;
  const int MAX_PASSES = 3;

  stopRobot();
  delay(100);

  for (int pass = 0; pass < MAX_PASSES; pass++) {
    float left = readFilteredLidar(LIDAR_LEFT);
    float right = readFilteredLidar(LIDAR_RIGHT);
    float front = readFilteredLidar(LIDAR_FRONT);

    bool leftWall = !isnan(left) && left < WALL_READING_LIMIT_MM;
    bool rightWall = !isnan(right) && right < WALL_READING_LIMIT_MM;
    bool frontWall = !isnan(front) && front < WALL_READING_LIMIT_MM;

    Serial.print("CELL CENTER pass ");
    Serial.print(pass + 1);
    Serial.print(" | L="); Serial.print(left);
    Serial.print(" R="); Serial.print(right);
    Serial.print(" F="); Serial.println(front);

    // Positive error means the robot must move towards its left side.
    float lateralError = 0.0;
    bool canCorrectSide = false;

    if (leftWall && rightWall) {
      lateralError = (left - right) / 2.0;
      canCorrectSide = true;
    } else if (leftWall) {
      lateralError = left - SIDE_TARGET_MM;
      canCorrectSide = true;
    } else if (rightWall) {
      lateralError = SIDE_TARGET_MM - right;
      canCorrectSide = true;
    }

    if (canCorrectSide && abs(lateralError) > SIDE_TOLERANCE_MM) {
      shiftSideways(lateralError);
      delay(80);
    }

    // A front wall supplies the missing front/back position reference.
    // Positive error means the robot is too far from that wall.
    float forwardError = 0.0;
    if (frontWall) {
      forwardError = front - FRONT_TARGET_MM;
      if (abs(forwardError) > FRONT_TOLERANCE_MM) {
        forwardError = constrain(forwardError, -30.0f, 30.0f);
        driveForward(forwardError);
        stopRobot();
        delay(80);
      }
    }

    bool sideDone = !canCorrectSide ||
                    abs(lateralError) <= SIDE_TOLERANCE_MM;
    bool frontDone = !frontWall ||
                     abs(forwardError) <= FRONT_TOLERANCE_MM;

    if (sideDone && frontDone) {
      Serial.println("CELL CENTER: complete");
      stopRobot();
      return true;
    }
  }

  stopRobot();
  Serial.println("CELL CENTER: stopped after maximum passes");
  return false;
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

  float heading = getRobotH() * 180.0 / PI;

  while (heading > 180.0) heading -= 360.0;
  while (heading < -180.0) heading += 360.0;

  return heading;
}

float getRobotX() {
  float dx = encoder_odometry.getX() - odomOriginX;
  float dy = encoder_odometry.getY() - odomOriginY;
  return cos(odomOriginH) * dx + sin(odomOriginH) * dy;
}

float getRobotY() {
  float dx = encoder_odometry.getX() - odomOriginX;
  float dy = encoder_odometry.getY() - odomOriginY;
  return -sin(odomOriginH) * dx + cos(odomOriginH) * dy;
}

float getRobotH() {
  float heading = encoder_odometry.getH() - odomOriginH;
  while (heading > PI) heading -= 2.0 * PI;
  while (heading < -PI) heading += 2.0 * PI;
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