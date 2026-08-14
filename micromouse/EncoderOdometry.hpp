#pragma once

#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
  EncoderOdometry(float radius, float wheelBase, float imuAlpha = 0.98) : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0), ALPHA(imuAlpha) {}

  void update(float leftValue, float rightValue, float imuHeadingRad) {
    delta_left_radians = -(lastLPos - leftValue); // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION
    delta_right_radians = -(lastRPos - rightValue); // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION

    float delta_s = (R * delta_left_radians)/2 + (R * delta_right_radians)/2;
    float delta_theta = (R * delta_right_radians)/B - (R * delta_left_radians)/B;

    x += delta_s * cos(h);
    y += delta_s * sin(h);

    float h_encoder = h + delta_theta;
    h = ALPHA * h_encoder + (1.0f - ALPHA) * imuHeadingRad;

    lastLPos = leftValue;
    lastRPos = rightValue;
  }

  void setHeading(float headingRad) { h = headingRad; }

  void setPosition(float startX, float startY) {
    x = startX;
    y = startY;
  }

  float getX() const { return x; }
  float getY() const { return y; }
  float getH() const { return h; }
  float getDeltaLeft() const { return delta_left_radians; }
  float getDeltaRight() const { return delta_right_radians; }

private:
  float x, y, h;
  const float R, B;
  float lastLPos, lastRPos;
  float delta_left_radians, delta_right_radians;
  const float ALPHA;
};

}