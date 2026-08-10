#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class VelocityController {
public:
    VelocityController(float kp, float ki, float kd, int16_t maxPWM)
        : kp(kp), ki(ki), kd(kd), maxPWM(maxPWM) {}

    // Call every control loop tick with the wheel's current raw rotation (radians) and the
    // desired angular velocity (rad/s). Returns a PWM command clamped to +-maxPWM.
    int compute(float currentRotationRad, float targetVelocityRadPerSec) {
        unsigned long currentTime = micros();
        float dt = 0.0f;

        if (previousTime != 0) {
            dt = (currentTime - previousTime) / 1000000.0f;
        }

        previousTime = currentTime;

        if (dt > 0.0f) {
            measuredVelocity = (currentRotationRad - prevRotationRad) / dt;
        } else {
            measuredVelocity = 0.0f;
        }
        prevRotationRad = currentRotationRad;

        error = targetVelocityRadPerSec - measuredVelocity;
        integral += error * dt;
        if (dt > 0.0f) {
            derivative = (error - prev_error) / dt;
        } else {
            derivative = 0.0f;
        }
        prev_error = error;

        output = kp * error + ki * integral + kd * derivative;

        int16_t pwm = constrain(output, -maxPWM, maxPWM);
        return pwm;
    }

    // Call before starting a new run of this wheel's control loop so the next compute() doesn't
    // use the old prevRotationRad/prev_time (which would spike dt) or carry over old integral windup.
    void reset(float currentRotationRad) {
        prevRotationRad = currentRotationRad;
        previousTime = 0;
        integral = 0;
        prev_error = 0;
    }

    float getMeasuredVelocity() const { return measuredVelocity; }

public:
    float kp, ki, kd;
    int maxPWM;
    float error = 0, derivative = 0, integral = 0, output = 0;
    float prev_error = 0;
    float measuredVelocity = 0;
    float prevRotationRad = 0;
    unsigned long previousTime = 0;
};

}  // namespace mtrn3100
