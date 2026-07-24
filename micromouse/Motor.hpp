#pragma once

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {
        if (abs(pwm) >= 0 && abs(pwm) <= 255) {
            if (pwm >= 0) {
                digitalWrite(dir_pin, HIGH);   // when dir = high rotation is clockwise.
            }
            else {
                digitalWrite(dir_pin, LOW);
            }
            analogWrite(pwm_pin, abs(pwm));
        }
      // TODO: Output digital direction pin based on if input signal is positive or negative.
      // TODO: Output PWM signal between 0 - 255.
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
