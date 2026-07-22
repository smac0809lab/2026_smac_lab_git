#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <Arduino.h>

namespace Vehicle {
    const float WHEEL_DIAMETER_MM = 270.0;
    const float TICKS_PER_REV     = 300.0;
    const float CONTROL_DT_MS     = 50.0;
    const int PWM_FREQ = 500;
    const int PWM_RES  = 8;
}

struct MotorPin { uint8_t pwm; uint8_t ena; uint8_t enb; };

// 핀 맵 정의
const MotorPin FRONT_MOTOR = {32, 33, 22};
const MotorPin REAR_MOTOR  = {4, 13, 14};
const MotorPin STEER_MOTOR = {25, 26, 27};

const uint8_t ENC_CS_PIN = 5;
const uint8_t STR_POT_PIN = 34;

#endif  