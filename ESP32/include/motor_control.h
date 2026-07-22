#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "vehicle_config.h"

void initMotors();
void moveMotor(MotorPin motor, int channel, int speed, bool stop);
void driveVehicle(int drive_pwm, int steer_pwm, bool stop);

#endif