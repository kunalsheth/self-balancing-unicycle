//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_MOTOR_IO_H
#define UNICYCLE_MOTOR_IO_H

#include <Arduino.h>

extern "C" {
#include "motor.h"
}

#define REACTION_MOTOR_PWM 12
#define REACTION_MOTOR_DIR 13
#define REACTION_MOTOR_POLARITY false

#define DRIVE_MOTOR_PWM 14
#define DRIVE_MOTOR_DIR 15
#define DRIVE_MOTOR_POLARITY false

void motorIoSetup();

void motorIoOut(MotorState *m);

#endif //UNICYCLE_MOTOR_IO_H
