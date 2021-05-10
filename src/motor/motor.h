//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_MOTOR_H
#define UNICYCLE_MOTOR_H

#include <stdint.h>

typedef struct MotorState {
    int8_t reaction;
    int8_t drive;
} MotorState;

void motorSet(MotorState *m, int8_t reaction, int8_t drive);

#endif //UNICYCLE_MOTOR_H
