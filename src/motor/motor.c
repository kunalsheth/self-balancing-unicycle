//
// Created by Kunal Sheth on 5/2/21.
//

#include "motor.h"

void motorSet(MotorState *m, int8_t reaction, int8_t drive) {
    m->reaction = reaction;
    m->drive = drive;
}