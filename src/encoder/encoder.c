//
// Created by Kunal Sheth on 5/2/21.
//

#include "encoder.h"

double reactionTicksToRotations(int32_t ticks) {
    return (double) ticks / REACTION_TICKS_PER_ROT;
}

double driveTicksToRotations(int32_t ticks) {
    return (double) ticks / DRIVE_TICKS_PER_ROT;
}