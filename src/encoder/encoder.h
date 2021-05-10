//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_ENCODER_H
#define UNICYCLE_ENCODER_H

#include <stdint.h>

#define REACTION_TICKS_PER_ROT (400.0 * 1./1.) // https://www.pololu.com/product/4757
#define DRIVE_TICKS_PER_ROT (-640.0 * 32./13.) // https://www.pololu.com/product/4758

typedef struct EncoderState {
    uint32_t timestamp;
    double reactionRotations;
    double reactionRps;
    double driveRotations;
    double driveRps;
} EncoderState;

double reactionTicksToRotations(int32_t ticks);

double driveTicksToRotations(int32_t ticks);

#endif //UNICYCLE_ENCODER_H
