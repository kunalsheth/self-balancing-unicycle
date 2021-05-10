//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_ENCODER_IO_H
#define UNICYCLE_ENCODER_IO_H

//#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Encoder.h>

extern "C" {
#include "encoder.h"
}

#define REACTION_CHANNEL_A 6
#define REACTION_CHANNEL_B 10

#define DRIVE_CHANNEL_A 5
#define DRIVE_CHANNEL_B 9

void encoderIoSetup();

void encoderIoGetReading(EncoderState *r);

#endif //UNICYCLE_ENCODER_IO_H
