//
// Created by Kunal Sheth on 5/2/21.
//

#include "encoder_io.h"

Encoder reaction(REACTION_CHANNEL_A, REACTION_CHANNEL_B);
Encoder drive(DRIVE_CHANNEL_A, DRIVE_CHANNEL_B);

void encoderIoSetup() {
    reaction.write(0);
    drive.write(0);
}

void encoderIoGetReading(EncoderState *r) {
    const uint32_t lastTimestamp = r->timestamp;

    const uint32_t timestamp = micros();
    const double driveDeltaRot = driveTicksToRotations(drive.readAndReset());
    const double reactionDeltaRot = reactionTicksToRotations(
            abs(reaction.readAndReset()) // broken wiring
    );

    double dt = abs((double) timestamp - lastTimestamp);
    if (dt == 0) dt++;

    r->reactionRps = reactionDeltaRot / dt * 1E6;
    r->reactionRotations += reactionDeltaRot;

    r->driveRps = driveDeltaRot / dt * 1E6;
    r->driveRotations += driveDeltaRot;
}