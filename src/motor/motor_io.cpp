//
// Created by Kunal Sheth on 5/2/21.
//

#include "motor_io.h"

void motorIoSetup() {
    pinMode(REACTION_MOTOR_PWM, OUTPUT);
    pinMode(REACTION_MOTOR_DIR, OUTPUT);
    pinMode(DRIVE_MOTOR_PWM, OUTPUT);
    pinMode(DRIVE_MOTOR_DIR, OUTPUT);
}

void motorIoOut(MotorState *m) {
    digitalWrite(REACTION_MOTOR_DIR, (m->reaction < 0) == REACTION_MOTOR_POLARITY);
    analogWrite(REACTION_MOTOR_PWM, abs(m->reaction));

    digitalWrite(DRIVE_MOTOR_DIR, (m->drive < 0) == DRIVE_MOTOR_POLARITY);
    analogWrite(DRIVE_MOTOR_PWM, abs(m->drive));
}