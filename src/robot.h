//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_ROBOT_STATE_H
#define UNICYCLE_ROBOT_STATE_H

#include <stdint.h>
#include "display/display.h"
#include "imu/imu.h"
#include "encoder/encoder.h"
#include "motor/motor.h"

typedef struct RobotState {
    DisplayState display;
    ImuState imu;
    EncoderState encoder;
    CurrentState current;
    MotorState motor;
} RobotState;

#endif //UNICYCLE_ROBOT_STATE_H
