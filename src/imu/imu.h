//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_IMU_H
#define UNICYCLE_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

typedef struct ImuMeasurement {
    float x, y, z;
    uint32_t timestampUs;
    float accuracy;
} ImuMeasurement;

typedef struct ImuState {
    ImuMeasurement angle;
    ImuMeasurement angular_velocity;
    ImuMeasurement linear_acceleration;
    ImuMeasurement gravity;
} ImuState;

void quaternionToEuler(float qi, float qj, float qk, float qr,
                       float *x, float *y, float *z);

#endif //UNICYCLE_IMU_H
