//
// Created by Kunal Sheth on 5/2/21.
//

#include "imu.h"

void quaternionToEuler(float qi, float qj, float qk, float qr,
                       float *x, float *y, float *z) {
    float sqr = qr * qr;
    float sqi = qi * qi;
    float sqj = qj * qj;
    float sqk = qk * qk;

    *x = atan2f(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
    *y = asinf(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    *z = atan2f(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
}