//
// Created by Kunal Sheth on 5/2/21.
//

#include "current.h"

float adcToAmps(float voltage) {
    return (voltage - V_CENTER) / V_PER_A;
}