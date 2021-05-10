//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_CURRENT_H
#define UNICYCLE_CURRENT_H

#include <stdint.h>

#define V_PER_A (0.4f) // https://www.pololu.com/product/4041
#define V_CENTER (2.5f) // https://www.pololu.com/product/4041

typedef struct CurrentState {
    uint32_t timestamp;
    float reaction;
    float drive;
} CurrentState;

float adcToAmps(float voltage);

#endif //UNICYCLE_CURRENT_H
