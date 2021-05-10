//
// Created by Kunal Sheth on 5/6/21.
//

#ifndef UNICYCLE_SMART_DELAY_H
#define UNICYCLE_SMART_DELAY_H

#include <stdint.h>
#include <string.h>
#include <Arduino.h>

#define MAX_TASKS 8

uint8_t registerBackgroundTask(const char *name, uint32_t usEstimate, void (*f)());

void smartDelayUs(uint32_t us);

#endif //UNICYCLE_SMART_DELAY_H
