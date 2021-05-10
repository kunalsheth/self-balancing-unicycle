//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_CURRENT_IO_H
#define UNICYCLE_CURRENT_IO_H

#include "Arduino.h"

extern "C" {
#include "current.h"
}

#define DRIVE_CURRENT 19
#define REACTION_CURRENT 21

#define ADC_RANGE 1024.f
#define VOLTAGE_DIV (1/3.f)

void currentIoSetup();

void currentIoGetReading(CurrentState *r);

#endif //UNICYCLE_CURRENT_IO_H
