//
// Created by Kunal Sheth on 5/2/21.
//

#include "current_io.h"

void currentIoSetup() {
    pinMode(REACTION_CURRENT, INPUT_PULLUP);
    pinMode(DRIVE_CURRENT, INPUT_PULLUP);
}

void currentIoGetReading(CurrentState *r) {
    r->timestamp = micros();
    r->reaction = adcToAmps((float) analogRead(REACTION_CURRENT) / VOLTAGE_DIV / ADC_RANGE);
    r->drive = adcToAmps((float) analogRead(DRIVE_CURRENT) / VOLTAGE_DIV / ADC_RANGE);
}