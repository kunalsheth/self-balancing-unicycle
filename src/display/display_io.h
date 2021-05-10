//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_DISPLAY_IO_H
#define UNICYCLE_DISPLAY_IO_H

#include <Arduino.h>

extern "C" {
#include "display.h"
}

#define DISPLAY_DEFAULT_BAUD_RATE 115200
#define DISPLAY_SLOW_BAUD_RATE 4800

#define DISPLAY_CHUNK_SIZE 64
#define DISPLAY_CHUNK_DELAY_MS 250

#define DISPLAY_BACKLIGHT_DC 25

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

void displayIoSetup();

void displayIoOut(DisplayState *d);

#endif //UNICYCLE_DISPLAY_IO_H
