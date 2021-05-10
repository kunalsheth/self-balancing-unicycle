//
// Created by Kunal Sheth on 5/2/21.
//

#ifndef UNICYCLE_DISPLAY_H
#define UNICYCLE_DISPLAY_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#define MAX_COMMANDS 256

typedef struct DisplayState {
    uint8_t drawCommands[MAX_COMMANDS];
    unsigned int size;
} DisplayState;

void displayClear(DisplayState *d);

void displayDemo(DisplayState *d);

void displayInvert(DisplayState *d);

void displayBacklight(DisplayState *d, uint8_t dc);

void displayText(DisplayState *d, const char *text);

void displaySetX(DisplayState *d, uint8_t position);

void displaySetY(DisplayState *d, uint8_t position);

void displaySetPixel(DisplayState *d, uint8_t x, uint8_t y, bool set);

void displayDrawLine(DisplayState *d, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool set);

void displayDrawBox(DisplayState *d, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t set);

void displayDrawCircle(DisplayState *d, uint8_t x, uint8_t y, uint8_t rad, uint8_t set);

#endif //UNICYCLE_DISPLAY_H
