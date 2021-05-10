//
// Created by Kunal Sheth on 5/2/21.
//

#include "display.h"

void displayAppendCommand(DisplayState *d, const uint8_t *command, size_t len) {
    if (d->size + len > sizeof(d->drawCommands)) return;
    memcpy(d->drawCommands + d->size, command, len);
    d->size += len;
}

void displayClear(DisplayState *d) {
    d->size = 0;
    uint8_t command[] = {0x7C, 0x00};
    displayAppendCommand(d, command, sizeof(command));
    displaySetX(d, 0);
    displaySetY(d, 0);
}

void displayDemo(DisplayState *d) {
    uint8_t command[] = {0x7C, 0x04};
    displayAppendCommand(d, command, sizeof(command));
}

void displayInvert(DisplayState *d) {
    uint8_t command[] = {0x7C, 0x12};
    displayAppendCommand(d, command, sizeof(command));
}

void displayBacklight(DisplayState *d, uint8_t dc) {
    uint8_t command[] = {0x7C, 0x02, dc};
    displayAppendCommand(d, command, sizeof(command));
}

void displayText(DisplayState *d, const char *text) {
    displayAppendCommand(d, (const uint8_t *) text, strlen(text));
}

void displaySetX(DisplayState *d, uint8_t position) {
    uint8_t command[] = {0x7C, 0x18, position};
    displayAppendCommand(d, command, sizeof(command));
}

void displaySetY(DisplayState *d, uint8_t position) {
    uint8_t command[] = {0x7C, 0x19, position};
    displayAppendCommand(d, command, sizeof(command));
}

void displaySetPixel(DisplayState *d, uint8_t x, uint8_t y, bool set) {
    uint8_t command[] = {0x7C, 0x10, x, y, set};
    displayAppendCommand(d, command, sizeof(command));
}

void displayDrawLine(DisplayState *d, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, bool set) {
    uint8_t command[] = {0x7C, 0x0C, x1, y1, x2, y2, set};
    displayAppendCommand(d, command, sizeof(command));
}

void displayDrawBox(DisplayState *d, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t set) {
    uint8_t command[] = {0x7C, 0x0F, x1, y1, x2, y2, set};
    displayAppendCommand(d, command, sizeof(command));
}

void displayDrawCircle(DisplayState *d, uint8_t x, uint8_t y, uint8_t rad, uint8_t set) {
    uint8_t command[] = {0x7C, 0x03, x, y, rad, set};
    displayAppendCommand(d, command, sizeof(command));
}
