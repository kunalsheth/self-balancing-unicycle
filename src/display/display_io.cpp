//
// Created by Kunal Sheth on 5/2/21.
//

#include "display_io.h"

void displayIoSetup() {
    Serial1.begin(DISPLAY_DEFAULT_BAUD_RATE);
    Serial1.write(0x7C);
    Serial1.write(0x07);
    Serial1.write(49);
    Serial1.flush();
    Serial1.end();
    Serial1.begin(DISPLAY_SLOW_BAUD_RATE);

    DisplayState d;
    d.size = 0;

    displayClear(&d);
    displayBacklight(&d, DISPLAY_BACKLIGHT_DC);
    displayIoOut(&d);
}

elapsedMillis chunkMs;
void displayIoOut(DisplayState *d) {
    if (chunkMs < DISPLAY_CHUNK_DELAY_MS) return;

    unsigned int size = min(d->size, DISPLAY_CHUNK_SIZE);

    Serial1.write(d->drawCommands, size);
    chunkMs = 0;

    d->size -= size;
    memmove(d->drawCommands, d->drawCommands + size, d->size);
}