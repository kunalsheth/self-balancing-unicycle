//
// Created by Kunal Sheth on 5/6/21.
//

#include "smart_delay.h"

#define NAME_SIZE 8
typedef struct TaskEntry {
    char name[NAME_SIZE];
    uint32_t usEstimate;

    void (*task)();
} TaskEntry;

TaskEntry registry[MAX_TASKS];

bool wasSetup = false;

void smartDelaySetup() {
    if (wasSetup) return;
    for (auto &i : registry) i.task = nullptr;
    wasSetup = true;
}

uint8_t registerBackgroundTask(const char *name, uint32_t usEstimate, void (*f)()) {
    smartDelaySetup();

    for (auto &i : registry)
        if (i.task == nullptr) {
            strncpy(i.name, name, NAME_SIZE);
            i.name[NAME_SIZE - 1] = 0;

            i.usEstimate = usEstimate;
            i.task = f;
            Serial.printf(F("[INFO] registerBackgroundTask(%s, %u, %p) was successful.\n"),
                          name, usEstimate, f);
            return 1;
        }

    Serial.printf(F("[ERROR] registerBackgroundTask(%s, %u, %p) failed.\n"),
                  name, usEstimate, f);
    return 0;
}

elapsedMicros elapsed;

void smartDelayUs(const uint32_t us) {
    elapsed = 0;
    Serial.printf(F("[INFO] smartDelayUs(%u) called.\n"), us);

    smartDelaySetup();

    int i = -1;
    uint32_t lastElapsed = elapsed;
    while (elapsed < us) {
        i++;
        i %= MAX_TASKS;
        if (registry[i].task != nullptr && registry[i].usEstimate < (int64_t) us - elapsed) {
            Serial.printf(F("[INFO] smartDelayUs(%u) calling "), us);
            Serial.print(registry[i].name);
            uint32_t elapsedUs = elapsed;
            Serial.printf(" at %d us.\n", elapsedUs);

            registry[i].task();

            if (elapsed < lastElapsed) { // nightmare bug idk
                Serial.printf(F("[INFO] smartDelayUs(%u) failed.\n"), us);
                return;
            }
            lastElapsed = elapsed;
        }
    }
    Serial.printf(F("[INFO] smartDelayUs(%u) was successful.\n"), us);
}