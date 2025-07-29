#pragma once

#include "state.h"

#define MAGIC_NUMBER 0xDEADBEEF

struct Telemetry {
    uint32_t magic_number;
    uint32_t size;
    state_t state;
};

void packTelemetry(Telemetry &telemetry, state_t &state) {
    telemetry.magic_number = MAGIC_NUMBER;
    telemetry.size = sizeof(Telemetry);
    telemetry.state = state;
}

bool unloadTelemetry(const char *buffer, Telemetry &telemetry) {
    memcpy(&telemetry, buffer, sizeof(Telemetry));
    if (telemetry.magic_number != MAGIC_NUMBER) {
        Serial.println("Error: Invalid magic number in telemetry data.");
        return false;
    }
    if (telemetry.size != sizeof(Telemetry)) {
        Serial.println("Error: Invalid telemetry size.");
        return false;
    }
    return true;
}