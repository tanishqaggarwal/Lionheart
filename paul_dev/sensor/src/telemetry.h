#pragma once

#include "state.h"

#define TELEMETRY_PORT 3001
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
