#pragma once

#include "ports.h"
#include "state.h"

#define MAGIC_NUMBER 0xDEADBEEF

struct __attribute__((packed)) Telemetry {
    uint32_t magic_number;
    uint32_t size;
    state_t state;
};

void packTelemetry(Telemetry &telemetry, state_t &state) {
    telemetry.magic_number = MAGIC_NUMBER;
    telemetry.size = sizeof(Telemetry);
    telemetry.state = state;
}
