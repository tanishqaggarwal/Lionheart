#pragma once

#include "telemetry.h"
#include <iostream>

struct TelemetryProcessor {

    enum class State {
        IDLE,
        SIZE,
        PROCESSING,
    };

    char buffer[sizeof(Telemetry)] = {};
    size_t buffer_size = 0;
    State state = State::IDLE;

    // Returns true if the buffer is complete with a valid telemetry packet.
    bool processChar(char c) {
        switch (state) {
        case State::IDLE:
            buffer[buffer_size++] = c;
            if (buffer_size == 4) {
                if (((uint32_t *)buffer)[0] == MAGIC_NUMBER) {
                    state = State::SIZE;
                } else {
                    buffer[0] = buffer[1];
                    buffer[1] = buffer[2];
                    buffer[2] = buffer[3];
                    buffer_size = 3;
                }
            }
            return false;
        case State::SIZE: {
            buffer[buffer_size++] = c;
            if (buffer_size == 8) {
                if (((uint32_t *)buffer)[1] == sizeof(Telemetry)) {
                    state = State::PROCESSING;
                } else {
                    std::cout
                        << "Invalid telemetry size: " << ((uint32_t *)buffer)[1]
                        << std::endl;
                    std::cout << "Expected size: " << sizeof(Telemetry)
                              << std::endl;
                    state = State::IDLE;
                    buffer_size = 0;
                }
            }
            return false;
        }
        case State::PROCESSING:
            buffer[buffer_size++] = c;
            if (buffer_size == sizeof(Telemetry)) {
                state = State::IDLE;
                buffer_size = 0;
                return true;
            }
            return false;
        }
        // Should never reach here, but add return for safety
        return false;
    }
};