#pragma once

#ifndef __AVR__
#include <cstdlib>
#else
#include <stdlib.h>
#endif

#include <stdint.h>
#include <string.h>

#define READ_ONLY_FIELDS                                                       \
    FIELD(uint32_t, gps_time_nano_s_hi, 0)                                     \
    FIELD(uint32_t, gps_time_nano_s_lo, 0)                                     \
    FIELD(bool, m1_in1, 0)                                                     \
    FIELD(bool, m1_in2, 0)                                                     \
    FIELD(bool, m2_in1, 0)                                                     \
    FIELD(bool, m2_in2, 0)

#define READ_WRITE_FIELDS                                                      \
    FIELD(uint32_t, m1_speed, 0)                                               \
    FIELD(uint32_t, m2_speed, 0)

struct state_t {
#define FIELD(type, name, default_value) type name;
    READ_WRITE_FIELDS
    READ_ONLY_FIELDS
#undef FIELD
};

void init_state(state_t &state) {
#define FIELD(type, name, default_value) state.name = default_value;
    READ_WRITE_FIELDS
    READ_ONLY_FIELDS
#undef FIELD
}

bool process_command(const char *command, int len, state_t &state) {
    bool success = false;
    char cmd_buffer[64];
    memcpy(cmd_buffer, command, len);
    // -1 to remove the trailing newline
    cmd_buffer[len - 1] = '\0';
#define FIELD(type, name, default_value)                                       \
    {                                                                          \
        if (strncmp(cmd_buffer, #name, sizeof(#name) - 1) == 0) {              \
            int val = atoi(&cmd_buffer[sizeof(#name)]);                        \
            if (val == 0 && cmd_buffer[sizeof(#name)] != '0') {                \
                return false;                                                  \
            }                                                                  \
            state.name = (type)val;                                            \
            success = true;                                                    \
        }                                                                      \
    }
    READ_WRITE_FIELDS
#undef FIELD

    return success;
}

#undef READ_WRITE_FIELDS