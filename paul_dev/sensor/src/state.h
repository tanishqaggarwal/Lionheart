#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#define FIELDS                                                                 \
    FIELD(uint32_t, gps_time_nano_s_hi, 0)                                     \
    FIELD(uint32_t, gps_time_nano_s_lo, 0)                                     \
    FIELD(uint32_t, m1_speed, 0)                                               \
    FIELD(bool, m1_en, 0)                                                      \
    FIELD(bool, m1_in1, 0)                                                     \
    FIELD(bool, m1_in2, 0)                                                     \
    FIELD(bool, m1_gnd, 0)                                                     \
    FIELD(uint32_t, m2_speed, 0)                                               \
    FIELD(bool, m2_en, 0)                                                      \
    FIELD(bool, m2_in1, 0)                                                     \
    FIELD(bool, m2_in2, 0)                                                     \
    FIELD(bool, m2_gnd, 0)

struct state_t {
#define FIELD(type, name, default_value) type name;
    FIELDS
#undef FIELD
};

void init_state(state_t &state) {
#define FIELD(type, name, default_value) state.name = default_value;
    FIELDS
#undef FIELD
}

bool process_command(const char *command, int len, state_t &state) {
    bool success = false;
    char cmd_buffer[64];
    memcpy(cmd_buffer, command, len);
    // -1 to remove the trailing newline
    cmd_buffer[len - 1] = '\0';
    String s(cmd_buffer);
    Serial.print("Command Received: ");
    Serial.println(s);
#define FIELD(type, name, default_value)                                       \
    {                                                                          \
        if (strncmp(cmd_buffer, #name, sizeof(#name) - 1) == 0) {              \
            String input = String(&cmd_buffer[sizeof(#name)]);                 \
            int val = input.toInt();                                           \
            if (val == 0 && input != "0") {                                    \
                return false;                                                  \
            }                                                                  \
            state.name = (type)val;                                            \
            success = true;                                                    \
            Serial.print("Success: Set ");                                     \
            Serial.print(#name);                                               \
            Serial.print(" to ");                                              \
            Serial.println(state.name);                                        \
        }                                                                      \
    }
    FIELDS
#undef FIELD

    return success;
}

#undef FIELDS