#pragma once

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
