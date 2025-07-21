#pragma once

#include <stdint.h>

struct state_t {
    // GPS time in nanoseconds
    uint32_t gps_time_nano_s_hi = 0;
    uint32_t gps_time_nano_s_lo = 0;

    // M1
    uint32_t m1_speed = 0;
    bool m1_en = 0;
    bool m1_in1 = 0;
    bool m1_in2 = 0;
    bool m1_gnd = 0;

    // M2
    uint32_t m2_speed = 0;
    bool m2_en = 0;
    bool m2_in1 = 0;
    bool m2_in2 = 0;
    bool m2_gnd = 0;
};