#pragma once

#include "hardware_configs.h"
#include "state.h"

#define Kp 1.0
#define Ki 1.0

struct DepthController {

    int integral_factor = 0;

    // Calculates the force required to maintain the desired depth
    int calculate_force(int r_ecef_z, int v_ecef_z, int dt) {
        integral_factor += v_ecef_z * dt;
        int force = Kp * r_ecef_z + Ki * integral_factor;
        return force;
    }
};