#pragma once

#include "hardware_configs.h"
#include "state.h"

#define Kp 1.0
#define Ki 1.0

struct DepthController {

    int integral_factor = 0;

    // Calculates the force required to maintain the desired depth
    int calculate_force(int r_ecef_z, int target_z, int dt) {
        int error = target_z - r_ecef_z;
        integral_factor += error * dt;
        int force = Kp * error + Ki * integral_factor;
        return force + F_buoyancy;
    }
};