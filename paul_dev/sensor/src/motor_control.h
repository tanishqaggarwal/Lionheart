#pragma once

#include "depth_controller.h"
#include "state.h"
#include <Arduino.h>
#include <Wire.h>

#define M1_ENA_1_PIN 6
#define M1_IN_1_PIN 7
#define M1_IN_2_PIN 4

#define M2_ENA_1_PIN 5
#define M2_IN_1_PIN 3
#define M2_IN_2_PIN 2

struct Motor {

    state_t *state;
    DepthController depth_controller;

    void initMotorController(state_t *state_) {
        state = state_;
        Serial.println("Motor controller initialized.");

        pinMode(M1_ENA_1_PIN, OUTPUT);
        pinMode(M1_IN_1_PIN, OUTPUT);
        pinMode(M1_IN_2_PIN, OUTPUT);

        analogWrite(M1_ENA_1_PIN, state->m1_speed);
        digitalWrite(M1_IN_1_PIN, state->m1_in1);
        digitalWrite(M1_IN_2_PIN, state->m1_in2);

        pinMode(M2_ENA_1_PIN, OUTPUT);
        pinMode(M2_IN_1_PIN, OUTPUT);
        pinMode(M2_IN_2_PIN, OUTPUT);

        analogWrite(M2_ENA_1_PIN, state->m2_speed);
        digitalWrite(M2_IN_1_PIN, state->m2_in1);
        digitalWrite(M2_IN_2_PIN, state->m2_in2);
    }

    void dispatch(int dt) {
        if (state->use_manual_motor_control) {
            state->m1_speed = state->set_m1_speed;
            state->m2_speed = state->set_m2_speed;
        } else {
            int force = depth_controller.calculate_force(state->r_ecef_z,
                                                         state->v_ecef_z, dt);
            int motor_speed = min(max(force / alpha_thrust, 0), 255);
            state->m1_speed = motor_speed;
            state->m2_speed = motor_speed;
        }
        analogWrite(M1_ENA_1_PIN, state->m1_speed);
        analogWrite(M2_ENA_1_PIN, state->m2_speed);
    }
};