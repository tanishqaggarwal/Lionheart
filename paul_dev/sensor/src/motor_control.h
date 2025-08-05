#pragma once

#include "depth_controller.h"
#include "state.h"
#include <Arduino.h>
#include <Wire.h>

#define ENA_1_PIN 6
#define IN_1_PIN 4
#define IN_2_PIN 7

struct Motor {

    state_t *state;
    DepthController depth_controller;

    Motor(state_t *state) : state(state) {}

    void initMotorController() {
        Serial.println("Motor controller initialized.");

        pinMode(ENA_1_PIN, OUTPUT);
        pinMode(IN_1_PIN, OUTPUT);
        pinMode(IN_2_PIN, OUTPUT);

        analogWrite(ENA_1_PIN, state->m1_speed);
        digitalWrite(IN_1_PIN, state->m1_in1);
        digitalWrite(IN_2_PIN, state->m1_in2);
    }

    void dispatch(int dt) {
        if (state->use_manual_motor_control) {
            state->m1_speed = state->set_m1_speed;
        } else {
            int force = depth_controller.calculate_force(state->r_ecef_z,
                                                         state->v_ecef_z, dt);
            int motor_speed = min(max(force / alpha_thrust, 0), 255);
            state->m1_speed = motor_speed;
        }
        analogWrite(ENA_1_PIN, state->m1_speed);
    }
};