#pragma once

#include "state.h"
#include <Arduino.h>
#include <Wire.h>

#define ENA_1_PIN 6
#define IN_1_PIN 4
#define IN_2_PIN 7

struct Motor {

    state_t *state;

    void initMotorController() {
        Serial.println("Motor controller initialized.");

        pinMode(ENA_1_PIN, OUTPUT);
        pinMode(IN_1_PIN, OUTPUT);
        pinMode(IN_2_PIN, OUTPUT);

        dispatch();
    }

    void dispatch() {
        analogWrite(ENA_1_PIN, state->m1_speed);
        digitalWrite(IN_1_PIN, state->m1_in1);
        digitalWrite(IN_2_PIN, state->m1_in2);
    }
};