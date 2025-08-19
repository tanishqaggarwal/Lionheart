/// Reference: https://learn.adafruit.com/adafruit-9-dof-imu-breakout/software

#include "command_server_manager.h"
#include "motor_control.h"
#include "state.h"
#include "telemetry.h"
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string.h>

struct SystemContext {
    state_t state{};
    CommandServerManager comms;
    Motor motor;
} system_context;

void setup() {

    Serial.begin(115200);
    Serial.println("Setting Up...");

    init_state(system_context.state);

    system_context.comms.init(&system_context.state);
    system_context.motor.initMotorController(&system_context.state);
}

void loop() {
    system_context.comms.dispatch();
    system_context.motor.dispatch(1);

    delay(1000);
}
