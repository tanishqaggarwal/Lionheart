/// Reference: https://learn.adafruit.com/adafruit-9-dof-imu-breakout/software

#include "ethernet.h"
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
    state_t *state;
    CommandServerManager *comms;

} system_context;

// Timing variables
float loop_start_ms = 0.0;
float loop_end_ms = 0.0;

unsigned long uptime_ms = 0;

// 20 Hz
const float CONTROL_CYCLE_TIME_ms = 50.0;

// Adafruit_LSM303_Accel_Unified accel;
// Adafruit_LSM303_Mag_Unified mag;
// Adafruit_9DOF dof;

// bool init_sensors_done = false;
// void init_sensors() {

//     Serial.println("Setting up Sensors...");

//     if (!accel.begin()) {
//         /* There was a problem detecting the LSM303 ... check your
//         connections
//          */
//         Serial.println(F("Ooops, no LSM303 detected ... Check your
//         wiring!")); return;
//     }
//     if (!mag.begin()) {
//         /* There was a problem detecting the LSM303 ... check your
//         connections
//          */
//         Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
//         return;
//     }
//     init_sensors_done = true;

//     Serial.println("Sensor setup complete.");
// }

#define ENA_1_PIN 6
#define IN_1_PIN 4
#define IN_2_PIN 7
void initMotorController() {
    // Placeholder for motor controller initialization
    Serial.println("Motor controller initialized.");

    pinMode(ENA_1_PIN, OUTPUT);
    pinMode(IN_1_PIN, OUTPUT);
    pinMode(IN_2_PIN, OUTPUT);

    analogWrite(ENA_1_PIN, 0);
    digitalWrite(IN_1_PIN, LOW);
    digitalWrite(IN_2_PIN, LOW);
}

void setMotorSpeed(int speed) {
    // Placeholder for setting motor speed
    Serial.println("Setting motor speed.");
    analogWrite(ENA_1_PIN, speed);
    digitalWrite(IN_1_PIN, HIGH);
    digitalWrite(IN_2_PIN, LOW);
}

void setup() {

    Serial.begin(115200);
    Serial.println("Setting Up...");

    system_context.state = new state_t();
    system_context.comms = new CommandServerManager(system_context.state);
    system_context.comms->init();

    // put your setup code here, to run once:
    // init_sensors();
    initMotorController();

    setMotorSpeed(100);
}

void loop() {
    loop_start_ms = static_cast<float>(micros()) / 1000.0;
    // if (loop_start_ms - loop_end_ms < CONTROL_CYCLE_TIME_ms) {
    //     delay(1); // Wait 1 millisecond in between polls.
    //     return;
    // }
    delay(1000);

    system_context.comms->dispatch();

    // if (init_sensors_done) {
    //     // Do work here:
    //     sensors_event_t accel_event;
    //     sensors_event_t mag_event;
    //     sensors_vec_t orientation;

    //     accel.getEvent(&accel_event);
    //     if (dof.accelGetOrientation(&accel_event, &orientation)) {
    //         Telemetry::imu.roll = orientation.roll;
    //         Telemetry::imu.pitch = orientation.pitch;
    //     }

    //     /* Calculate the heading using the magnetometer */
    //     mag.getEvent(&mag_event);
    //     if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
    //         Telemetry::imu.heading = orientation.heading;
    //     }

    //     Telemetry::imu.uptime_ms = static_cast<uint32_t>(millis());
    //     Telemetry::imu.loop_time_ms = loop_end_ms - loop_start_ms;

    //     Telemetry::imu.writePacket();
    //     Serial.write(Telemetry::imu.packet,
    //     Telemetry::imu.PACKET_SIZE_BYTES);
    // }

    loop_end_ms = static_cast<float>(micros()) / 1000.0;
}
