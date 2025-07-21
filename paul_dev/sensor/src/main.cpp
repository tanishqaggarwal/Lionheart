/// Reference: https://learn.adafruit.com/adafruit-9-dof-imu-breakout/software

#include "telemetry.h"
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>
#include <Wire.h>

// Timing variables
float loop_start_ms = 0.0;
float loop_end_ms = 0.0;

unsigned long uptime_ms = 0;

// 20 Hz
const float CONTROL_CYCLE_TIME_ms = 50.0;

Adafruit_LSM303_Accel_Unified accel;
Adafruit_LSM303_Mag_Unified mag;
Adafruit_9DOF dof;

bool init_sensors_done = false;
void init_sensors() {

    Serial.println("Setting up Sensors...");

    if (!accel.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections
         */
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    }
    if (!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections
         */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    }
    init_sensors_done = true;

    Serial.println("Sensor setup complete.");
}

bool init_ethernet_done = false;
EthernetServer server(23);
EthernetClient clients[8];
void init_ethernet() {

    Serial.println("Setting up Ethernet...");

    // Enter a MAC address and IP address for your controller below.
    // The IP address will be dependent on your local network.
    // gateway and subnet are optional:
    byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
    IPAddress ip(192, 168, 1, 177);
    IPAddress myDns(192, 168, 1, 1);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress subnet(255, 255, 0, 0);

    // You can use Ethernet.init(pin) to configure the CS pin
    Ethernet.init(10); // Most Arduino shields
    // Ethernet.init(5);   // MKR ETH Shield
    // Ethernet.init(0);   // Teensy 2.0
    // Ethernet.init(20);  // Teensy++ 2.0
    // Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
    // Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet

    // initialize the Ethernet device
    Ethernet.begin(mac, ip, myDns, gateway, subnet);

    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.  Sorry, can't run "
                       "without hardware. :(");
    }
    Serial.println("Checking ethernet link status.");
    if (Ethernet.linkStatus() == LinkOFF) {
        // W5100 will expect link status UNKNOWN even if the cable is connected.
        Serial.println("Ethernet cable is not connected.");
    }

    // start listening for clients
    server.begin();

    Serial.print("Chat server address:");
    Serial.println(Ethernet.localIP());

    Serial.println("Ethernet setup complete.");

    init_ethernet_done = true;
}

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
    // put your setup code here, to run once:

    Serial.begin(115200);
    Serial.println("Setting Up...");

    init_sensors();
    init_ethernet();
    initMotorController();

    setMotorSpeed(100);
}

void loop() {
    loop_start_ms = static_cast<float>(micros()) / 1000.0;

    if (loop_start_ms - loop_end_ms < CONTROL_CYCLE_TIME_ms) {
        delay(1); // Wait 1 millisecond in between polls.
        return;
    }

    if (init_ethernet_done) {
    }

    if (init_sensors_done) {
        // Do work here:
        sensors_event_t accel_event;
        sensors_event_t mag_event;
        sensors_vec_t orientation;

        accel.getEvent(&accel_event);
        if (dof.accelGetOrientation(&accel_event, &orientation)) {
            Telemetry::imu.roll = orientation.roll;
            Telemetry::imu.pitch = orientation.pitch;
        }

        /* Calculate the heading using the magnetometer */
        mag.getEvent(&mag_event);
        if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation)) {
            Telemetry::imu.heading = orientation.heading;
        }

        loop_end_ms = static_cast<float>(micros()) / 1000.0;

        Telemetry::imu.uptime_ms = static_cast<uint32_t>(millis());
        Telemetry::imu.loop_time_ms = loop_end_ms - loop_start_ms;

        Telemetry::imu.writePacket();
        Serial.write(Telemetry::imu.packet, Telemetry::imu.PACKET_SIZE_BYTES);
    }
}
