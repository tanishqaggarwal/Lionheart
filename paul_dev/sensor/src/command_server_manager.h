#pragma once

#include "state.h"
#include "telemetry.h"
#include <Ethernet.h>
#include <EthernetUdp.h>

#define NET_PORT 3000

struct CommandServerManager {

    bool initialized = false;
    IPAddress control_panel_addr;
    int control_panel_port = -1;

    state_t *state;

    EthernetUDP net_driver;

    Telemetry telemetry;

    CommandServerManager(state_t *state) { state = state; }

    void init() {

        Serial.println("Setting up Ethernet...");

        // Enter a MAC address and IP address for your controller below.
        // The IP address will be dependent on your local network.
        // gateway and subnet are optional:
        byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
        IPAddress ip(192, 168, 1, 177);
        IPAddress myDns(192, 168, 1, 177);
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
            return;
        }
        Serial.println("Checking ethernet link status.");
        if (Ethernet.linkStatus() == LinkOFF) {
            // W5100 will expect link status UNKNOWN even if the cable is
            // connected.
            Serial.println("Ethernet cable is not connected.");
            return;
        }

        net_driver.begin(NET_PORT);

        Serial.print("Chat server address:");
        Serial.println(Ethernet.localIP());

        Serial.println("Ethernet setup complete.");

        initialized = true;
    }

    void dispatch() {
        if (!initialized) {
            return;
        }

        if (control_panel_port == -1) {
            listen_for_command_server();
        } else {
            read_command();
        }

        sendTelemetry();
    }

  private:
    void listen_for_command_server() {
        unsigned char net_buffer[64];
        int packet_size = net_driver.parsePacket();
        if (packet_size == 0) {
            return;
        }
        if (packet_size > sizeof(net_buffer)) {
            Serial.println("Error: Received packet too large. Emptying.");
            while (net_driver.read(net_buffer, sizeof(net_buffer)) != -1) {
                // Keep reading until the buffer is empty
            }
            return;
        }

        int bytes_read = net_driver.read(net_buffer, sizeof(net_buffer));
        if (bytes_read != 0 &&
            strncmp(reinterpret_cast<const char *>(net_buffer), "connect", 7) ==
                0) {
            control_panel_port = net_driver.remotePort();
            control_panel_addr = net_driver.remoteIP();
            net_driver.beginPacket(control_panel_addr, control_panel_port);
            net_driver.write("Connection Success!\n");
            net_driver.endPacket();
            Serial.print("Connection Success! Control panel port: ");
            Serial.println(control_panel_port);
        }
    }

    void sendTelemetry() {
        packTelemetry(telemetry, *state);
        net_driver.beginPacket(control_panel_addr, control_panel_port);
        net_driver.write(reinterpret_cast<const char *>(&telemetry),
                         sizeof(telemetry));
        net_driver.endPacket();
    }

    bool unloadTelemetry(const char *buffer, Telemetry &telemetry) {
        memcpy(&telemetry, buffer, sizeof(Telemetry));
        if (telemetry.magic_number != MAGIC_NUMBER) {
            Serial.println("Error: Invalid magic number in telemetry data.");
            return false;
        }
        if (telemetry.size != sizeof(Telemetry)) {
            Serial.println("Error: Invalid telemetry size.");
            return false;
        }
        return true;
    }

    void read_command() {
        unsigned char net_buffer[64];
        int packet_size = net_driver.parsePacket();
        if (packet_size == 0) {
            return;
        }
        if (packet_size > sizeof(net_buffer)) {
            Serial.println("Error: Received packet too large. Emptying.");
            while (net_driver.read(net_buffer, sizeof(net_buffer)) != -1) {
                // Keep reading until the buffer is empty
            }
            return;
        }

        int bytes_read = net_driver.read(net_buffer, sizeof(net_buffer));
        bool success = process_command(
            reinterpret_cast<const char *>(net_buffer), bytes_read, *state);

        net_driver.beginPacket(control_panel_addr, control_panel_port);
        if (success) {
            net_driver.write("Command Success!\n");
        } else {
            net_driver.write("Command Failed!\n");
        }
        net_driver.endPacket();
    }
};