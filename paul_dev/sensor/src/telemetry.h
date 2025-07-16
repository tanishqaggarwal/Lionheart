#pragma once

#include <stdint.h>
#include <string.h>

namespace Telemetry {

char START_DELIMITER = char(0xFF);

struct Imu {
    float roll;
    float pitch;
    float heading;
    uint32_t uptime_ms;
    float loop_time_ms;

    const static uint32_t PACKET_SIZE_BYTES =
        sizeof(START_DELIMITER) +
        sizeof(uint32_t) + // Reserved field for later differentiating between
                           // packet types.
        sizeof(roll) + sizeof(pitch) + sizeof(heading) + sizeof(uptime_ms) +
        sizeof(loop_time_ms);

    char packet[PACKET_SIZE_BYTES];
    uint32_t packet_i = 0;

    void writePacket() {
        packet[0] = START_DELIMITER;
        // memcpy(&packet[1], &roll, sizeof(roll));
        memcpy(&packet[5], &roll, sizeof(roll));
        memcpy(&packet[9], &pitch, sizeof(pitch));
        memcpy(&packet[13], &heading, sizeof(heading));
        memcpy(&packet[17], &uptime_ms, sizeof(uptime_ms));
        memcpy(&packet[21], &loop_time_ms, sizeof(loop_time_ms));
    }

    void _decodePacket(char *arriving_packet) {
        memcpy(&roll, &arriving_packet[5], sizeof(roll));
        memcpy(&pitch, &arriving_packet[9], sizeof(pitch));
        memcpy(&heading, &arriving_packet[13], sizeof(heading));
        memcpy(&uptime_ms, &arriving_packet[17], sizeof(uptime_ms));
        memcpy(&loop_time_ms, &arriving_packet[21], sizeof(loop_time_ms));
    }

    bool processByte(char byte) {
        if (byte == START_DELIMITER) {
            packet_i = 0;
        }

        packet[packet_i++] = byte;

        if (packet_i == PACKET_SIZE_BYTES) {
            _decodePacket(packet);
            return true;
        }

        return false;
    }
} imu;
} // namespace Telemetry