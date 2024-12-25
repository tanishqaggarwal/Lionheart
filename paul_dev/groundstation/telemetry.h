namespace Telemetry {

    char START_DELIMITER = char(0xFF);

    struct Imu
    {
        float roll;
        float pitch;
        float heading;

        const static uint32_t PACKET_SIZE_BYTES = 
            sizeof(START_DELIMITER) + 4
            + sizeof(roll) + sizeof(pitch) + sizeof(heading);

        char packet[PACKET_SIZE_BYTES];

        void writePacket()
        {
            packet[0] = START_DELIMITER;
            packet[1] = 3;
            memcpy(&packet[2], &roll, sizeof(roll));
            memcpy(&packet[6], &pitch, sizeof(pitch));
            memcpy(&packet[10], &heading, sizeof(heading));
        }

        void decodePacket(char* arriving_packet)
        {
            memcpy(&roll, &arriving_packet[2], sizeof(roll));
            memcpy(&pitch, &arriving_packet[6], sizeof(pitch));
            memcpy(&heading, &arriving_packet[10], sizeof(heading));
        }
    }
    imu;
}