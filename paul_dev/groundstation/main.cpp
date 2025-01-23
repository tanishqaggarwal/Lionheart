#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <fstream>

#include "telemetry.h"

#include <ctime>

constexpr uint32_t BUFFER_SIZE = 256;

int main(int argc, char *argv[]) {

    // Check if the port name is provided as an argument
    // Ex. "/dev/tty.usbserial"
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_port>" << std::endl;
        return 1;
    }

    const char *portName = argv[1];

    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Error opening serial port" << std::endl;
        return -1;
    }

    termios tty;
    tcgetattr(fd, &tty);

    // Set serial port settings (baud rate, parity, etc.)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input

    tty.c_oflag &= ~OPOST; // Raw output

    tcsetattr(fd, TCSANOW, &tty);

    char buffer[BUFFER_SIZE];
    ssize_t bytesRead;

    time_t now = time(0);

    char filename[100];
    std::snprintf(filename, sizeof(filename), "paul_dev/data/%lu.txt", now);
    std::ofstream outputFile(filename);

    outputFile << "Roll,Pitch,Heading,Uptime_ms,LoopTime_ms\n";
    while (true) {
        bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            for (uint32_t i=0; i < bytesRead; ++i)
            {
                if (Telemetry::imu.processByte(buffer[i]))
                {
                    outputFile << Telemetry::imu.roll << "," << Telemetry::imu.pitch << "," << Telemetry::imu.heading << "," << Telemetry::imu.uptime_ms << "," << Telemetry::imu.loop_time_ms << "\n";
                }
            }
        }
        usleep(100000); // Wait 100ms between reads
    }

    close(fd);
    return 0;
}