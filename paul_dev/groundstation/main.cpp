#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

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
    cfsetospeed(&tty, B9600); // Example: 9600 baud
    cfsetispeed(&tty, B9600);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input

    tty.c_oflag &= ~OPOST; // Raw output

    tcsetattr(fd, TCSANOW, &tty);

    char buffer[256];
    ssize_t bytesRead;

    while (true) {
        bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            std::string data(buffer, bytesRead);
            std::cout << "Received: " << data << std::endl;
        }
        usleep(100000); // Wait 100ms between reads
    }

    close(fd);
    return 0;
}