#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

int main() {

    int serverSocket = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(0);

    bind(serverSocket, (struct sockaddr *)&addr, sizeof(addr));

    socklen_t len = sizeof(addr);
    getsockname(serverSocket, (struct sockaddr *)&addr, &len);
    int assigned_port = ntohs(addr.sin_port);
    printf("Assigned port: %d\n", assigned_port);

    char buffer[1024];
    while (1) {
        int bytes_read = recv(serverSocket, buffer, sizeof(buffer), 0);
        std::cout.write(buffer, bytes_read);
        std::cout << std::endl;

        sleep(1);
    }

    return 0;
}