#include "telemetry.h"
#include <atomic>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

// struct TripleBuffer {
//     std::atomic_flag exists_new_data = ATOMIC_FLAG_INIT;
//     Telemetry telemetry;
// };

void listenToTelemetry(std::atomic<bool> &done_flag) {

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

        // Signal to server thread that there is new telemetry data.
        done_flag.store(true, std::memory_order_release);

        sleep(1);
    }
}

int main() {

    // std::atomic<bool> telemetry_processing_done{false};

    websocketpp::server<websocketpp::config::asio> server;

    server.init_asio();
    server.set_message_handler([&](auto hdl, auto msg) {
        server.send(hdl, "Hello from Lionheart!",
                    websocketpp::frame::opcode::text);
    });

    server.listen(9002);
    server.start_accept();

    std::cout << "WebSocket on ws://localhost:9002" << std::endl;
    server.run();

    // std::thread(listenToTelemetry).detach();

    return 0;
}