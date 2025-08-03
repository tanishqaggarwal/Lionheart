#include "telemetry.h"
#include <atomic>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

std::string get_c_str_of_state(const state_t &state) {
    std::string res;
    res += "[";
#define FIELD(type, name, default_value)                                       \
    res += (std::string(#name) + ":" + std::to_string(state.name) + ",");
    READ_WRITE_FIELDS
    READ_ONLY_FIELDS
#undef FIELD
    res += "]";
    return res;
}

struct TripleBuffer {
    Telemetry buffers[3];

    std::atomic<int> write_idx{0};
    std::atomic<int> read_idx{1};
    std::atomic<int> latest_idx{2};

    std::atomic<bool> new_data{false};

    // Producer writes data
    void write(const Telemetry &data) {
        int write_buffer = write_idx.load();
        buffers[write_buffer] = data;

        int old_latest = latest_idx.exchange(write_buffer);
        write_idx.store(old_latest);

        new_data.store(true);
    }

    // Consumer reads data
    bool read(Telemetry &data) {

        if (!new_data.load()) {
            return false;
        }

        int old_latest = latest_idx.exchange(read_idx.load());
        read_idx.store(old_latest);

        // Possible race condition where write could complete and then we
        // execute new_data.store(false) but it is okay since this is just
        // telemetry.
        new_data.store(false);

        data = buffers[read_idx.load()];
        return true;
    }
};

void listenToTelemetry(TripleBuffer &telemetry_buffer) {

    int serverSocket = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(TELEMETRY_PORT);

    bind(serverSocket, (struct sockaddr *)&addr, sizeof(addr));

    Telemetry telemetry;
    while (1) {
        int bytes_read = recv(serverSocket, static_cast<void *>(&telemetry),
                              sizeof(telemetry), MSG_PEEK);
        if (bytes_read != sizeof(telemetry)) {
            continue;
        }
        bytes_read = recv(serverSocket, static_cast<void *>(&telemetry),
                          sizeof(telemetry), 0);
        if (telemetry.magic_number != MAGIC_NUMBER) {
            std::cerr << "Error: Invalid magic number in telemetry data."
                      << std::endl;
            continue;
        }
        if (telemetry.size != sizeof(Telemetry)) {
            std::cerr << "Error: Invalid telemetry size." << std::endl;
            continue;
        }
        telemetry_buffer.write(telemetry);

        usleep(1000);
    }
}

int main() {

    TripleBuffer telemetry_buffer;

    websocketpp::server<websocketpp::config::asio> server;

    Telemetry telemetry{};
    server.init_asio();
    server.set_message_handler([&](auto hdl, auto msg) {
        telemetry_buffer.read(telemetry);
        server.send(hdl, get_c_str_of_state(telemetry.state),
                    websocketpp::frame::opcode::text);
    });

    server.listen(9002);
    server.start_accept();

    std::cout << "WebSocket on ws://localhost:9002" << std::endl;
    server.run();

    // std::thread(listenToTelemetry).detach();

    return 0;
}