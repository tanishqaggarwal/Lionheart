#include "telemetry.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <netinet/in.h>
#include <set>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#define CMD_SERVER_PORT 9001
#define MONITOR_SERVER_PORT 9002

std::string get_json_of_state(const state_t &state) {
    std::string res;
    res += "{";
#define FIELD(type, name, default_value)                                       \
    res += ("\"" + std::string(#name) + "\":" + std::to_string(state.name) +   \
            ",");
    READ_WRITE_FIELDS
    READ_ONLY_FIELDS
#undef FIELD
    // Remove trailing comma if present
    if (!res.empty() && res.back() == ',') {
        res.pop_back();
    }
    res += "}";
    return res;
}

struct TripleBuffer {
    Telemetry buffers[3] = {};

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

    Telemetry telemetry = {};
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
        std::cout << "Writing to telemetry buffer" << std::endl;
        telemetry_buffer.write(telemetry);

        usleep(1000);
    }
}

int main() {

    TripleBuffer telemetry_buffer;

    // Server 1: Command/Control Server
    websocketpp::server<websocketpp::config::asio> cmd_server;
    std::set<websocketpp::connection_hdl,
             std::owner_less<websocketpp::connection_hdl>>
        cmd_connections;

    // Server 2: Telemetry Monitor Server
    websocketpp::server<websocketpp::config::asio> monitor_server;
    std::set<websocketpp::connection_hdl,
             std::owner_less<websocketpp::connection_hdl>>
        monitor_connections;

    // Initialize both servers
    cmd_server.init_asio();
    monitor_server.init_asio();

    // Set socket reuse options to avoid "Address already in use" errors
    cmd_server.set_reuse_addr(true);
    monitor_server.set_reuse_addr(true);

    // Disable verbose WebSocket++ logging
    cmd_server.clear_access_channels(websocketpp::log::alevel::all);
    cmd_server.clear_error_channels(websocketpp::log::elevel::all);
    monitor_server.clear_access_channels(websocketpp::log::alevel::all);
    monitor_server.clear_error_channels(websocketpp::log::elevel::all);

    // Or keep only important error messages
    // cmd_server.set_error_channels(websocketpp::log::elevel::warn |
    // websocketpp::log::elevel::rerror | websocketpp::log::elevel::fatal);
    // monitor_server.set_error_channels(websocketpp::log::elevel::warn |
    // websocketpp::log::elevel::rerror | websocketpp::log::elevel::fatal);

    // === COMMAND SERVER HANDLERS ===
    cmd_server.set_open_handler(
        [&cmd_connections](websocketpp::connection_hdl hdl) {
            cmd_connections.insert(hdl);
            std::cout << "New command connection established" << std::endl;
        });

    cmd_server.set_close_handler(
        [&cmd_connections](websocketpp::connection_hdl hdl) {
            cmd_connections.erase(hdl);
            std::cout << "Command connection closed" << std::endl;
        });

    cmd_server.set_message_handler([&](auto hdl, auto msg) {
        std::cout << "Command received: " << msg->get_payload() << std::endl;
        // Handle commands here - send to rover, etc.
    });

    // === MONITOR SERVER HANDLERS ===
    monitor_server.set_open_handler(
        [&monitor_connections](websocketpp::connection_hdl hdl) {
            monitor_connections.insert(hdl);
            std::cout << "New monitor connection established" << std::endl;
        });

    monitor_server.set_close_handler(
        [&monitor_connections](websocketpp::connection_hdl hdl) {
            monitor_connections.erase(hdl);
            std::cout << "Monitor connection closed" << std::endl;
        });

    monitor_server.set_message_handler([&](auto hdl, auto msg) {
        std::cout << "Monitor message: " << msg->get_payload() << std::endl;
    });

    // Start telemetry listening thread
    std::thread telemetry_thread(
        [&telemetry_buffer]() { listenToTelemetry(telemetry_buffer); });
    telemetry_thread.detach();

    // Start periodic telemetry broadcast thread (only to monitor clients)
    std::thread broadcast_thread([&monitor_server, &monitor_connections,
                                  &telemetry_buffer]() {
        Telemetry current_telemetry{};
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            telemetry_buffer.read(current_telemetry);
            std::string json_data = get_json_of_state(current_telemetry.state);

            // Send to all connected monitor clients
            for (auto &hdl : monitor_connections) {
                try {
                    monitor_server.send(hdl, json_data,
                                        websocketpp::frame::opcode::text);
                } catch (const std::exception &e) {
                    std::cerr << "Error sending to monitor client: " << e.what()
                              << std::endl;
                }
            }
        }
    });
    broadcast_thread.detach();

    // Start both servers in separate threads
    std::thread cmd_server_thread([&cmd_server]() {
        try {
            cmd_server.listen(
                CMD_SERVER_PORT); // Command server on port CMD_SERVER_PORT
            cmd_server.start_accept();
            std::cout
                << "Command server running on ws://localhost:CMD_SERVER_PORT"
                << std::endl;
            cmd_server.run();
        } catch (const websocketpp::exception &e) {
            std::cerr << "Command server error: " << e.what() << std::endl;
            std::cerr << "Try waiting a few seconds before restarting, or use: "
                         "sudo lsof -ti:CMD_SERVER_PORT | xargs kill -9"
                      << std::endl;
        }
    });

    std::thread monitor_server_thread([&monitor_server]() {
        try {
            monitor_server.listen(
                MONITOR_SERVER_PORT); // Monitor server on port
                                      // MONITOR_SERVER_PORT
            monitor_server.start_accept();
            std::cout << "Monitor server running on "
                         "ws://localhost:MONITOR_SERVER_PORT"
                      << std::endl;
            monitor_server.run();
        } catch (const websocketpp::exception &e) {
            std::cerr << "Monitor server error: " << e.what() << std::endl;
            std::cerr << "Try waiting a few seconds before restarting, or use: "
                         "sudo lsof -ti:MONITOR_SERVER_PORT | xargs kill -9"
                      << std::endl;
        }
    });

    std::cout << "Both WebSocket servers started:" << std::endl;
    std::cout << "- Command server: ws://localhost:CMD_SERVER_PORT"
              << std::endl;
    std::cout << "- Monitor server: ws://localhost:MONITOR_SERVER_PORT"
              << std::endl;
    std::cout << "Telemetry will be broadcast every 1 second to monitor clients"
              << std::endl;

    // Wait for both server threads
    cmd_server_thread.join();
    monitor_server_thread.join();

    return 0;
}