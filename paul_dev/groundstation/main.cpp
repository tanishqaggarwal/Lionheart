#include "telemetry_processor.h"
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <errno.h>
#include <functional>
#include <iostream>
#include <netinet/in.h>
#include <set>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

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

struct CommandSender {
    char buffer[64] = {};
    int socketfd; // Use single socket for both send and receive
    struct sockaddr_in dest_addr;
    bool initialized = false;
    std::atomic<bool> running{true};

    // Add callback for Arduino responses
    std::function<void(const std::string &)> response_callback;

    CommandSender() {
        // Create UDP socket for both sending and receiving
        socketfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socketfd < 0) {
            std::cerr << "Error creating command socket" << std::endl;
            return;
        }

        // Set up destination address (192.168.1.177:MICRO_CMD_PORT)
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(MICRO_CMD_PORT);
        inet_pton(AF_INET, "192.168.1.177", &dest_addr.sin_addr);

        // Bind socket to a specific source port (SERVER_CMD_PORT)
        struct sockaddr_in local_addr;
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(SERVER_CMD_PORT);

        if (bind(socketfd, (struct sockaddr *)&local_addr, sizeof(local_addr)) <
            0) {
            std::cerr << "Error binding socket to port SERVER_CMD_PORT: "
                      << strerror(errno) << std::endl;
            close(socketfd);
            return;
        }

        initialized = true;
        std::cout << "CommandSender initialized:" << std::endl;
        std::cout << "  - Sending from: 0.0.0.0:SERVER_CMD_PORT to "
                     "192.168.1.177:MICRO_CMD_PORT"
                  << std::endl;
        std::cout << "  - Listening on: 0.0.0.0:SERVER_CMD_PORT for responses"
                  << std::endl;
    }

    ~CommandSender() {
        running = false;
        if (socketfd >= 0) {
            close(socketfd);
        }
    }

    void sendBuffer() {
        if (!initialized) {
            std::cerr << "CommandSender not initialized" << std::endl;
            return;
        }

        int bytes_sent =
            sendto(socketfd, buffer, strlen(buffer), 0,
                   (struct sockaddr *)&dest_addr, sizeof(dest_addr));

        if (bytes_sent < 0) {
            std::cerr << "Error sending command: " << strerror(errno)
                      << std::endl;
        } else {
            std::cout << "Sent " << bytes_sent
                      << " bytes to 192.168.1.177:MICRO_CMD_PORT: " << buffer
                      << std::endl;
        }
    }

    void
    setResponseCallback(std::function<void(const std::string &)> callback) {
        response_callback = callback;
    }

    void setBuffer(const char *data) {
        strncpy(buffer, data, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0'; // Ensure null termination
    }

    void startListening() {
        if (!initialized) {
            std::cerr << "CommandSender not initialized, cannot start listening"
                      << std::endl;
            return;
        }

        std::thread listen_thread([this]() {
            char response_buffer[256];
            struct sockaddr_in sender_addr;
            socklen_t sender_len = sizeof(sender_addr);

            std::cout << "CommandSender listening thread started" << std::endl;

            while (running) {
                int bytes_received = recvfrom(
                    socketfd, response_buffer, sizeof(response_buffer) - 1, 0,
                    (struct sockaddr *)&sender_addr, &sender_len);

                if (bytes_received > 0) {
                    response_buffer[bytes_received] = '\0'; // Null terminate

                    char sender_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip,
                              INET_ADDRSTRLEN);

                    std::string response_msg = std::string(response_buffer);
                    std::cout << "Arduino response from " << sender_ip << ":"
                              << ntohs(sender_addr.sin_port) << " - "
                              << response_msg << std::endl;

                    // Call the callback to send response to WebSocket clients
                    if (response_callback) {
                        response_callback("Arduino response: " + response_msg);
                    }
                } else if (bytes_received < 0 && running) {
                    std::cerr << "Error receiving command response: "
                              << strerror(errno) << std::endl;
                }

                usleep(1000); // Small delay to prevent busy waiting
            }

            std::cout << "CommandSender listening thread stopped" << std::endl;
        });

        listen_thread.detach();
    }
};

void listenToTelemetry(TripleBuffer &telemetry_buffer) {

    int serverSocket = socket(AF_INET, SOCK_DGRAM, 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(SERVER_TELM_PORT);

    bind(serverSocket, (struct sockaddr *)&addr, sizeof(addr));

    TelemetryProcessor telemetry_processor;
    char recv_buffer[sizeof(Telemetry)] = {};
    while (1) {
        int bytes_read =
            recv(serverSocket, recv_buffer, sizeof(recv_buffer), 0);
        for (int i = 0; i < bytes_read; ++i) {
            if (telemetry_processor.processChar(recv_buffer[i])) {
                telemetry_buffer.write(*(
                    reinterpret_cast<Telemetry *>(telemetry_processor.buffer)));
            }
        }
        usleep(20000);
    }
}

int main() {

    TripleBuffer telemetry_buffer;
    CommandSender command_sender;

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
        std::string command = msg->get_payload();
        std::cout << "Command received: " << command << std::endl;

        // Send command to 192.168.1.177:MICRO_CMD_PORT
        command_sender.setBuffer(command.c_str());
        command_sender.sendBuffer();

        // Send confirmation back to WebSocket client
        cmd_server.send(hdl, "Command sent to rover: " + command,
                        websocketpp::frame::opcode::text);
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

    // Start command sender listening thread
    command_sender.startListening();

    // Set callback to broadcast Arduino responses to command WebSocket clients
    command_sender.setResponseCallback(
        [&cmd_server, &cmd_connections](const std::string &response) {
            // Send Arduino response to all connected command clients
            for (auto &hdl : cmd_connections) {
                try {
                    cmd_server.send(hdl, response,
                                    websocketpp::frame::opcode::text);
                } catch (const std::exception &e) {
                    std::cerr
                        << "Error sending Arduino response to command client: "
                        << e.what() << std::endl;
                }
            }
        });

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
                CMD_UI_PORT); // Command server on port CMD_UI_PORT
            cmd_server.start_accept();
            std::cout << "Command server running on ws://localhost:CMD_UI_PORT"
                      << std::endl;
            cmd_server.run();
        } catch (const websocketpp::exception &e) {
            std::cerr << "Command server error: " << e.what() << std::endl;
            std::cerr << "Try waiting a few seconds before restarting, or use: "
                         "sudo lsof -ti:CMD_UI_PORT | xargs kill -9"
                      << std::endl;
        }
    });

    std::thread monitor_server_thread([&monitor_server]() {
        try {
            monitor_server.listen(MONITOR_UI_PORT); // Monitor server on port
                                                    // MONITOR_UI_PORT
            monitor_server.start_accept();
            std::cout << "Monitor server running on "
                         "ws://localhost:MONITOR_UI_PORT"
                      << std::endl;
            monitor_server.run();
        } catch (const websocketpp::exception &e) {
            std::cerr << "Monitor server error: " << e.what() << std::endl;
            std::cerr << "Try waiting a few seconds before restarting, or use: "
                         "sudo lsof -ti:MONITOR_UI_PORT | xargs kill -9"
                      << std::endl;
        }
    });

    std::cout << "Both WebSocket servers started:" << std::endl;
    std::cout << "- Command server: ws://localhost:CMD_UI_PORT" << std::endl;
    std::cout << "- Monitor server: ws://localhost:MONITOR_UI_PORT"
              << std::endl;
    std::cout << "Telemetry will be broadcast every 1 second to monitor clients"
              << std::endl;

    // Wait for both server threads
    cmd_server_thread.join();
    monitor_server_thread.join();

    return 0;
}