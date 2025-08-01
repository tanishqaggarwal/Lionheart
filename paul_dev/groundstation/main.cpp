#include <iostream>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

int main() {
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

    return 0;
}