#include "lidar_demo/lidar_demo.hpp"

#include <array>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle kInvalidSocket = INVALID_SOCKET;
#else
using SocketHandle = int;
constexpr SocketHandle kInvalidSocket = -1;
#endif

void close_socket(SocketHandle handle) {
#ifdef _WIN32
    if (handle != INVALID_SOCKET) {
        closesocket(handle);
    }
#else
    if (handle >= 0) {
        close(handle);
    }
#endif
}

struct SocketGuard {
    SocketHandle handle = kInvalidSocket;

    SocketGuard() = default;
    explicit SocketGuard(SocketHandle input) : handle(input) {}
    ~SocketGuard() {
        close_socket(handle);
    }

    SocketGuard(const SocketGuard&) = delete;
    SocketGuard& operator=(const SocketGuard&) = delete;

    SocketGuard(SocketGuard&& other) noexcept : handle(other.handle) {
        other.handle = kInvalidSocket;
    }

    SocketGuard& operator=(SocketGuard&& other) noexcept {
        if (this != &other) {
            close_socket(handle);
            handle = other.handle;
            other.handle = kInvalidSocket;
        }
        return *this;
    }

    SocketHandle release() {
        SocketHandle value = handle;
        handle = kInvalidSocket;
        return value;
    }
};

#ifdef _WIN32
struct WinsockSession {
    WSADATA data{};

    WinsockSession() {
        if (WSAStartup(MAKEWORD(2, 2), &data) != 0) {
            throw std::runtime_error("Failed to initialize Winsock");
        }
    }

    ~WinsockSession() {
        WSACleanup();
    }
};
#endif

std::string socket_error_prefix(const std::string& message) {
#ifdef _WIN32
    return message + " (winsock error " + std::to_string(WSAGetLastError()) + ")";
#else
    return message + ": " + std::strerror(errno);
#endif
}

lidar_demo::Json build_summary(const std::filesystem::path& config_path) {
    lidar_demo::PipelineConfig config = lidar_demo::parse_pipeline_config(lidar_demo::read_json_file(config_path));
    return lidar_demo::build_summary_payload(lidar_demo::run_end_to_end(config));
}

std::string build_response(
    int status_code,
    const std::string& status_text,
    const std::string& content_type,
    const std::string& body
) {
    std::ostringstream response;
    response << "HTTP/1.1 " << status_code << ' ' << status_text << "\r\n";
    response << "Content-Type: " << content_type << "\r\n";
    response << "Content-Length: " << body.size() << "\r\n";
    response << "Connection: close\r\n";
    response << "Cache-Control: no-store\r\n\r\n";
    response << body;
    return response.str();
}

void send_all(SocketHandle handle, const std::string& payload) {
    std::size_t sent = 0;
    while (sent < payload.size()) {
#ifdef _WIN32
        int count = send(handle, payload.data() + sent, static_cast<int>(payload.size() - sent), 0);
#else
        ssize_t count = send(handle, payload.data() + sent, payload.size() - sent, 0);
#endif
        if (count <= 0) {
            throw std::runtime_error(socket_error_prefix("Failed to send HTTP response"));
        }
        sent += static_cast<std::size_t>(count);
    }
}

std::string receive_request(SocketHandle handle) {
    std::string request;
    std::array<char, 4096> buffer{};
    while (request.find("\r\n\r\n") == std::string::npos) {
#ifdef _WIN32
        int count = recv(handle, buffer.data(), static_cast<int>(buffer.size()), 0);
#else
        ssize_t count = recv(handle, buffer.data(), buffer.size(), 0);
#endif
        if (count < 0) {
            throw std::runtime_error(socket_error_prefix("Failed to read HTTP request"));
        }
        if (count == 0) {
            break;
        }
        request.append(buffer.data(), static_cast<std::size_t>(count));
        if (request.size() > 16384) {
            break;
        }
    }
    return request;
}

std::string route_request(const std::string& request, const std::filesystem::path& config_path) {
    std::istringstream stream(request);
    std::string method;
    std::string target;
    std::string version;
    stream >> method >> target >> version;

    if (method.empty() || target.empty()) {
        return build_response(400, "Bad Request", "text/plain; charset=utf-8", "Bad Request");
    }
    if (method != "GET") {
        return build_response(405, "Method Not Allowed", "text/plain; charset=utf-8", "Method Not Allowed");
    }

    std::size_t query_index = target.find('?');
    if (query_index != std::string::npos) {
        target = target.substr(0, query_index);
    }

    if (target != "/api/summary" && target != "/api/hotspots") {
        return build_response(404, "Not Found", "text/plain; charset=utf-8", "Not Found");
    }

    try {
        lidar_demo::Json summary = build_summary(config_path);
        const lidar_demo::Json& payload = target == "/api/summary" ? summary : summary.at("latest_hotspots");
        return build_response(200, "OK", "application/json; charset=utf-8", lidar_demo::dump_json(payload, 2));
    } catch (const std::exception& error) {
        lidar_demo::Json body = lidar_demo::Json::object_type{{"error", error.what()}};
        return build_response(500, "Internal Server Error", "application/json; charset=utf-8", lidar_demo::dump_json(body, 2));
    }
}

SocketGuard create_listen_socket(const std::string& host, int port) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    addrinfo* result = nullptr;
    std::string port_text = std::to_string(port);
    const char* node = host.empty() ? nullptr : host.c_str();
    int status = getaddrinfo(node, port_text.c_str(), &hints, &result);
    if (status != 0) {
#ifdef _WIN32
        throw std::runtime_error("getaddrinfo failed for host binding");
#else
        throw std::runtime_error(std::string("getaddrinfo failed: ") + gai_strerror(status));
#endif
    }

    SocketGuard listen_socket;
    for (addrinfo* current = result; current != nullptr; current = current->ai_next) {
        SocketHandle handle = socket(current->ai_family, current->ai_socktype, current->ai_protocol);
        if (handle == kInvalidSocket) {
            continue;
        }

        int reuse = 1;
        setsockopt(handle, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&reuse), sizeof(reuse));
        if (bind(handle, current->ai_addr, static_cast<int>(current->ai_addrlen)) == 0 && listen(handle, SOMAXCONN) == 0) {
            listen_socket = SocketGuard(handle);
            break;
        }
        close_socket(handle);
    }
    freeaddrinfo(result);

    if (listen_socket.handle == kInvalidSocket) {
        throw std::runtime_error(socket_error_prefix("Failed to bind HTTP server socket"));
    }
    return listen_socket;
}

void serve_forever(const std::string& host, int port, const std::filesystem::path& config_path) {
    SocketGuard listen_socket = create_listen_socket(host, port);
    std::cout << "Serving on http://" << host << ':' << port << "/api/summary" << std::endl;

    for (;;) {
        SocketHandle client_handle = accept(listen_socket.handle, nullptr, nullptr);
        if (client_handle == kInvalidSocket) {
            throw std::runtime_error(socket_error_prefix("Failed to accept HTTP connection"));
        }
        SocketGuard client(client_handle);
        std::string request = receive_request(client.handle);
        if (request.empty()) {
            continue;
        }
        std::string response = route_request(request, config_path);
        send_all(client.handle, response);
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        std::filesystem::path config_path = "configs/default_pipeline.json";
        std::string host = "127.0.0.1";
        int port = 8765;
        bool once = false;
        for (int index = 1; index < argc; ++index) {
            std::string argument = argv[index];
            if (argument == "--config" && index + 1 < argc) {
                config_path = argv[++index];
            } else if (argument == "--host" && index + 1 < argc) {
                host = argv[++index];
            } else if (argument == "--port" && index + 1 < argc) {
                port = std::stoi(argv[++index]);
            } else if (argument == "--once") {
                once = true;
            }
        }

        if (once) {
            lidar_demo::Json summary = build_summary(config_path);
            std::cout << lidar_demo::dump_json(summary, 2) << std::endl;
            return 0;
        }

#ifdef _WIN32
        WinsockSession winsock;
#endif
        serve_forever(host, port, config_path);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}