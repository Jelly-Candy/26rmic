#include "RuntimeLogServer.hpp"
#include "RuntimeLogBuffer.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <cstdint>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

namespace {

std::string jsonEscape(const std::string &s) {
    std::string out;
    out.reserve(s.size() + 8);

    for (unsigned char c : s) {
        switch (c) {
        case '"':  out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20) {
                char buf[7];
                std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                out += buf;
            } else {
                out += static_cast<char>(c);
            }
        }
    }

    return out;
}

bool sendAll(int fd, const std::string &data) {
    std::size_t sent = 0;

    while (sent < data.size()) {
        const ssize_t n = ::send(
            fd, data.data() + sent, data.size() - sent, 0);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }

        if (n == 0) {
            return false;
        }

        sent += static_cast<std::size_t>(n);
    }

    return true;
}

std::string buildJson() {
    const auto entries = runtimeLogBuffer().snapshot();

    std::string body = "{\"logs\":[";
    bool first = true;

    for (const auto &entry : entries) {
        if (!first) {
            body += ',';
        }
        first = false;

        body += "{\"time_ms\":";
        body += std::to_string(entry.timestampMs);
        body += ",\"level\":\"";
        body += jsonEscape(entry.level);
        body += "\",\"message\":\"";
        body += jsonEscape(entry.message);
        body += "\"}";
    }

    body += "]}";
    return body;
}

} // namespace

RuntimeLogServer::RuntimeLogServer(int port) : port_(port) {}

RuntimeLogServer::~RuntimeLogServer() {
    stop();
}

bool RuntimeLogServer::start() {
    if (running_) {
        return true;
    }

    listenFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) {
        return false;
    }

    int reuse = 1;
    ::setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR,
                 &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(listenFd_, reinterpret_cast<sockaddr *>(&addr),
               sizeof(addr)) < 0 ||
        ::listen(listenFd_, 4) < 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return false;
    }

    running_ = true;
    worker_ = std::thread(&RuntimeLogServer::run, this);
    return true;
}

void RuntimeLogServer::stop() {
    if (!running_.exchange(false)) {
        return;
    }

    if (listenFd_ >= 0) {
        ::shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
        listenFd_ = -1;
    }

    if (worker_.joinable()) {
        worker_.join();
    }
}

void RuntimeLogServer::run() {
    while (running_) {
        sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);

        const int client = ::accept(
            listenFd_,
            reinterpret_cast<sockaddr *>(&clientAddr),
            &clientLen);

        if (client < 0) {
            if (!running_) {
                break;
            }
            if (errno == EINTR) {
                continue;
            }
            continue;
        }

        // 只需要读取请求行；限制读取长度，避免无限等待大请求。
        char request[1024]{};
        const ssize_t n = ::recv(client, request, sizeof(request) - 1, 0);

        if (n > 0 && std::strncmp(request, "GET /logs ", 10) == 0) {
            const std::string body = buildJson();

            std::string response =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: application/json; charset=utf-8\r\n"
                "Cache-Control: no-store\r\n"
                "Access-Control-Allow-Origin: *\r\n"
                "Connection: close\r\n"
                "Content-Length: " + std::to_string(body.size()) +
                "\r\n\r\n" + body;

            sendAll(client, response);
        } else {
            const std::string body = "{\"error\":\"not found\"}";
            const std::string response =
                "HTTP/1.1 404 Not Found\r\n"
                "Content-Type: application/json\r\n"
                "Connection: close\r\n"
                "Content-Length: " + std::to_string(body.size()) +
                "\r\n\r\n" + body;

            sendAll(client, response);
        }

        ::close(client);
    }
}