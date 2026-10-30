#include "RuntimeLogServer.hpp"
#include "RuntimeLogBuffer.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <cstdint>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <unistd.h>

namespace {

constexpr std::size_t MAX_HEADER_SIZE = 4096;
constexpr std::size_t MAX_BODY_SIZE = 32;

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

std::string makeResponse(
    int status,
    const char *reason,
    const std::string &body) {

    return "HTTP/1.1 " + std::to_string(status) + " " + reason + "\r\n"
           "Content-Type: application/json; charset=utf-8\r\n"
           "Cache-Control: no-store\r\n"
           "Access-Control-Allow-Origin: *\r\n"
           "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
           "Access-Control-Allow-Headers: Content-Type\r\n"
           "Connection: close\r\n"
           "Content-Length: " + std::to_string(body.size()) +
           "\r\n\r\n" + body;
}

std::string trim(const std::string &value) {
    std::size_t begin = 0;
    std::size_t end = value.size();

    while (begin < end &&
           std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }

    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }

    return value.substr(begin, end - begin);
}

bool parseContentLength(
    const std::string &headers,
    std::size_t &contentLength) {

    const std::size_t lineEnd = headers.find("\r\n");
    if (lineEnd == std::string::npos) {
        return false;
    }

    std::size_t pos = lineEnd + 2;

    while (pos < headers.size()) {
        const std::size_t end = headers.find("\r\n", pos);
        if (end == std::string::npos || end == pos) {
            break;
        }

        const std::string line = headers.substr(pos, end - pos);
        const std::size_t colon = line.find(':');

        if (colon != std::string::npos) {
            std::string name = line.substr(0, colon);

            std::transform(
                name.begin(), name.end(), name.begin(),
                [](unsigned char c) {
                    return static_cast<char>(std::tolower(c));
                });

            if (name == "content-length") {
                const std::string value = trim(line.substr(colon + 1));

                try {
                    std::size_t parsed = 0;
                    contentLength = std::stoul(value, &parsed);

                    return parsed == value.size() &&
                           contentLength <= MAX_BODY_SIZE;
                } catch (...) {
                    return false;
                }
            }
        }

        pos = end + 2;
    }

    contentLength = 0;
    return true;
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
    ::setsockopt(
        listenFd_, SOL_SOCKET, SO_REUSEADDR,
        &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port_));

    if (::bind(
            listenFd_,
            reinterpret_cast<sockaddr *>(&addr),
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

bool RuntimeLogServer::consumeEnemyColor(int &color) {
    const int value = pendingEnemyColor_.exchange(0);

    if (value == 0) {
        return false;
    }

    color = value;
    return true;
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

        // 防止客户端连接后一直不发送数据。
        timeval timeout{};
        timeout.tv_sec = 2;
        timeout.tv_usec = 0;
        ::setsockopt(
            client, SOL_SOCKET, SO_RCVTIMEO,
            &timeout, sizeof(timeout));

        std::string request;
        request.reserve(1024);

        char buffer[1024];
        std::size_t headerEnd = std::string::npos;
        std::size_t contentLength = 0;
        bool requestValid = true;

        // 先读取完整 HTTP 请求头。
        while (request.size() < MAX_HEADER_SIZE) {
            const ssize_t n = ::recv(
                client, buffer, sizeof(buffer), 0);

            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                requestValid = false;
                break;
            }

            if (n == 0) {
                requestValid = false;
                break;
            }

            request.append(buffer, static_cast<std::size_t>(n));

            headerEnd = request.find("\r\n\r\n");
            if (headerEnd != std::string::npos) {
                break;
            }
        }

        if (headerEnd == std::string::npos) {
            requestValid = false;
        }

        std::string requestLine;
        std::string body;

        if (requestValid) {
            const std::string headers =
                request.substr(0, headerEnd);

            const std::size_t lineEnd = headers.find("\r\n");
            if (lineEnd == std::string::npos) {
                requestValid = false;
            } else {
                requestLine = headers.substr(0, lineEnd);
            }

            if (requestValid &&
                !parseContentLength(headers, contentLength)) {
                requestValid = false;
            }

            const std::size_t bodyStart = headerEnd + 4;

            if (requestValid) {
                body = request.substr(bodyStart);

                // 继续接收剩余请求正文。
                while (body.size() < contentLength) {
                    const std::size_t remaining =
                        contentLength - body.size();

                    const ssize_t n = ::recv(
                        client, buffer,
                        std::min(remaining, sizeof(buffer)), 0);

                    if (n < 0) {
                        if (errno == EINTR) {
                            continue;
                        }
                        requestValid = false;
                        break;
                    }

                    if (n == 0) {
                        requestValid = false;
                        break;
                    }

                    body.append(buffer, static_cast<std::size_t>(n));
                }

                if (body.size() > contentLength) {
                    body.resize(contentLength);
                }
            }
        }

        std::string response;

        if (!requestValid) {
            response = makeResponse(
                400, "Bad Request",
                "{\"ok\":false,\"error\":\"invalid request\"}");

        } else if (requestLine.rfind("OPTIONS ", 0) == 0) {
            response = makeResponse(204, "No Content", "");

        } else if (requestLine.rfind("GET /logs ", 0) == 0) {
            response = makeResponse(200, "OK", buildJson());

        } else if (requestLine.rfind("POST /api/config ", 0) == 0) {
            const std::string color = trim(body);

            if (color == "RED") {
                pendingEnemyColor_.store(1);

                response = makeResponse(
                    200, "OK",
                    "{\"ok\":true,\"enemy_color\":\"RED\"}");

            } else if (color == "BLUE") {
                pendingEnemyColor_.store(2);

                response = makeResponse(
                    200, "OK",
                    "{\"ok\":true,\"enemy_color\":\"BLUE\"}");

            } else {
                response = makeResponse(
                    400, "Bad Request",
                    "{\"ok\":false,\"error\":\"color must be RED or BLUE\"}");
            }

        } else {
            response = makeResponse(
                404, "Not Found",
                "{\"ok\":false,\"error\":\"not found\"}");
        }

        sendAll(client, response);
        ::close(client);
    }
}