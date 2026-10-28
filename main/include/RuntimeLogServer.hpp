#pragma once

#include <atomic>
#include <thread>

class RuntimeLogServer {
public:
    explicit RuntimeLogServer(int port = 8001);
    ~RuntimeLogServer();

    RuntimeLogServer(const RuntimeLogServer &) = delete;
    RuntimeLogServer &operator=(const RuntimeLogServer &) = delete;

    bool start();
    void stop();

private:
    void run();

    int port_;
    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::thread worker_;
};