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

    // 网页提交的阵营：0 = 无，1 = RED，2 = BLUE
    // 主线程调用此函数读取并消费网页选择。
    bool consumeEnemyColor(int &color);

private:
    void run();

    int port_;
    int listenFd_ = -1;
    std::atomic<bool> running_{false};
    std::atomic<int> pendingEnemyColor_{0};
    std::thread worker_;
};