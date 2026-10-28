#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

struct RuntimeLogEntry {
    uint64_t timestampMs = 0;
    std::string level;
    std::string message;
};

class RuntimeLogBuffer {
public:
    static constexpr std::size_t MAX_ENTRIES = 200;

    void push(const std::string &level, const std::string &message);
    std::vector<RuntimeLogEntry> snapshot() const;

private:
    mutable std::mutex mutex_;
    std::deque<RuntimeLogEntry> entries_;
};

// 记录日志，同时保留原来的终端输出。
void runtimeLog(const char *level, const char *format, ...);

RuntimeLogBuffer &runtimeLogBuffer();