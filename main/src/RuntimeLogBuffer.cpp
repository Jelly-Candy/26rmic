#include "RuntimeLogBuffer.hpp"

#include "maix_time.hpp"

#include <cstdarg>
#include <cstdio>
#include <utility>
#include <vector>

RuntimeLogBuffer &runtimeLogBuffer() {
    static RuntimeLogBuffer instance;
    return instance;
}

void RuntimeLogBuffer::push(const std::string &level,
                            const std::string &message) {
    RuntimeLogEntry entry;
    entry.timestampMs = static_cast<uint64_t>(maix::time::ticks_ms());
    entry.level = level;
    entry.message = message;

    std::lock_guard<std::mutex> lock(mutex_);

    if (entries_.size() >= MAX_ENTRIES) {
        entries_.pop_front();
    }

    entries_.push_back(std::move(entry));
}

std::vector<RuntimeLogEntry> RuntimeLogBuffer::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<RuntimeLogEntry>(entries_.begin(), entries_.end());
}

void runtimeLog(const char *level, const char *format, ...) {
    char message[512];

    va_list args;
    va_start(args, format);
    std::vsnprintf(message, sizeof(message), format, args);
    va_end(args);

    // 继续输出到设备终端。
    std::printf("[%s] %s\n", level, message);

    // 同时写入网页日志缓冲区。
    runtimeLogBuffer().push(level, message);
}