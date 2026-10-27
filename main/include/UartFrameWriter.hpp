#pragma once
#include <cstddef>
#include <cstdint>

// Continue from the remaining bytes after a short write; never interleave frames.
template <typename Port>
bool writeCompleteFrame(Port& port, const uint8_t* data, std::size_t size)
{
    std::size_t offset = 0;
    while (offset < size) {
        int n = port.write(data + offset, static_cast<int>(size - offset));
        if (n <= 0 || static_cast<std::size_t>(n) > size - offset)
            return false;
        offset += static_cast<std::size_t>(n);
    }
    return true;
}
