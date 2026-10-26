#pragma once

#include <cstddef>
#include <cstdint>

#include "AimResult.hpp"

class AimUart
{
public:
    static constexpr std::size_t FRAME_SIZE = 48;
    static constexpr uint8_t VERSION = 0x02;

    static void encode(
        const AimResult& result,
        uint8_t seq,
        uint8_t frame[FRAME_SIZE]);

    static uint16_t crc16(const uint8_t* data, std::size_t len);
};
