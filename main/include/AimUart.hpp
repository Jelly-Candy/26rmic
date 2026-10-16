#pragma once

#include <cstdint>
#include <cstddef>
#include "AimResult.hpp"

class AimUart {
public:
    static constexpr std::size_t FRAME_SIZE = 14;

    // 将瞄准结果编码为 14 字节数据帧
    static void encode(
        const AimResult& result,
        uint8_t seq,
        uint8_t frame[FRAME_SIZE]
    );

    // CRC-16/IBM，初值 0x0000，多项式反射形式 0xA001
    static uint16_t crc16(const uint8_t* data, std::size_t len);
};