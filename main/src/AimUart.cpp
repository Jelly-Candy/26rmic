#include "AimUart.hpp"

#include <cmath>
#include <cstdint>

uint16_t AimUart::crc16(const uint8_t* data, std::size_t len)
{
    uint16_t crc = 0x0000;

    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];

        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void AimUart::encode(
    const AimResult& result,
    uint8_t seq,
    uint8_t frame[FRAME_SIZE])
{
    for (std::size_t i = 0; i < FRAME_SIZE; ++i) {
        frame[i] = 0;
    }

    frame[0] = 0xA5;
    frame[1] = 0x5A;
    frame[2] = seq;
    frame[3] = result.targetFound ? 0x01 : 0x00;

    int32_t yawMilliDeg = 0;
    int32_t pitchMilliDeg = 0;

    if (result.targetFound) {
        const float yaw = result.yawDeg;
        const float pitch = result.pitchDeg;

        // 防止 NaN、无穷值及异常角度进入通信帧
        if (std::isfinite(yaw) && std::isfinite(pitch)
            && std::abs(yaw) <= 180.0f
            && std::abs(pitch) <= 90.0f) {
            yawMilliDeg =
                static_cast<int32_t>(std::lround(yaw * 1000.0f));
            pitchMilliDeg =
                static_cast<int32_t>(std::lround(pitch * 1000.0f));
        } else {
            frame[3] = 0x00;
        }
    }

    const auto putInt32LE = [&frame](
        std::size_t offset, int32_t value) {
        const uint32_t u = static_cast<uint32_t>(value);
        frame[offset + 0] = static_cast<uint8_t>(u);
        frame[offset + 1] = static_cast<uint8_t>(u >> 8);
        frame[offset + 2] = static_cast<uint8_t>(u >> 16);
        frame[offset + 3] = static_cast<uint8_t>(u >> 24);
    };

    putInt32LE(4, yawMilliDeg);
    putInt32LE(8, pitchMilliDeg);

    const uint16_t crc = crc16(frame, 12);
    frame[12] = static_cast<uint8_t>(crc & 0xFF);
    frame[13] = static_cast<uint8_t>(crc >> 8);
}