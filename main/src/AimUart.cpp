#include "AimUart.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

uint16_t AimUart::crc16(const uint8_t* data, std::size_t len)
{
    uint16_t crc = 0x0000;

    for (std::size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];

        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1)
                ? static_cast<uint16_t>((crc >> 1) ^ 0xA001)
                : static_cast<uint16_t>(crc >> 1);
        }
    }

    return crc;
}

namespace
{
void putI16LE(uint8_t* frame, std::size_t offset, int16_t value)
{
    const uint16_t u = static_cast<uint16_t>(value);
    frame[offset] = static_cast<uint8_t>(u);
    frame[offset + 1] = static_cast<uint8_t>(u >> 8);
}

void putU16LE(uint8_t* frame, std::size_t offset, uint16_t value)
{
    frame[offset] = static_cast<uint8_t>(value);
    frame[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void putI32LE(uint8_t* frame, std::size_t offset, int32_t value)
{
    const uint32_t u = static_cast<uint32_t>(value);
    frame[offset] = static_cast<uint8_t>(u);
    frame[offset + 1] = static_cast<uint8_t>(u >> 8);
    frame[offset + 2] = static_cast<uint8_t>(u >> 16);
    frame[offset + 3] = static_cast<uint8_t>(u >> 24);
}

int32_t scaleI32(float value, float scale, float limit)
{
    if (!std::isfinite(value))
        return 0;

    value = std::max(-limit, std::min(limit, value));
    return static_cast<int32_t>(std::lround(value * scale));
}

int16_t scaleI16(float value, float scale, float limit)
{
    if (!std::isfinite(value))
        return 0;

    value = std::max(-limit, std::min(limit, value));
    return static_cast<int16_t>(std::lround(value * scale));
}
}

void AimUart::encode(
    const AimResult& result,
    uint8_t seq,
    uint8_t frame[FRAME_SIZE])
{
    for (std::size_t i = 0; i < FRAME_SIZE; ++i)
        frame[i] = 0;

    frame[0] = 0xA5;
    frame[1] = 0x5A;
    frame[2] = VERSION;
    frame[3] = seq;

    uint16_t flags = 0;

    const bool targetValid =
        result.targetFound &&
        std::isfinite(result.yawDeg) &&
        std::isfinite(result.pitchDeg) &&
        std::abs(result.yawDeg) <= 180.0f &&
        std::abs(result.pitchDeg) <= 90.0f;

    if (targetValid)
        flags |= (1u << 0);

    const ImuTelemetry& imu = result.imu;

    if (imu.valid)
        flags |= (1u << 1);

    if (imu.valid && imu.attitudeValid)
        flags |= (1u << 2);

    if (imu.dtOutOfRange)
        flags |= (1u << 3);

    putU16LE(frame, 4, flags);

    if (targetValid)
    {
        putI32LE(frame, 6, scaleI32(result.yawDeg, 1000.0f, 180.0f));
        putI32LE(frame, 10, scaleI32(result.pitchDeg, 1000.0f, 90.0f));
    }

    if (imu.valid)
    {
        if (imu.attitudeValid)
        {
            putI32LE(frame, 14, scaleI32(imu.rollDeg, 1000.0f, 180.0f));
            putI32LE(frame, 18, scaleI32(imu.pitchDeg, 1000.0f, 180.0f));
            putI32LE(frame, 22, scaleI32(imu.yawDeg, 1000.0f, 180.0f));
        }

        putI32LE(frame, 26, scaleI32(imu.gx, 1000.0f, 2000.0f));
        putI32LE(frame, 30, scaleI32(imu.gy, 1000.0f, 2000.0f));
        putI32LE(frame, 34, scaleI32(imu.gz, 1000.0f, 2000.0f));

        // 加速度单位 g -> mg。
        putI16LE(frame, 38, scaleI16(imu.ax, 1000.0f, 32.0f));
        putI16LE(frame, 40, scaleI16(imu.ay, 1000.0f, 32.0f));
        putI16LE(frame, 42, scaleI16(imu.az, 1000.0f, 32.0f));

        const float dtUs = imu.dtSeconds * 1000000.0f;

        const bool dtValid =
            std::isfinite(dtUs) &&
            dtUs >= 0.0f &&
            dtUs <= 65535.0f &&
            !imu.dtOutOfRange;

        if (!dtValid)
            flags |= (1u << 3);

        const uint16_t dtEncoded =
            !std::isfinite(dtUs) || dtUs < 0.0f
                ? 0
                : static_cast<uint16_t>(
                      std::min(65535.0f, std::round(dtUs)));

        putU16LE(frame, 44, dtEncoded);
        putU16LE(frame, 4, flags);
    }

    const uint16_t crc = crc16(frame, 46);
    frame[46] = static_cast<uint8_t>(crc & 0xFF);
    frame[47] = static_cast<uint8_t>(crc >> 8);
}
