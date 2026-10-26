#pragma once

#include <cstdint>
#include <memory>

#include "maix_imu.hpp"

class ImuReader
{
public:
    struct Sample
    {
        float gx = 0.0f;
        float gy = 0.0f;
        float gz = 0.0f;

        float ax = 0.0f;
        float ay = 0.0f;
        float az = 0.0f;

        float temperature = 0.0f;
        uint64_t timestampMs = 0;
        float dtSeconds = 0.0f;

        float rollDeg = 0.0f;
        float pitchDeg = 0.0f;
        float yawDeg = 0.0f;

        bool attitudeValid = false;
        bool dtOutOfRange = false;
    };

    ImuReader() = default;
    ~ImuReader() = default;

    ImuReader(const ImuReader&) = delete;
    ImuReader& operator=(const ImuReader&) = delete;

    bool initialize();
    bool read(Sample& sample);

    bool isInitialized() const
    {
        return static_cast<bool>(imu_);
    }

private:
    void updateAttitude(Sample& sample);

    std::unique_ptr<maix::ext_dev::imu::IMU> imu_;

    uint64_t lastTimestampMs_ = 0;

    float rollRad_ = 0.0f;
    float pitchRad_ = 0.0f;
    float yawRad_ = 0.0f;

    bool attitudeInitialized_ = false;
};