
#pragma once

#include <cstdint>
#include <memory>
#include <string>

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
    };

    ImuReader() = default;
    ~ImuReader() = default;

    ImuReader(const ImuReader&) = delete;
    ImuReader& operator=(const ImuReader&) = delete;

    // 初始化 IMU，并加载已有校准数据。
    // 若没有校准数据，则在设备静止时执行校准。
    bool initialize();

    // 读取一次数据。
    bool read(Sample& sample);

    bool isInitialized() const
    {
        return static_cast<bool>(imu_);
    }

private:
    std::unique_ptr<maix::ext_dev::imu::IMU> imu_;
    uint64_t lastTimestampMs_ = 0;
};
