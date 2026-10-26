#include "ImuReader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

#include "maix_time.hpp"

namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr float kDegToRad = kPi / 180.0f;

// 加速度计只在模长接近 1g 时用于校正。
// 具体范围应按实际 IMU 噪声与运动强度调整。
constexpr float kAccMinG = 0.75f;
constexpr float kAccMaxG = 1.25f;

// 互补滤波：越大越信任陀螺仪的短期动态。
constexpr float kGyroWeight = 0.98f;

// 避免异常 dt 使积分突然跳变。
constexpr float kMinDt = 0.0005f;
constexpr float kMaxDt = 0.050f;
}

bool ImuReader::initialize()
{
    try
    {
        using namespace maix::ext_dev::imu;

        imu_ = std::make_unique<IMU>("default");

        if (imu_->calib_gyro_exists("default"))
        {
            imu_->load_calib_gyro("default");
            std::printf("[IMU] Loaded gyro calibration.\n");
        }
        else
        {
            std::printf(
                "[IMU] No calibration found.\n"
                "[IMU] Keep the camera completely still for 3 seconds...\n");

            const auto bias = imu_->calib_gyro(3000, 10, "default");

            std::printf(
                "[IMU] Calibration completed: "
                "bias=(%.5f, %.5f, %.5f) deg/s\n",
                bias.x, bias.y, bias.z);
        }

        imu_->load_calib_gyro("default");

        lastTimestampMs_ = maix::time::ticks_ms();
        attitudeInitialized_ = false;
        rollRad_ = pitchRad_ = yawRad_ = 0.0f;

        std::printf("[IMU] Initialization successful.\n");
        return true;
    }
    catch (const std::exception& e)
    {
        std::printf("[IMU] Initialization failed: %s\n", e.what());
        imu_.reset();
        return false;
    }
    catch (...)
    {
        std::printf("[IMU] Initialization failed: unknown exception.\n");
        imu_.reset();
        return false;
    }
}

void ImuReader::updateAttitude(Sample& sample)
{
    const float accNorm = std::sqrt(
        sample.ax * sample.ax +
        sample.ay * sample.ay +
        sample.az * sample.az);

    const bool accUsable =
        std::isfinite(accNorm) &&
        accNorm >= kAccMinG &&
        accNorm <= kAccMaxG;

    if (!attitudeInitialized_)
    {
        if (accUsable)
        {
            // 假设传感器静止时加速度计主要测得重力。
            // 这里的轴向和符号必须通过实机姿态测试确认。
            rollRad_ = std::atan2(sample.ay, sample.az);
            pitchRad_ = std::atan2(
                -sample.ax,
                std::sqrt(sample.ay * sample.ay +
                          sample.az * sample.az));
        }

        // 没有绝对航向参考时，初始 yaw 设为 0。
        yawRad_ = 0.0f;
        attitudeInitialized_ = true;
    }
    else if (!sample.dtOutOfRange)
    {
        const float dt = sample.dtSeconds;

        // 陀螺仪单位是 deg/s，转换为 rad/s 后积分。
        const float gx = sample.gx * kDegToRad;
        const float gy = sample.gy * kDegToRad;
        const float gz = sample.gz * kDegToRad;

        rollRad_ += gx * dt;
        pitchRad_ += gy * dt;
        yawRad_ += gz * dt;

        if (accUsable)
        {
            const float accRoll =
                std::atan2(sample.ay, sample.az);

            const float accPitch =
                std::atan2(
                    -sample.ax,
                    std::sqrt(sample.ay * sample.ay +
                              sample.az * sample.az));

            rollRad_ =
                kGyroWeight * rollRad_ +
                (1.0f - kGyroWeight) * accRoll;

            pitchRad_ =
                kGyroWeight * pitchRad_ +
                (1.0f - kGyroWeight) * accPitch;
        }
    }

    sample.rollDeg = rollRad_ * kRadToDeg;
    sample.pitchDeg = pitchRad_ * kRadToDeg;
    sample.yawDeg = yawRad_ * kRadToDeg;

    sample.attitudeValid =
        attitudeInitialized_ &&
        std::isfinite(sample.rollDeg) &&
        std::isfinite(sample.pitchDeg) &&
        std::isfinite(sample.yawDeg);
}

bool ImuReader::read(Sample& sample)
{
    if (!imu_)
        return false;

    try
    {
        // 在实际采样前后取时间，减少 read_all() 耗时造成的 dt 偏差。
        const auto data = imu_->read_all(true, false);
        const uint64_t now = maix::time::ticks_ms();

        sample.dtSeconds =
            lastTimestampMs_ == 0
                ? 0.0f
                : static_cast<float>(now - lastTimestampMs_) / 1000.0f;

        sample.timestampMs = now;
        lastTimestampMs_ = now;

        sample.dtOutOfRange =
            !std::isfinite(sample.dtSeconds) ||
            sample.dtSeconds < kMinDt ||
            sample.dtSeconds > kMaxDt;

        sample.ax = data.acc.x;
        sample.ay = data.acc.y;
        sample.az = data.acc.z;

        sample.gx = data.gyro.x;
        sample.gy = data.gyro.y;
        sample.gz = data.gyro.z;

        sample.temperature = data.temp;

        // 异常 dt 不参与积分，但仍保留当前原始传感器数据用于诊断。
        updateAttitude(sample);

        return
            std::isfinite(sample.ax) &&
            std::isfinite(sample.ay) &&
            std::isfinite(sample.az) &&
            std::isfinite(sample.gx) &&
            std::isfinite(sample.gy) &&
            std::isfinite(sample.gz);
    }
    catch (const std::exception& e)
    {
        std::printf("[IMU] Read failed: %s\n", e.what());
        return false;
    }
    catch (...)
    {
        std::printf("[IMU] Read failed: unknown exception.\n");
        return false;
    }
}
