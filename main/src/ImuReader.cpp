
#include "ImuReader.hpp"

#include <cstdio>
#include <exception>

#include "maix_time.hpp"

bool ImuReader::initialize()
{
    try
    {
        using namespace maix::ext_dev::imu;

        // 自动选择开发板对应的板载 IMU。
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
                "[IMU] Keep the camera completely still for 3 seconds...\n"
            );

            // 采集期间必须保持静止。
            const auto bias =
                imu_->calib_gyro(3000, 10, "default");

            std::printf(
                "[IMU] Calibration completed: "
                "bias=(%.5f, %.5f, %.5f) deg/s\n",
                bias.x, bias.y, bias.z
            );
        }

        // 重新加载校准数据，确保后续 read_all() 使用它。
        imu_->load_calib_gyro("default");

        lastTimestampMs_ = maix::time::ticks_ms();

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

bool ImuReader::read(Sample& sample)
{
    if (!imu_)
        return false;

    try
    {
        const uint64_t now = maix::time::ticks_ms();

        sample.dtSeconds =
            lastTimestampMs_ == 0
                ? 0.0f
                : static_cast<float>(now - lastTimestampMs_) / 1000.0f;

        sample.timestampMs = now;
        lastTimestampMs_ = now;

        // gyro 使用 deg/s，acc 使用 g。
        const auto data = imu_->read_all(true, false);

        sample.ax = data.acc.x;
        sample.ay = data.acc.y;
        sample.az = data.acc.z;

        sample.gx = data.gyro.x;
        sample.gy = data.gyro.y;
        sample.gz = data.gyro.z;

        sample.temperature = data.temp;

        return true;
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
