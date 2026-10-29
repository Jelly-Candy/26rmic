
#ifndef FRAME_TIMING_HPP
#define FRAME_TIMING_HPP

#include <chrono>
#include <algorithm>

class FrameTiming
{
public:
    struct Result
    {
        double dtSeconds = 0.0;  // 与上一帧的时间间隔
        double fps = 0.0;        // 平滑后的帧率
        bool valid = false;      // 本次 dt 是否有效
    };

    FrameTiming() = default;

    // 每处理一帧调用一次，返回本帧与上一帧的实际时间间隔。
    Result tick()
    {
        using Clock = std::chrono::steady_clock;

        const auto now = Clock::now();
        Result result;

        if (!_initialized)
        {
            _lastTime = now;
            _initialized = true;
            result.valid = false;
            return result;
        }

        const std::chrono::duration<double> elapsed = now - _lastTime;
        _lastTime = now;

        const double rawDt = elapsed.count();

        // 避免异常时间间隔污染后续滤波。
        // dt 超出范围时仍更新计时基准，但不作为有效测量输出。
        if (rawDt <= 0.0 || rawDt > 0.5)
        {
            result.valid = false;
            result.dtSeconds = 0.0;
            result.fps = _fps;
            return result;
        }

        result.dtSeconds = rawDt;
        result.valid = true;

        const double currentFps = 1.0 / rawDt;

        // 指数平滑，减少单帧耗时波动造成的 FPS 跳变。
        constexpr double alpha = 0.1;

        if (_fps <= 0.0)
        {
            _fps = currentFps;
        }
        else
        {
            _fps = alpha * currentFps + (1.0 - alpha) * _fps;
        }

        result.fps = _fps;
        return result;
    }

    void reset()
    {
        _initialized = false;
        _fps = 0.0;
    }

    double fps() const
    {
        return _fps;
    }

private:
    using Clock = std::chrono::steady_clock;

    Clock::time_point _lastTime{};
    bool _initialized = false;
    double _fps = 0.0;
};

#endif
