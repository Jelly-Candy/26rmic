#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>

// Shared by the clock and tracker: long gaps invalidate tracking history rather
// than pretending the delayed frame arrived after a normal 1/30-second step.
class FrameTiming {
public:
  static constexpr double DEFAULT_DT_SECONDS = 1.0 / 30.0;
  static constexpr double MIN_DT_SECONDS = 0.001;
  static constexpr double MAX_DT_SECONDS = 0.120;

  explicit FrameTiming(double maximumInterval = MAX_DT_SECONDS)
      : maximumInterval_(std::isfinite(maximumInterval) && maximumInterval >= MAX_DT_SECONDS && maximumInterval <= 0.300 ? maximumInterval : MAX_DT_SECONDS) {}

  struct Result {
    double dtSeconds = DEFAULT_DT_SECONDS;
    double rawDtSeconds = 0.0;
    double fps = 0.0;
    bool valid = false;
    bool resetTracking = false;
  };

  static bool validDt(double dt) {
    return std::isfinite(dt) && dt >= MIN_DT_SECONDS &&
           dt <= MAX_DT_SECONDS;
  }

  Result tickAt(uint64_t nowUs) {
    Result result;
    if (!initialized_) {
      initialized_ = true;
      lastUs_ = nowUs;
      return result;
    }
    const double dt = nowUs > lastUs_ ? (nowUs - lastUs_) * 1e-6 : 0.0;
    lastUs_ = nowUs;
    result.rawDtSeconds = dt;
    if (!std::isfinite(dt) || dt < MIN_DT_SECONDS || dt > maximumInterval_) {
      fps_ = 0.0;
      result.resetTracking = true;
      return result;
    }
    const double fps = 1.0 / dt;
    fps_ = fps_ > 0.0 ? 0.1 * fps + 0.9 * fps_ : fps;
    result.dtSeconds = dt;
    result.fps = fps_;
    result.valid = true;
    return result;
  }

  Result tick() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return tickAt(static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count()));
  }

  void reset() { initialized_ = false; lastUs_ = 0; fps_ = 0.0; }

private:
  uint64_t lastUs_ = 0;
  double fps_ = 0.0;
  double maximumInterval_ = MAX_DT_SECONDS;
  bool initialized_ = false;
};
