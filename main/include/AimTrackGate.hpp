#pragma once
#include "ArmorTracker.hpp"
#include <cmath>
#include <cstdint>
// At most two missed frames AND 60ms since a real matched observation.
// Hold the last measured center, never keep advancing an unobserved target.
class AimTrackGate {
public:
  bool update(const TrackedTarget &t, uint32_t now, cv::Point2f &center) {
    held_ = false;
    if (t.state == TrackState::TRACKING && std::isfinite(t.center.x) &&
        std::isfinite(t.center.y)) {
      if (!have_ || uint32_t(now - lastMs_) > 120U ||
          cv::norm(t.center - last_) > 80.0)
        hits_ = 1;
      else if (hits_ < 3)
        ++hits_;
      have_ = true;
      lastMs_ = now;
      last_ = t.center;
      center = last_;
      return true;
    }
    if (t.state == TrackState::TEMP_LOST && have_ && hits_ >= 2 &&
        t.missedFrames <= 2 && uint32_t(now - lastMs_) <= 60U) {
      center = last_;
      held_ = true;
      return true;
    }
    have_ = false;
    hits_ = 0;
    return false;
  }
  bool held() const { return held_; }

private:
  cv::Point2f last_{};
  uint32_t lastMs_ = 0;
  unsigned hits_ = 0;
  bool have_ = false, held_ = false;
};
