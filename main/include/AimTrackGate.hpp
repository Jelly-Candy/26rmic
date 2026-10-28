#pragma once

#include "ArmorTracker.hpp"

#include <cmath>
#include <cstdint>

class AimTrackGate {
public:
  enum class RejectReason {
    NONE,
    LOST,
    NON_FINITE,
    OUT_OF_BOUNDS,
    INVALID_BBOX,
    NEED_CONFIRMATION,
    TARGET_JUMP,
    HOLD_EXPIRED
  };

  explicit AimTrackGate(int imageWidth = 640, int imageHeight = 480,
                        float maxJumpPx = 80.0f,
                        uint32_t maxHoldMs = 60,
                        int maxHoldMissedFrames = 2)
      : imageWidth_(imageWidth > 0 ? imageWidth : 640),
        imageHeight_(imageHeight > 0 ? imageHeight : 480),
        maxJumpPx_(maxJumpPx > 0.0f ? maxJumpPx : 80.0f),
        maxHoldMs_(maxHoldMs),
        maxHoldMissedFrames_(maxHoldMissedFrames >= 0
                                 ? maxHoldMissedFrames
                                 : 0) {}

  bool update(const TrackedTarget &t, uint32_t now,
              cv::Point2f &center) {
    held_ = false;
    reason_ = RejectReason::NONE;

    if (t.state == TrackState::TRACKING) {
      if (!finite(t.center)) {
        invalidate(RejectReason::NON_FINITE);
        return false;
      }

      if (!insideImage(t.center)) {
        invalidate(RejectReason::OUT_OF_BOUNDS);
        return false;
      }

      if (t.bbox.width <= 0 || t.bbox.height <= 0 ||
          t.bbox.area() <= 0) {
        invalidate(RejectReason::INVALID_BBOX);
        return false;
      }

      // TRACKING 表示跟踪器已经匹配到当前帧的观测。
      // 首次出现或超时重现时，必须重新连续确认。
      if (!haveCandidate_ ||
          static_cast<uint32_t>(now - lastObservedMs_) > confirmGapMs_) {
        beginCandidate(t.center, now);
        reason_ = RejectReason::NEED_CONFIRMATION;
        return false;
      }

      const float jump = cv::norm(t.center - last_);
      if (!std::isfinite(jump) || jump > maxJumpPx_) {
        // 不允许大跳变目标直接获得瞄准授权。
        // 将其作为新的候选，下一帧仍需再次确认。
        beginCandidate(t.center, now);
        reason_ = RejectReason::TARGET_JUMP;
        return false;
      }

      last_ = t.center;
      lastObservedMs_ = now;
      if (consecutiveHits_ < requiredHits_)
        ++consecutiveHits_;

      haveCandidate_ = true;
      if (consecutiveHits_ < requiredHits_) {
        reason_ = RejectReason::NEED_CONFIRMATION;
        return false;
      }

      haveConfirmedTarget_ = true;
      center = last_;
      return true;
    }

    if (t.state == TrackState::TEMP_LOST) {
      const uint32_t elapsed = static_cast<uint32_t>(now - lastObservedMs_);
      if (haveConfirmedTarget_ && haveCandidate_ &&
          t.missedFrames >= 1 &&
          t.missedFrames <= maxHoldMissedFrames_ &&
          elapsed <= maxHoldMs_) {
        // 只保持最后一次真实观测的位置，不使用跟踪器预测位置。
        center = last_;
        held_ = true;
        return true;
      }

      invalidate(RejectReason::HOLD_EXPIRED);
      return false;
    }

    invalidate(RejectReason::LOST);
    return false;
  }

  bool held() const { return held_; }

  RejectReason reason() const { return reason_; }

  void reset() {
    last_ = cv::Point2f();
    lastObservedMs_ = 0;
    consecutiveHits_ = 0;
    haveCandidate_ = false;
    haveConfirmedTarget_ = false;
    held_ = false;
    reason_ = RejectReason::NONE;
  }

private:
  static constexpr unsigned requiredHits_ = 2;
  static constexpr uint32_t confirmGapMs_ = 120;

  int imageWidth_;
  int imageHeight_;
  float maxJumpPx_;
  uint32_t maxHoldMs_;
  int maxHoldMissedFrames_;

  cv::Point2f last_{};
  uint32_t lastObservedMs_ = 0;
  unsigned consecutiveHits_ = 0;
  bool haveCandidate_ = false;
  bool haveConfirmedTarget_ = false;
  bool held_ = false;
  RejectReason reason_ = RejectReason::NONE;

  static bool finite(const cv::Point2f &p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
  }

  bool insideImage(const cv::Point2f &p) const {
    return p.x >= 0.0f && p.y >= 0.0f &&
           p.x < static_cast<float>(imageWidth_) &&
           p.y < static_cast<float>(imageHeight_);
  }

  void beginCandidate(const cv::Point2f &center, uint32_t now) {
    last_ = center;
    lastObservedMs_ = now;
    consecutiveHits_ = 1;
    haveCandidate_ = true;
    haveConfirmedTarget_ = false;
    held_ = false;
  }

  void invalidate(RejectReason why) {
    reset();
    reason_ = why;
  }
};
