#pragma once

#include "Armorboard_Detector.hpp"

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

enum class TrackState { LOST, TRACKING, TEMP_LOST };

struct TrackedTarget {
  cv::Point2f center{0.0f, 0.0f};
  cv::Point2f velocity{0.0f, 0.0f};
  cv::Rect bbox;
  TrackState state = TrackState::LOST;
  int missedFrames = 0;
  // 当前跟踪目标对应的本帧检测候选索引。
  // -1 表示本帧没有真实检测匹配，例如 TEMP_LOST。
  int detectionIndex = -1;
};

class ArmorTracker {
public:
  ArmorTracker(float baseMatchDistance = 80.0f, int maxMissedFrames = 5)
      : baseMatchDistance_(std::max(1.0f, baseMatchDistance)),
        maxMissedFrames_(std::max(0, maxMissedFrames)) {
    kf_.init(4, 2, 0, CV_32F);

    // 状态：[x, y, vx, vy]
    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0,
                            0, 1, 0, 0, 0, 0, 1);

    // 观测：[x, y]
    kf_.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
    kf_.measurementMatrix.at<float>(0, 0) = 1.0f;
    kf_.measurementMatrix.at<float>(1, 1) = 1.0f;

    // 过程噪声将在每次 update() 时根据 dtSeconds 动态更新。
    kf_.processNoiseCov = cv::Mat::zeros(4, 4, CV_32F);

    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(9.0));

    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

    cv::setIdentity(kf_.errorCovPre, cv::Scalar::all(1.0));

    kf_.statePost = cv::Mat::zeros(4, 1, CV_32F);
    kf_.statePre = cv::Mat::zeros(4, 1, CV_32F);
  }

  const TrackedTarget &update(const std::vector<ArmorTarget> &detections,
                              float dtSeconds = 1.0f / 30.0f) {
    // 无效或异常的帧间隔使用安全默认值。
    // 限制 dt，避免偶发卡顿导致预测一步跳得过远。
    if (!std::isfinite(dtSeconds) || dtSeconds < 0.001f || dtSeconds > 0.1f) {
      dtSeconds = 1.0f / 30.0f;
    }

    // 状态：[x, y, vx, vy]
    // 速度单位为像素/秒，因此位置预测必须使用实际 dt。
    kf_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, dtSeconds, 0, 0, 1,
                            0, dtSeconds, 0, 0, 1, 0, 0, 0, 0, 1);

    // 使用匀速模型，并考虑未知加速度带来的不确定性。
    // 状态顺序：[x, y, vx, vy]。
    // q 表示加速度噪声的方差，单位约为 pixel^2 / second^3。
    constexpr float accelerationNoiseVariance = 14400.0f;

    const float dt2 = dtSeconds * dtSeconds;
    const float dt3 = dt2 * dtSeconds;
    const float dt4 = dt2 * dt2;

    const float q = accelerationNoiseVariance;

    // 按白噪声加速度模型构造过程噪声协方差矩阵 Q。
    kf_.processNoiseCov =
        (cv::Mat_<float>(4, 4) << q * dt4 / 4.0f, 0.0f, q * dt3 / 2.0f, 0.0f,
         0.0f, q * dt4 / 4.0f, 0.0f, q * dt3 / 2.0f, q * dt3 / 2.0f, 0.0f,
         q * dt2, 0.0f, 0.0f, q * dt3 / 2.0f, 0.0f, q * dt2);

    // 每次 update 默认没有本帧匹配结果。
    target_.detectionIndex = -1;
    // LOST 状态下没有历史目标可预测。
    if (target_.state == TrackState::LOST) {
      if (!detections.empty()) {
        const int index = selectLargestIndex(detections);

        if (index >= 0) {
          initialize(detections[index], index);
        }
      }

      return target_;
    }

    // 每个有效跟踪帧只预测一次。
    const cv::Mat prediction = kf_.predict();

    const cv::Point2f predictedCenter(prediction.at<float>(0, 0),
                                      prediction.at<float>(1, 0));

    const float vx = prediction.at<float>(2, 0);
    const float vy = prediction.at<float>(3, 0);

    // 速度单位是像素/秒，换算成当前帧预计移动的像素距离。
    const float speed = std::sqrt(vx * vx + vy * vy);
    const float predictedDisplacement = speed * dtSeconds;

    // 根据预计位移和连续丢失帧数调整匹配范围。
    const float gate =
        std::min(240.0f, baseMatchDistance_ + 1.5f * predictedDisplacement +
                             15.0f * target_.missedFrames);

    // 将上一帧的框平移到预测中心
    // 保留上一帧检测框的宽高，以卡尔曼预测中心重新构造预测框。
    const int boxWidth = target_.bbox.width;
    const int boxHeight = target_.bbox.height;

    cv::Rect predictedBox(
        static_cast<int>(std::lround(predictedCenter.x - boxWidth * 0.5f)),
        static_cast<int>(std::lround(predictedCenter.y - boxHeight * 0.5f)),
        boxWidth, boxHeight);

    int bestIndex = -1;
    float bestScore = std::numeric_limits<float>::max();

    for (int i = 0; i < static_cast<int>(detections.size()); ++i) {
      const ArmorTarget &candidate = detections[i];
      const cv::Point2f delta = candidate.center - predictedCenter;

      const float distance = std::sqrt(delta.x * delta.x + delta.y * delta.y);

      // 第一关：中心距离不能超过门限
      if (distance > gate) {
        continue;
      }

      // 第二关：候选框的面积不能与原目标差异过大
      const float oldArea = static_cast<float>(predictedBox.area());
      const float newArea = static_cast<float>(candidate.bbox.area());

      if (oldArea <= 0.0f || newArea <= 0.0f) {
        continue;
      }

      const float areaRatio =
          std::min(oldArea, newArea) / std::max(oldArea, newArea);

      if (areaRatio < 0.35f) {
        continue;
      }

      // 第三关：计算预测框与候选框的 IoU
      const cv::Rect intersection = predictedBox & candidate.bbox;

      const float intersectionArea = static_cast<float>(intersection.area());

      const float unionArea = oldArea + newArea - intersectionArea;

      const float iou = unionArea > 0.0f ? intersectionArea / unionArea : 0.0f;

      // 综合评分：距离越小、重叠越大、面积越接近越好
      const float score =
          distance + 35.0f * (1.0f - iou) + 20.0f * (1.0f - areaRatio);

      if (score < bestScore) {
        bestScore = score;
        bestIndex = i;
      }
    }

    // 找到匹配目标：用真实观测修正卡尔曼预测。
    if (bestIndex >= 0) {
      const ArmorTarget &detection = detections[bestIndex];
      target_.detectionIndex = bestIndex;

      const cv::Mat measurement =
          (cv::Mat_<float>(2, 1) << detection.center.x, detection.center.y);

      const cv::Mat corrected = kf_.correct(measurement);

      target_.center =
          cv::Point2f(corrected.at<float>(0, 0), corrected.at<float>(1, 0));

      target_.velocity =
          cv::Point2f(corrected.at<float>(2, 0), corrected.at<float>(3, 0));

      target_.bbox = detection.bbox;
      target_.missedFrames = 0;
      target_.state = TrackState::TRACKING;

      return target_;
    }

    // 没有匹配：将预测结果推进到下一帧的先验状态，
    // 使短暂丢失时的预测不会停留在旧位置。
    kf_.statePost = kf_.statePre.clone();
    kf_.errorCovPost = kf_.errorCovPre.clone();

    ++target_.missedFrames;

    if (target_.missedFrames > maxMissedFrames_) {
      reset();
      return target_;
    }

    target_.center = predictedCenter;
    target_.velocity = cv::Point2f(vx, vy);

    // 预测框始终以预测中心为中心，保持原有宽高。
    target_.bbox.x = static_cast<int>(
        std::lround(target_.center.x - target_.bbox.width * 0.5f));
    target_.bbox.y = static_cast<int>(
        std::lround(target_.center.y - target_.bbox.height * 0.5f));

    target_.state = TrackState::TEMP_LOST;

    return target_;
  }

  const TrackedTarget &getTarget() const { return target_; }

  void reset() {
    target_ = TrackedTarget{};

    kf_.statePost = cv::Mat::zeros(4, 1, CV_32F);
    kf_.statePre = cv::Mat::zeros(4, 1, CV_32F);

    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

    cv::setIdentity(kf_.errorCovPre, cv::Scalar::all(1.0));
  }

private:
  float baseMatchDistance_;
  int maxMissedFrames_;

  cv::KalmanFilter kf_;
  TrackedTarget target_;

  static int selectLargestIndex(const std::vector<ArmorTarget> &detections) {
    if (detections.empty()) {
      return -1;
    }

    int bestIndex = 0;
    int bestArea = detections[0].bbox.area();

    for (int i = 1; i < static_cast<int>(detections.size()); ++i) {
      const int area = detections[i].bbox.area();

      if (area > bestArea) {
        bestArea = area;
        bestIndex = i;
      }
    }

    return bestIndex;
  }

  void initialize(const ArmorTarget &detection, int detectionIndex) {
    target_ = TrackedTarget{};

    target_.center = detection.center;
    target_.velocity = cv::Point2f(0.0f, 0.0f);
    target_.bbox = detection.bbox;
    target_.state = TrackState::TRACKING;

    target_.missedFrames = 0;
    target_.detectionIndex = detectionIndex;

    kf_.statePost = (cv::Mat_<float>(4, 1) << detection.center.x,
                     detection.center.y, 0.0f, 0.0f);

    kf_.statePre = kf_.statePost.clone();

    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

    cv::setIdentity(kf_.errorCovPre, cv::Scalar::all(1.0));
  }
};
