#include "Armorboard_Detector.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <utility>
#include <vector>
using namespace cv;
using namespace std;
// 几何评分上限：越低越严格。
// 3.0 是初始调试值，需根据真实画面调整。
static constexpr float MAX_PAIR_SCORE = 3.0f;
struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f center;
  float length;
  float width;
  float angle; // 长轴相对水平轴的角度
};
struct ArmorPairCandidate {
  cv::Rect bbox;
  cv::Point2f center;
  float score;
  size_t lightA;
  size_t lightB;
  float heightRatio;
  float verticalRatio;
  float gapRatio;
  float angleDiff;
};
ArmorDetector::ArmorDetector(EnemyColor enemyColor) : enemyColor_(enemyColor) {}
void ArmorDetector::setEnemyColor(EnemyColor color) { enemyColor_ = color; }
void ArmorDetector::setBinaryThreshold(int threshold) {
  binaryThreshold_ = std::max(0, std::min(threshold, 255));
}
Mat ArmorDetector::makeColorMask(const Mat &frame) {
  Mat hsv;
  cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
  Mat mask;
  if (enemyColor_ == EnemyColor::BLUE) {
    // 保留你原来的蓝色 HSV 阈值
    inRange(hsv, Scalar(90, 200, 120), Scalar(140, 255, 255), mask);

  } else {
    cv::Mat redMask1, redMask2;
    cv::inRange(hsv, cv::Scalar(0, 180, 130), cv::Scalar(10, 255, 255),
                redMask1);
    cv::inRange(hsv, cv::Scalar(170, 180, 130), cv::Scalar(180, 255, 255),
                redMask2);
    cv::bitwise_or(redMask1, redMask2, mask);
  }
  return mask;
}
void ArmorDetector::preprocessMask(Mat &mask) {
  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
  morphologyEx(mask, mask, MORPH_CLOSE, kernel);
  morphologyEx(mask, mask, MORPH_OPEN, kernel);
}
static bool hasEnemyColor(const cv::Mat &frame,
                          const std::vector<cv::Point> &contour,
                          EnemyColor enemyColor) {
  cv::Rect roi=cv::boundingRect(contour);
  // Include a 2px fringe: an overexposed white core can have colored edges.
  roi=cv::Rect(roi.x-2,roi.y-2,roi.width+4,roi.height+4)&cv::Rect(0,0,frame.cols,frame.rows);
  if(roi.empty())return false;
  cv::Mat inside=cv::Mat::zeros(roi.size(),CV_8UC1);
  std::vector<cv::Point> shifted;for(const auto &p:contour)shifted.emplace_back(p.x-roi.x,p.y-roi.y);
  cv::drawContours(inside,std::vector<std::vector<cv::Point>>{shifted},0,cv::Scalar(255),cv::FILLED);
  cv::dilate(inside,inside,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(5,5)));
  int colored=0,count=0;
  for(int y=0;y<roi.height;++y)for(int x=0;x<roi.width;++x){
    if(!inside.at<uchar>(y,x))continue;
    const auto p=frame.at<cv::Vec3b>(roi.y+y,roi.x+x);++count;
    int dominant=enemyColor==EnemyColor::RED?p[0]:p[2];
    int other=enemyColor==EnemyColor::RED?p[2]:p[0];
    if(dominant>=100 && dominant-std::max(other,int(p[1]))>=25)++colored;
  }
  return colored>=5 && colored>=count*0.06f;
}
vector<ArmorTarget> ArmorDetector::detect(const Mat &frame,
                                          ArmorDebugInfo *debug) {
  vector<ArmorTarget> targets;
  // 检查输入图像
  if (frame.empty() || frame.type() != CV_8UC3) {
    return targets;
  }
  cv::Mat gray;
  cv::Mat mask;
  // 灰度提取亮区域：适应灯条白色发光核心
  cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
  cv::threshold(gray, mask, binaryThreshold_, 255, cv::THRESH_BINARY);
  // Luminance alone suppresses saturated red/blue lamps (blue Y is only 29
  // at full brightness). Recover bright, clearly enemy-colored pixels.
  cv::Mat colorMask=cv::Mat::zeros(frame.size(),CV_8UC1);
  for(int y=0;y<frame.rows;++y){const auto *src=frame.ptr<cv::Vec3b>(y);auto *dst=colorMask.ptr<uchar>(y);
    for(int x=0;x<frame.cols;++x){const auto p=src[x];int dominant=enemyColor_==EnemyColor::RED?p[0]:p[2];int other=enemyColor_==EnemyColor::RED?p[2]:p[0];
      if(dominant>binaryThreshold_ && dominant-std::max(other,int(p[1]))>=35)dst[x]=255;
    }
  }
  // 先闭运算连接较小的断裂，再开运算去除小噪点
  // 加强闭运算，尝试连接灯条内部的小断裂
  cv::Mat closeKernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3));

  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, closeKernel);
  cv::morphologyEx(colorMask,colorMask,cv::MORPH_CLOSE,closeKernel);

  if (debug != nullptr) {
    cv::bitwise_or(mask,colorMask,debug->mask);
    debug->lights.clear();
    debug->pairs.clear();
    debug->pairScores.clear();
    debug->keptPairs.clear();
  }
  std::vector<LightBar> lights;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // Separate passes prevent colored halos from joining a white lamp core
  // to the bright window/wall and destroying its thin-bar geometry.
  std::vector<std::vector<cv::Point>> coloredContours;
  cv::findContours(colorMask,coloredContours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  contours.insert(contours.end(),coloredContours.begin(),coloredContours.end());
  // DEBUG
  int rejectArea = 0;
  int rejectShape = 0;
  int rejectFill = 0;
  int rejectRectAngle = 0;
  int rejectFitAngle = 0;
  int rejectColor = 0;
  int acceptedLights = 0;
  for (const auto &contour : contours) {
    const double area = cv::contourArea(contour);
    cv::RotatedRect rect = cv::minAreaRect(contour);
    const float w = rect.size.width;
    const float h = rect.size.height;
    const float length = std::max(w, h);
    const float width = std::min(w, h);
    const float rectArea = w * h;
    const float ratio = length / width;
    // 面积筛选
    if (area < 20) {
      ++rejectArea;
      continue;
    }
    if (width < 1.5f || length < 8.0f) {
      continue;
    }
    // 排除太短、太宽或极端细长的亮斑
    if (ratio < 2.5f || ratio > 15.0f) {
      ++rejectShape;
      continue;
    }
    if (width > 0.0f && length / width < 2.5f) {
      continue;
    }
    // 过滤填充率过低的细长外接矩形

    if (rectArea <= 0.0f || area / rectArea < 0.25f) {
      ++rejectFill;
      continue;
    }
    // 灯条长轴方向统一到 [-90, 90]
    float angle = rect.angle;
    if (w < h) {
      angle += 90.0f;
    }
    while (angle > 90.0f)
      angle -= 180.0f;
    while (angle < -90.0f)
      angle += 180.0f;
    if (std::abs(angle) < 55.0f) {
      ++rejectRectAngle;
      continue;
    }
    // 用轮廓拟合主方向，避免仅依赖外接矩形角度
    cv::Vec4f line;
    cv::fitLine(contour, line, cv::DIST_L2, 0, 0.01, 0.01);
    float fittedAngle =
        static_cast<float>(std::atan2(line[1], line[0]) * 180.0 / CV_PI);
    while (fittedAngle > 90.0f)
      fittedAngle -= 180.0f;
    while (fittedAngle < -90.0f)
      fittedAngle += 180.0f;
    if (std::abs(fittedAngle) < 55.0f) {
      ++rejectFitAngle;
      continue;
    }
    // 几何筛选通过后，再验证敌方颜色
    if (!hasEnemyColor(frame, contour, enemyColor_)) {
      ++rejectColor;
      continue;
    }
    bool sameBar=false;
    for(const auto &old:lights)if(cv::norm(old.center-rect.center)<std::max(4.0f,0.25f*std::min(old.length,length)) && std::min(old.length,length)/std::max(old.length,length)>0.60f){sameBar=true;break;}
    if(sameBar)continue;
    lights.push_back({rect, rect.center, length, width, fittedAngle});
    ++acceptedLights;
  }

  static unsigned logFrame=0;
  if((logFrame++%30U)==0U)std::cout
    << "[ArmorDetector] contours=" << contours.size()
    << " area=" << rejectArea
    << " shape=" << rejectShape
    << " fill=" << rejectFill
    << " rectAngle=" << rejectRectAngle
    << " fitAngle=" << rejectFitAngle
    << " color=" << rejectColor
    << " accepted=" << acceptedLights
    << std::endl;

  if (debug != nullptr) {
    for (const auto &light : lights) {
      debug->lights.push_back(light.rect);
    }
  }
  // 4. 两两配对
  // 保存所有通过几何条件的装甲板候选
  std::vector<ArmorPairCandidate> candidates;
  for (size_t i = 0; i < lights.size(); ++i) {
    for (size_t j = i + 1; j < lights.size(); ++j) {
      const auto &a = lights[i];
      const auto &b = lights[j];
      // 按图像中的 x 坐标排序
      const LightBar *left = &a;
      const LightBar *right = &b;
      if (left->center.x > right->center.x) {
        std::swap(left, right);
      }
      const float maxLen = std::max(left->length, right->length);
      const float minLen = std::min(left->length, right->length);
      const float avgLen = (left->length + right->length) * 0.5f;
      if (minLen <= 0.0f || avgLen <= 1.0f) {
        continue;
      }
      // 条件 1：灯条长度比例
      const float heightRatio = minLen / maxLen;
      if (heightRatio < 0.55f) {
        continue;
      }
      // 条件 2：中心垂直偏差
      const float verticalDiff = std::abs(left->center.y - right->center.y);
      if (verticalDiff > avgLen * 0.60f) {
        continue;
      }
      // 条件 3：两根灯条的水平间距
      const float horizontalDist = right->center.x - left->center.x;
      const float gapRatio = horizontalDist / avgLen;
      if (gapRatio < 0.50f || gapRatio > 4.50f) {
        continue;
      }
      // 条件 4：灯条长轴角度差
      // 前面已经把 angle 归一化到 [-90, 90]。
      // 这里必须考虑长轴方向以 180 度为周期。
      float angleDiff = std::abs(left->angle - right->angle);
      angleDiff = std::min(angleDiff, 180.0f - angleDiff);
      if (angleDiff > 20.0f) {
        continue;
      }
      // 条件 5：几何评分，越低越好
      const float heightPenalty = 1.0f - heightRatio;
      const float verticalPenalty = verticalDiff / avgLen;
      const float anglePenalty = angleDiff / 20.0f;
      const float gapPenalty = std::abs(gapRatio - 2.0f) / 2.0f;
      const float score = 2.0f * heightPenalty + 2.0f * verticalPenalty +
                          1.0f * anglePenalty + 1.0f * gapPenalty;
      // 6. 生成候选矩形框
      const float centerX = (left->center.x + right->center.x) * 0.5f;
      const float centerY = (left->center.y + right->center.y) * 0.5f;
      const float boxWidth = horizontalDist + avgLen * 0.80f;
      const float boxHeight = avgLen * 2.00f;
      int x = static_cast<int>(centerX - boxWidth * 0.5f);
      int y = static_cast<int>(centerY - boxHeight * 0.5f);
      int w = static_cast<int>(boxWidth);
      int h = static_cast<int>(boxHeight);
      // 先限制左上角，再裁剪宽高，防止越界
      x = std::max(0, std::min(x, frame.cols));
      y = std::max(0, std::min(y, frame.rows));
      w = std::min(w, frame.cols - x);
      h = std::min(h, frame.rows - y);
      if (w <= 0 || h <= 0) {
        continue;
      }
      // PAIRS 调试模式保留所有通过基础几何条件的候选，
      // 这样可以观察哪些候选因为评分过高而被排除。
      if (debug != nullptr) {
        debug->pairs.emplace_back(x, y, w, h);
        debug->pairScores.push_back(score);
      }
      // 评分门槛：分数过高的候选不进入最终筛选流程。
      if (score > MAX_PAIR_SCORE) {
        continue;
      }
      candidates.push_back({cv::Rect(x, y, w, h), cv::Point2f(centerX, centerY),
                            score, i, j, heightRatio, verticalDiff / avgLen,
                            gapRatio, angleDiff});
    }
  }
  // 7. 优先处理几何评分较好的候选
  std::sort(candidates.begin(), candidates.end(),
            [](const ArmorPairCandidate &a, const ArmorPairCandidate &b) {
              return a.score < b.score;
            });
  // 8. 贪心筛选：每根灯条最多用于一个最终目标
  std::vector<bool> usedLights(lights.size(), false);
  std::vector<cv::Rect> keptBoxes;
  for (const auto &candidate : candidates) {
    // 避免一根灯条参与多个最终配对
    if (usedLights[candidate.lightA] || usedLights[candidate.lightB]) {
      continue;
    }
    bool duplicated = false;
    for (const auto &kept : keptBoxes) {
      const cv::Rect intersection = candidate.bbox & kept;
      const float intersectionArea = static_cast<float>(intersection.area());
      const float minArea =
          static_cast<float>(std::min(candidate.bbox.area(), kept.area()));
      // 相交面积占较小矩形面积的比例
      if (minArea > 0.0f && intersectionArea / minArea > 0.65f) {
        duplicated = true;
        break;
      }
    }
    if (duplicated) {
      continue;
    }
    // 只有最终接受的候选才占用灯条
    usedLights[candidate.lightA] = true;
    usedLights[candidate.lightB] = true;
    keptBoxes.push_back(candidate.bbox);
    if (debug != nullptr) {
      debug->keptPairs.push_back(candidate.bbox);
    }
    ArmorTarget target;
    target.center = candidate.center;
    target.bbox = candidate.bbox;
    targets.push_back(target);
  }
  return targets;
}