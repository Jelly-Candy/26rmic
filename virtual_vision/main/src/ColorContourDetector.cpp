#include "ColorContourDetector.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>
using namespace cv;
using namespace std;
// 几何评分上限：越低越严格。
// 3.0 是初始调试值，需根据真实画面调整。
static constexpr float MAX_PAIR_SCORE = 3.0f;
namespace {
struct LightBar {
  cv::RotatedRect rect;
  cv::Point2f center;
  float length;
  float width;
  float angle; // 长轴相对水平轴的角度
};
struct ContourPairCandidate {
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

} // internal per-translation-unit geometry types

// 从近似竖直的灯条旋转矩形中提取上、下端点。
// 假定灯条的长轴接近竖直。
static void getLightEndpoints(const cv::RotatedRect &rect, cv::Point2f &top,
                              cv::Point2f &bottom) {
  cv::Point2f p[4];
  rect.points(p);

  std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) {
    return a.y < b.y;
  });

  top = (p[0] + p[1]) * 0.5f;
  bottom = (p[2] + p[3]) * 0.5f;
}

ColorContourDetector::ColorContourDetector(TargetColor enemyColor) : enemyColor_(enemyColor) {}
void ColorContourDetector::setTargetColor(TargetColor color) { enemyColor_ = color; }
void ColorContourDetector::setBinaryThreshold(int threshold) {
  binaryThreshold_ = std::max(0, std::min(threshold, 255));
}
Mat ColorContourDetector::makeColorMask(const Mat &frame) {
  Mat hsv;
  cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
  Mat mask;
  if (enemyColor_ == TargetColor::BLUE) {
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
void ColorContourDetector::preprocessMask(Mat &mask) {
  Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
  morphologyEx(mask, mask, MORPH_CLOSE, kernel);
  morphologyEx(mask, mask, MORPH_OPEN, kernel);
}
// Blue lamps can have a saturated cyan core (G almost equals B). Reject
// white/yellow with B-R evidence, and reject green with a bounded G/B relation.
// Red behaviour deliberately retains the previous predicate.
static bool enemyColorPixel(int dominant, int opposite, int green,
                            TargetColor color, int minimum, bool extraction) {
  if (dominant < minimum) return false;
  if (color == TargetColor::BLUE)
    return dominant-opposite >= 60 && dominant+12 >= green;
  return dominant-std::max(opposite,green) >= (extraction ? 35 : 25);
}

static bool hasTargetColor(const cv::Mat &frame,
                          const std::vector<cv::Point> &contour,
                          TargetColor enemyColor, int *coloredOut = nullptr,
                          int *countOut = nullptr) {
  cv::Rect roi = cv::boundingRect(contour);
  // Include a 2px fringe: an overexposed white core can have colored edges.
  roi = cv::Rect(roi.x - 2, roi.y - 2, roi.width + 4, roi.height + 4) &
        cv::Rect(0, 0, frame.cols, frame.rows);
  if (roi.empty())
    return false;
  cv::Mat inside = cv::Mat::zeros(roi.size(), CV_8UC1);
  std::vector<cv::Point> shifted;
  for (const auto &p : contour)
    shifted.emplace_back(p.x - roi.x, p.y - roi.y);
  cv::drawContours(inside, std::vector<std::vector<cv::Point>>{shifted}, 0,
                   cv::Scalar(255), cv::FILLED);
  cv::dilate(inside, inside,
             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
  int colored = 0, count = 0;
  for (int y = 0; y < roi.height; ++y)
    for (int x = 0; x < roi.width; ++x) {
      if (!inside.at<uchar>(y, x))
        continue;
      const auto p = frame.at<cv::Vec3b>(roi.y + y, roi.x + x);
      ++count;
      int dominant = enemyColor == TargetColor::RED ? p[0] : p[2];
      int other = enemyColor == TargetColor::RED ? p[2] : p[0];
      if (enemyColorPixel(dominant, other, int(p[1]), enemyColor, 100, false))
        ++colored;
    }
  if (coloredOut) *coloredOut = colored;
  if (countOut) *countOut = count;
  return colored >= 5 && colored >= count * 0.06f;
}

// Require a dark interior in both pairing paths. Allow blue illumination on a
// black plate, but retain red/green ceilings and contrast against both lamps.
static bool blackInteriorBetween(const cv::Mat &rgb, const LightBar &a,
                                 const LightBar &b) {
  cv::Point2f at, ab, bt, bb;
  getLightEndpoints(a.rect, at, ab);
  getLightEndpoints(b.rect, bt, bb);
  const int columns = std::min(48, std::max(8,
      int(cv::norm(b.center-a.center))));
  const int rows = std::min(24, std::max(5,
      int(std::min(a.length,b.length)*0.7f)));
  const auto luminance=[](const cv::Vec3b &p) {
    return (77*p[0]+150*p[1]+29*p[2])/256.0f;
  };
  const auto lampLuma=[&](const LightBar &lamp) {
    cv::Point2f top,bottom;getLightEndpoints(lamp.rect,top,bottom);
    float sum=0;
    for(int i=0;i<5;++i) {
      const auto point=top+(bottom-top)*(0.25f+0.50f*(i+0.5f)/5);
      const int x=cvRound(point.x),y=cvRound(point.y);
      if(x<0 || x>=rgb.cols || y<0 || y>=rgb.rows)return 0.0f;
      sum+=luminance(rgb.at<cv::Vec3b>(y,x));
    }
    return sum/5;
  };
  const float bright=std::min(lampLuma(a),lampLuma(b));
  const float spillLumaLimit=std::min(110.0f,bright*0.60f);
  int black=0, total=0;
  for(int row=0; row<rows; ++row) {
    // Exclude lamp tips and inset each inner edge by half its width plus
    // a small halo margin. Interpolate along the actual tilted lamp axes.
    const float t=0.15f+0.70f*(row+0.5f)/rows;
    const cv::Point2f left=at+(ab-at)*t, right=bt+(bb-bt)*t;
    const cv::Point2f delta=right-left;
    const float distance=cv::norm(delta);
    const float insetA=a.width*0.5f+std::max(2.0f,a.width*0.25f);
    const float insetB=b.width*0.5f+std::max(2.0f,b.width*0.25f);
    if(distance-insetA-insetB<3.0f) return false;
    int rowBlack=0;
    for(int col=0; col<columns; ++col) {
      const float offset=insetA+(distance-insetA-insetB)*(col+0.5f)/columns;
      const cv::Point2f point=left+delta*(offset/distance);
      const int x=cvRound(point.x), y=cvRound(point.y);
      if(x<0 || x>=rgb.cols || y<0 || y>=rgb.rows) return false;
      const auto p=rgb.at<cv::Vec3b>(y,x);
      const bool neutralDark=std::max({p[0],p[1],p[2]})<=90;
      const bool blueLitDark=bright>=100 && p[0]<=100 && p[1]<=100 &&
          p[2]>=p[0]+20 && p[2]+12>=p[1] &&
          int(p[2])-std::min(int(p[0]),int(p[1]))<=210 &&
          luminance(p)<=spillLumaLimit;
      if(neutralDark || blueLitDark) ++rowBlack;
    }
    // Each scanline must be mostly black, not just the overall average.
    if(rowBlack*100<columns*70) return false;
    black+=rowBlack; total+=columns;
  }
  return total>0 && black*100>=total*85;
}

// Pair geometry is measured in the mean lamp frame, rather than image rows.
// Existing primary targets retain first use of their lamps.
static std::vector<ContourPairCandidate> tiltedCandidates(const cv::Mat &rgb,
    const std::vector<LightBar> &lights,const std::vector<bool> &used,TargetColor color,ContourDebugInfo *debug) {
  std::vector<ContourPairCandidate> result;
  if(lights.size()>64)return result; // bound cubic obstruction checks on clutter
  for(size_t i=0;i<lights.size();++i)for(size_t j=i+1;j<lights.size();++j){
    if(used[i] || used[j])continue;
    const LightBar *a=&lights[i],*b=&lights[j];int ai=int(i),bi=int(j);
    if(a->center.x>b->center.x){std::swap(a,b);std::swap(ai,bi);}
    const float avg=(a->length+b->length)*0.5f;
    const float heightRatio=std::min(a->length,b->length)/std::max(a->length,b->length);
    float angle=std::abs(a->angle-b->angle);angle=std::min(angle,180.0f-angle);
    const float radians=float(CV_PI/180.0);
    const float theta=0.5f*std::atan2(std::sin(2*a->angle*radians)+std::sin(2*b->angle*radians),
      std::cos(2*a->angle*radians)+std::cos(2*b->angle*radians));
    const cv::Point2f along(std::cos(theta),std::sin(theta)),across(-along.y,along.x);
    const auto delta=b->center-a->center;
    const float local=std::abs(delta.dot(along))/avg,gap=std::abs(delta.dot(across))/avg;
    const float score=2*(1-heightRatio)+2*local+angle/20.0f+std::abs(gap-2.0f)/2.0f;
    const auto record=[&](const char *reason){if(debug)debug->tiltPairs.push_back({ai,bi,local,gap,angle,score,reason});};
    if(heightRatio<0.65f || local>0.85f || gap<1.20f || gap>4.50f || angle>18.0f || score>3.0f){record("GEOMETRY");continue;}
    if(color==TargetColor::BLUE && !blackInteriorBetween(rgb,*a,*b)) {
      record("NON_BLACK_INTERIOR");continue;
    }
    // Do not span over a third similarly oriented lamp from an adjacent plate.
    bool intervening=false;const float distance2=delta.dot(delta);
    for(size_t k=0;k<lights.size();++k){if(k==i || k==j)continue;
      float diff=std::abs(lights[k].angle-a->angle);diff=std::min(diff,180.0f-diff);if(diff>20)continue;
      const auto rel=lights[k].center-a->center;const float t=rel.dot(delta)/distance2;
      if(t>0.15f && t<0.85f && cv::norm(rel-t*delta)<0.65f*avg){intervening=true;break;}
    }
    if(intervening){record("INTERVENING_LIGHT");continue;}
    std::vector<cv::Point2f> corners;cv::Point2f points[4];
    a->rect.points(points);corners.insert(corners.end(),points,points+4);b->rect.points(points);corners.insert(corners.end(),points,points+4);
    cv::Rect box=cv::boundingRect(corners);box.x-=2;box.y-=2;box.width+=4;box.height+=4;box &= cv::Rect(0,0,rgb.cols,rgb.rows);
    if(box.empty())continue;
    record("CANDIDATE");result.push_back({box,(a->center+b->center)*0.5f,score,i,j,heightRatio,local,gap,angle});
  }
  std::stable_sort(result.begin(),result.end(),[](const auto&a,const auto&b){return a.score<b.score;});return result;
}

vector<VisualDetection> ColorContourDetector::detect(const Mat &frame,
                                          ContourDebugInfo *debug) {
  vector<VisualDetection> targets;
  const char *tiltEnv=std::getenv("RMIC_TILT_PAIRS");
  const bool tiltedPairs=enemyColor_==TargetColor::BLUE && tiltEnv && tiltEnv[0]=='1' && tiltEnv[1]=='\0';
  // 检查输入图像
  if (frame.empty() || frame.type() != CV_8UC3) {
    if(debug)*debug=ContourDebugInfo{};
    return targets;
  }
  cv::Mat gray;
  cv::Mat mask;
  // 灰度提取亮区域：适应灯条白色发光核心
  cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
  cv::threshold(gray, mask, binaryThreshold_, 255, cv::THRESH_BINARY);
  // Luminance alone suppresses saturated red/blue lamps (blue Y is only 29
  // at full brightness). Recover bright, clearly enemy-colored pixels.
  // Cyan-aware blue evidence and the legacy red predicate share one pixel
  // pass. Both colour and validation stages apply the same channel relation.
  cv::Mat colorMask(frame.rows, frame.cols, CV_8UC1);
  cv::Mat cyanCoreMask;
  if(enemyColor_==TargetColor::BLUE)cyanCoreMask.create(frame.rows,frame.cols,CV_8UC1);
  const int dominantIndex = enemyColor_ == TargetColor::RED ? 0 : 2;
  const int otherIndex = 2 - dominantIndex;
  const int threshold=binaryThreshold_;
  const int minimumCoreGreen=std::max(180,threshold+20);
  for (int y = 0; y < frame.rows; ++y) {
    const uchar *src = frame.ptr<uchar>(y);
    uchar *dst = colorMask.ptr<uchar>(y);
    uchar *core=cyanCoreMask.empty()?nullptr:cyanCoreMask.ptr<uchar>(y);
    for (int x = 0; x < frame.cols; ++x, src += 3) {
      const int dominant = src[dominantIndex];
      const int other = src[otherIndex];
      // Keep the legacy chroma path separate from the saturated cyan core:
      // joining them before contours lets the outer blue halo widen the bar.
      dst[x] = dominant>threshold && dominant-std::max(other,int(src[1]))>=35 ? 255 : 0;
      if(core)core[x]=src[1]>=minimumCoreGreen &&
          enemyColorPixel(dominant,other,src[1],TargetColor::BLUE,threshold+1,true) ? 255 : 0;
    }
  }
  // 先闭运算连接较小的断裂，再开运算去除小噪点
  // 加强闭运算，尝试连接灯条内部的小断裂
  cv::Mat closeKernel =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3));

  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, closeKernel);
  // Close only very small gaps in the supported colour region. A narrower
  // horizontal kernel avoids pulling the coloured halo into nearby background.
  cv::morphologyEx(colorMask, colorMask, cv::MORPH_CLOSE, closeKernel);
  if(!cyanCoreMask.empty()) {
    cv::morphologyEx(cyanCoreMask,cyanCoreMask,cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3)));
    // Remove isolated colour speckles before contour allocation. A 1x3
    // vertical opening preserves narrow lamps rather than eroding their width.
    cv::morphologyEx(cyanCoreMask,cyanCoreMask,cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT,cv::Size(1,3)));
  }

  if (debug != nullptr) {
    debug->tiltPairs.clear();
    debug->joinedBluePairs=0;
    cv::bitwise_or(mask, colorMask, debug->mask);
    if(!cyanCoreMask.empty())cv::bitwise_or(debug->mask,cyanCoreMask,debug->mask);
    debug->lights.clear();
    debug->diagnostics.clear();
    debug->pairs.clear();
    debug->pairScores.clear();
    debug->keptPairs.clear();
  }
  std::vector<LightBar> lights;
  std::vector<LightBar> deferredLights;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // Separate passes prevent colored halos from joining a white lamp core
  // to the bright window/wall and destroying its thin-bar geometry.
  std::vector<std::vector<cv::Point>> coloredContours;
  cv::findContours(colorMask, coloredContours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  const size_t grayContourCount = contours.size();
  contours.insert(contours.end(), coloredContours.begin(),
                  coloredContours.end());
  std::vector<std::vector<cv::Point>> cyanContours;
  if(!cyanCoreMask.empty())cv::findContours(cyanCoreMask,cyanContours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
  const size_t cyanContourCount=cyanContours.size();
  // Prefer compact, bright core geometry; later duplicate rejection keeps a
  // grey/halo version of the same bar from overriding it.
  contours.insert(contours.begin(),cyanContours.begin(),cyanContours.end());
  // DEBUG
  int rejectArea = 0;
  int rejectShape = 0;
  int rejectFill = 0;
  int rejectRectAngle = 0;
  int rejectFitAngle = 0;
  int rejectColor = 0;
  int acceptedLights = 0;
  size_t contourIndex = 0;
  for (const auto &contour : contours) {
    LightDiagnostic diagnostic;
    diagnostic.id = static_cast<int>(contourIndex);
    diagnostic.source = contourIndex < cyanContourCount ? "CYAN_CORE" :
        (contourIndex < cyanContourCount+grayContourCount ? "GRAY" : "COLOR");
    ++contourIndex;
    const auto record = [&](const char *reason) {
      if (debug) { diagnostic.reason = reason; debug->diagnostics.push_back(diagnostic); }
    };
    const double area = cv::contourArea(contour);
    cv::RotatedRect rect = cv::minAreaRect(contour);
    const float w = rect.size.width;
    const float h = rect.size.height;
    const float length = std::max(w, h);
    const float width = std::min(w, h);
    const float rectArea = w * h;
    const float ratio = length / width;
    diagnostic.rect = rect; diagnostic.area = area;
    diagnostic.length = length; diagnostic.width = width;
    diagnostic.ratio = ratio;
    diagnostic.fill = rectArea > 0 ? area / rectArea : 0;
    // 面积筛选
    if (area < 20) {
      record("AREA<20");
      ++rejectArea;
      continue;
    }
    if (width < 1.5f || length < 8.0f) {
      record(width < 1.5f ? "WIDTH<1.5" : "LENGTH<8");
      continue;
    }
    // A bright, densely filled cyan core can represent a physically wider
    // lamp. Only that branch gets a lower ratio floor, with stricter verticality.
    const bool denseCore=contourIndex<=cyanContourCount && diagnostic.fill>=0.70f;
    const float minimumRatio=denseCore?2.0f:2.5f;
    const float legacyMinimumAngle=denseCore && ratio<2.5f?70.0f:55.0f;
    const float minimumAngle=tiltedPairs?55.0f:legacyMinimumAngle;
    if (ratio < minimumRatio || ratio > 15.0f) {
      record(ratio < minimumRatio ? (denseCore?"CORE_RATIO<2":"RATIO<2.5") : "RATIO>15");
      ++rejectShape;
      continue;
    }
    // 过滤填充率过低的细长外接矩形

    if (rectArea <= 0.0f || area / rectArea < 0.25f) {
      record("FILL<0.25");
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
    diagnostic.rectAngle = angle;
    if (std::abs(angle) < minimumAngle) {
      record(minimumAngle>55?"CORE_RECT_ANGLE<70":"RECT_ANGLE<55");
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
    diagnostic.fitAngle = fittedAngle;
    if (std::abs(fittedAngle) < minimumAngle) {
      record(minimumAngle>55?"CORE_FIT_ANGLE<70":"FIT_ANGLE<55");
      ++rejectFitAngle;
      continue;
    }
    // 几何筛选通过后，再验证敌方颜色
    if (!hasTargetColor(frame, contour, enemyColor_,
                       &diagnostic.colored, &diagnostic.checked)) {
      record(diagnostic.colored < 5 ? "COLOR_COUNT<5" : "COLOR_RATIO<0.06");
      ++rejectColor;
      continue;
    }
    if(tiltedPairs && (std::abs(angle)<legacyMinimumAngle || std::abs(fittedAngle)<legacyMinimumAngle)) {
      record("TILT_ONLY");deferredLights.push_back({rect,rect.center,length,width,fittedAngle});continue;
    }
    bool sameBar = false;
    for (const auto &old : lights)
      if (cv::norm(old.center - rect.center) <
              std::max(4.0f, 0.25f * std::min(old.length, length)) &&
          std::min(old.length, length) / std::max(old.length, length) > 0.60f) {
        sameBar = true;
        break;
      }
    if (sameBar) { record("DUPLICATE"); continue; }
    record("ACCEPT");
    lights.push_back({rect, rect.center, length, width, fittedAngle});
    ++acceptedLights;
  }

  const size_t primaryLightCount=lights.size();
  for(const auto &candidate:deferredLights){
    bool duplicate=false;
    for(const auto &old:lights)if(cv::norm(old.center-candidate.center)<
      std::max(4.0f,0.25f*std::min(old.length,candidate.length)) &&
      std::min(old.length,candidate.length)/std::max(old.length,candidate.length)>0.60f){duplicate=true;break;}
    if(!duplicate)lights.push_back(candidate);
  }
  static unsigned logFrame = 0;
  if ((logFrame++ % 30U) == 0U)
    std::cout << "[ColorContourDetector] contours=" << contours.size()
              << " area=" << rejectArea << " shape=" << rejectShape
              << " fill=" << rejectFill << " rectAngle=" << rejectRectAngle
              << " fitAngle=" << rejectFitAngle << " color=" << rejectColor
              << " accepted=" << acceptedLights << std::endl;

  if (debug != nullptr) {
    for (const auto &light : lights) {
      debug->lights.push_back(light.rect);
    }
  }
  // 4. 两两配对
  // 保存所有通过几何条件的装甲板候选
  std::vector<ContourPairCandidate> candidates;
  for (size_t i = 0; i < lights.size(); ++i) {
    for (size_t j = i + 1; j < lights.size(); ++j) {
      if(i>=primaryLightCount || j>=primaryLightCount)continue;
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
      // Never span another similarly oriented lamp from a neighbouring target.
      if(enemyColor_==TargetColor::BLUE) {
        bool intervening=false;
        const auto delta=right->center-left->center;
        const float distance2=delta.dot(delta);
        for(size_t k=0;k<lights.size();++k) {
          if(k==i || k==j) continue;
          float diff=std::abs(lights[k].angle-left->angle);
          diff=std::min(diff,180.0f-diff);
          if(diff>20.0f) continue;
          const auto rel=lights[k].center-left->center;
          const float t=rel.dot(delta)/distance2;
          if(t>0.15f && t<0.85f && cv::norm(rel-t*delta)<0.65f*avgLen) {
            intervening=true;break;
          }
        }
        if(intervening) continue;
      }
      if(enemyColor_==TargetColor::BLUE && !blackInteriorBetween(frame,*left,*right)) {
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
            [](const ContourPairCandidate &a, const ContourPairCandidate &b) {
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
    VisualDetection target{};
    target.center = candidate.center;
    target.bbox = candidate.bbox;

    // candidate.lightA / lightB 对应本次配对的两根灯条。
    // 按图像 x 坐标确定左右，再计算各自的上、下端点。
    const LightBar &lightA = lights[candidate.lightA];
    const LightBar &lightB = lights[candidate.lightB];

    const LightBar *left = &lightA;
    const LightBar *right = &lightB;

    if (left->center.x > right->center.x) {
      std::swap(left, right);
    }

    cv::Point2f leftTop, leftBottom;
    cv::Point2f rightTop, rightBottom;

    getLightEndpoints(left->rect, leftTop, leftBottom);
    getLightEndpoints(right->rect, rightTop, rightBottom);

    // 固定顺序：左上、右上、右下、左下。
    // 注意：这是由灯条几何估计出的四角，并非真实装甲板边缘角点。
    target.corners[0] = leftTop;
    target.corners[1] = rightTop;
    target.corners[2] = rightBottom;
    target.corners[3] = leftBottom;

    targets.push_back(target);
  }
  if(tiltedPairs){
    const auto recovered=tiltedCandidates(frame,lights,usedLights,enemyColor_,debug);
    for(const auto &pair:recovered){
      if(usedLights[pair.lightA] || usedLights[pair.lightB])continue;
      bool overlap=false;for(const auto &old:targets){const float smaller=float(std::min(old.bbox.area(),pair.bbox.area()));
        if(smaller>0 && float((old.bbox & pair.bbox).area())/smaller>0.65f){overlap=true;break;}}
      if(overlap)continue;
      const LightBar *a=&lights[pair.lightA],*b=&lights[pair.lightB];if(a->center.x>b->center.x)std::swap(a,b);
      VisualDetection target{};target.center=pair.center;target.bbox=pair.bbox;
      getLightEndpoints(a->rect,target.corners[0],target.corners[3]);getLightEndpoints(b->rect,target.corners[1],target.corners[2]);
      targets.push_back(target);usedLights[pair.lightA]=usedLights[pair.lightB]=true;
      if(debug){debug->pairs.push_back(pair.bbox);debug->pairScores.push_back(pair.score);debug->keptPairs.push_back(pair.bbox);
        debug->tiltPairs.push_back({int(a-lights.data()),int(b-lights.data()),pair.verticalRatio,pair.gapRatio,pair.angleDiff,pair.score,"ACCEPT"});}
    }
  }
  return targets;
}
