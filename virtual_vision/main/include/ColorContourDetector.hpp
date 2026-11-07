#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <string>

// 要识别的敌方颜色
enum class TargetColor { BLUE, RED };

// 一块候选装甲板的识别结果
struct VisualDetection {
  cv::Point2f center; // 装甲板中心，单位：像素
  cv::Rect bbox;      // 装甲板估计外接矩形

  // 顺序固定：左上、右上、右下、左下
  cv::Point2f corners[4];
};

// 调试信息：供主程序绘制中间处理结果

struct LightDiagnostic {
    cv::RotatedRect rect;
    int id = 0;
    const char *source = "";
    std::string reason;
    double area = 0;
    float length = 0, width = 0, ratio = 0, fill = 0;
    float rectAngle = 0, fitAngle = 0;
    int colored = -1, checked = -1; // -1 means color stage was not reached
};

struct TiltPairDiagnostic {
    int left=-1,right=-1;
    float localOffset=0, gap=0, angleDiff=0, score=0;
    std::string reason;
};
struct ContourDebugInfo {
    std::vector<TiltPairDiagnostic> tiltPairs;
    unsigned joinedBluePairs = 0; // pair rejected: colour continues across the gap
    std::vector<LightDiagnostic> diagnostics;
    cv::Mat mask;
    std::vector<cv::RotatedRect> lights;

    // 所有通过几何条件的候选框
    std::vector<cv::Rect> pairs;

    // 所有候选的几何评分，分数越低越好
    std::vector<float> pairScores;

    // 最终通过筛选的候选框
    std::vector<cv::Rect> keptPairs;
};


// 装甲板识别器
class ColorContourDetector {
public:
    explicit ColorContourDetector(TargetColor enemyColor = TargetColor::BLUE);

    std::vector<VisualDetection> detect(
        const cv::Mat &frame,
        ContourDebugInfo *debug = nullptr
    );

    void setTargetColor(TargetColor color);

    // 新增：设置灰度二值化阈值
    void setBinaryThreshold(int threshold);

private:
    TargetColor enemyColor_;

    // 新增：默认亮度阈值，后续根据实机效果调整
    int binaryThreshold_ = 180;

    cv::Mat makeColorMask(const cv::Mat &frame);
    void preprocessMask(cv::Mat &mask);
};

