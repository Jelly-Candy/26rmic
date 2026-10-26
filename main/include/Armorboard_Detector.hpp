#pragma once

#include <opencv2/core.hpp>
#include <vector>

// 要识别的敌方颜色
enum class EnemyColor { BLUE, RED };

// 一块候选装甲板的识别结果
struct ArmorTarget {
  cv::Point2f center; // 装甲板中心，单位：像素
  cv::Rect bbox;      // 装甲板估计外接矩形

  // 顺序固定：左上、右上、右下、左下
  cv::Point2f corners[4];
};

// 调试信息：供主程序绘制中间处理结果

struct ArmorDebugInfo {
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
class ArmorDetector {
public:
    explicit ArmorDetector(EnemyColor enemyColor = EnemyColor::BLUE);

    std::vector<ArmorTarget> detect(
        const cv::Mat &frame,
        ArmorDebugInfo *debug = nullptr
    );

    void setEnemyColor(EnemyColor color);

    // 新增：设置灰度二值化阈值
    void setBinaryThreshold(int threshold);

private:
    EnemyColor enemyColor_;

    // 新增：默认亮度阈值，后续根据实机效果调整
    int binaryThreshold_ = 180;

    cv::Mat makeColorMask(const cv::Mat &frame);
    void preprocessMask(cv::Mat &mask);
};

