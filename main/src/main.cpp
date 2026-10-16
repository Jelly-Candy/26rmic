#include "maix_basic.hpp"
#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_touchscreen.hpp"

#include "AimController.hpp"
#include "ArmorTracker.hpp"
#include "Armorboard_Detector.hpp"
#include "CameraModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace maix;

// UI 按钮区域，坐标对应 640x480 画面
static const cv::Rect BLUE_BUTTON(40, 150, 240, 100);
static const cv::Rect RED_BUTTON(360, 150, 240, 100);
static const cv::Rect CONFIRM_BUTTON(180, 300, 280, 70);
static const cv::Rect DEBUG_MODE_BUTTON(440, 390, 180, 65);

enum class DebugMode { MASK, LIGHTS, PAIRS, FINAL };

static DebugMode next_debug_mode(DebugMode mode) {
  switch (mode) {
  case DebugMode::MASK:
    return DebugMode::LIGHTS;
  case DebugMode::LIGHTS:
    return DebugMode::PAIRS;
  case DebugMode::PAIRS:
    return DebugMode::FINAL;
  case DebugMode::FINAL:
    return DebugMode::MASK;
  }

  return DebugMode::MASK;
}

static const char *debug_mode_name(DebugMode mode) {
  switch (mode) {
  case DebugMode::MASK:
    return "MASK";
  case DebugMode::LIGHTS:
    return "LIGHTS";
  case DebugMode::PAIRS:
    return "PAIRS";
  case DebugMode::FINAL:
    return "FINAL";
  }

  return "UNKNOWN";
}

// 判断触点是否在按钮内
static bool in_button(int x, int y, const cv::Rect &button) {
  return button.contains(cv::Point(x, y));
}

// 绘制颜色选择界面
static void draw_color_menu(cv::Mat &rgb, EnemyColor selected) {
  rgb.setTo(cv::Scalar(25, 25, 25));

  cv::putText(rgb, "Select Enemy Color", cv::Point(115, 85),
              cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

  cv::rectangle(rgb, BLUE_BUTTON,
                selected == EnemyColor::BLUE ? cv::Scalar(0, 255, 0)
                                             : cv::Scalar(80, 150, 255),
                selected == EnemyColor::BLUE ? 5 : 2);

  cv::putText(rgb, "BLUE", cv::Point(115, 210), cv::FONT_HERSHEY_SIMPLEX, 1.2,
              cv::Scalar(255, 255, 255), 3);

  cv::rectangle(rgb, RED_BUTTON,
                selected == EnemyColor::RED ? cv::Scalar(0, 255, 0)
                                            : cv::Scalar(255, 100, 100),
                selected == EnemyColor::RED ? 5 : 2);

  cv::putText(rgb, "RED", cv::Point(445, 210), cv::FONT_HERSHEY_SIMPLEX, 1.2,
              cv::Scalar(255, 255, 255), 3);

  cv::rectangle(rgb, CONFIRM_BUTTON, cv::Scalar(80, 200, 80), 2);

  cv::putText(rgb, "CONFIRM", cv::Point(235, 345), cv::FONT_HERSHEY_SIMPLEX,
              1.0, cv::Scalar(255, 255, 255), 2);
}

int _main(int argc, char *argv[]) {
  camera::Camera cam(640, 480, image::Format::FMT_RGB888);

  display::Display disp;

  // 触屏对象
  touchscreen::TouchScreen ts;

  cam.skip_frames(5);

  EnemyColor selectedColor = EnemyColor::BLUE;
  bool confirmed = false;
  bool lastPressed = false;

  ArmorDetector detector(selectedColor);
  ArmorTracker tracker(80.0f, 5);
  CameraModel cameraModel;

  if (!cameraModel.load("/root/camera_intrinsics.yml")) {
    log::error("camera calibration load failed");
    return -1;
  }

  if (!cameraModel.setImageSize(640, 480)) {
    log::error("camera image size configuration failed");
    return -1;
  }

  log::info("=== ARMOR APP NEW VERSION ===");

  while (!app::need_exit() && !confirmed) {
    image::Image *img = cam.read();

    if (img == nullptr) {
      time::sleep_ms(10);
      continue;
    }

    if (img->format() != image::Format::FMT_RGB888) {
      delete img;
      time::sleep_ms(10);
      continue;
    }

    cv::Mat rgb(img->height(), img->width(), CV_8UC3, img->data());

    int touchX = 0;
    int touchY = 0;
    bool pressed = false;

    err::Err touchErr = ts.read(touchX, touchY, pressed);

    if (touchErr == err::ERR_NONE) {
      // 只在触摸刚刚按下时处理，避免长按重复触发
      if (pressed && !lastPressed) {
        if (in_button(touchX, touchY, BLUE_BUTTON)) {
          selectedColor = EnemyColor::BLUE;
        } else if (in_button(touchX, touchY, RED_BUTTON)) {
          selectedColor = EnemyColor::RED;
        } else if (in_button(touchX, touchY, CONFIRM_BUTTON)) {
          confirmed = true;
        }
      }

      lastPressed = pressed;
    }

    draw_color_menu(rgb, selectedColor);

    // 画面几何中心：红色十字
    cv::drawMarker(rgb, cv::Point(img->width() / 2, img->height() / 2),
                   cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 30, 3);

    // 当前内参主点：绿色十字
    cv::drawMarker(rgb, cv::Point(303, 238), cv::Scalar(0, 255, 0),
                   cv::MARKER_CROSS, 30, 3);

    // 左上角：蓝色实心方块
    cv::rectangle(rgb, cv::Rect(10, 10, 40, 40), cv::Scalar(0, 0, 255), -1);
    disp.show(*img);

    delete img;
    time::sleep_ms(10);
  }

  if (app::need_exit()) {
    return 0;
  }
  // 将选择结果应用到检测器
  detector.setEnemyColor(selectedColor);

  log::info("Enemy color selected: %s",
            selectedColor == EnemyColor::BLUE ? "BLUE" : "RED");

  DebugMode debugMode = DebugMode::FINAL;
  bool debugLastPressed = false;

  while (!app::need_exit()) {
    image::Image *img = cam.read();

    if (img == nullptr) {
      time::sleep_ms(10);
      continue;
    }

    if (img->format() != image::Format::FMT_RGB888) {
      delete img;
      continue;
    }

    const int width = img->width();
    const int height = img->height();

    cv::Mat rgb(height, width, CV_8UC3, img->data());

    // 获取当前帧的检测结果和中间调试数据
    ArmorDebugInfo debugInfo;
    std::vector<ArmorTarget> targets = detector.detect(rgb, &debugInfo);
    const TrackedTarget &tracked = tracker.update(targets);

    if (tracked.state == TrackState::TRACKING) {

      cv::Point2f angle = cameraModel.pixelToAngle(tracked.center);

      cv::putText(rgb, "Yaw:" + std::to_string(angle.x), cv::Point(15, 190),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

      cv::putText(rgb, "Pitch:" + std::to_string(angle.y), cv::Point(15, 220),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
    }

    // 读取触摸屏
    int touchX = 0;
    int touchY = 0;
    bool pressed = false;

    err::Err touchErr = ts.read(touchX, touchY, pressed);

    if (touchErr == err::ERR_NONE) {
      if (pressed && !debugLastPressed) {
        if (DEBUG_MODE_BUTTON.contains(cv::Point(touchX, touchY))) {
          debugMode = next_debug_mode(debugMode);
        }
      }

      debugLastPressed = pressed;
    }

    // MASK：显示颜色掩膜

    if (debugMode == DebugMode::MASK) {
      if (!debugInfo.mask.empty()) {
        cv::cvtColor(debugInfo.mask, rgb, cv::COLOR_GRAY2RGB);
      } else {
        cv::putText(rgb, "MASK EMPTY!", cv::Point(15, 100),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
      }
    }

    // LIGHTS：原图上显示通过筛选的灯条
    else if (debugMode == DebugMode::LIGHTS) {
      for (const auto &light : debugInfo.lights) {
        cv::Point2f points[4];
        light.points(points);

        for (int i = 0; i < 4; ++i) {
          cv::line(rgb, points[i], points[(i + 1) % 4], cv::Scalar(0, 255, 0),
                   2);
        }

        cv::circle(rgb, light.center, 3, cv::Scalar(255, 0, 0), -1);
      }
    }

    // PAIRS：青色框显示所有候选，绿色框显示最终保留候选
    else if (debugMode == DebugMode::PAIRS) {
      for (size_t i = 0; i < debugInfo.pairs.size(); ++i) {
        const cv::Rect &box = debugInfo.pairs[i];

        // 青色：通过基础几何条件的候选框
        cv::rectangle(rgb, box, cv::Scalar(255, 255, 0), 2);

        // 显示几何评分，越低代表越符合当前评分模型
        if (i < debugInfo.pairScores.size()) {
          char text[32];

          std::snprintf(text, sizeof(text), "%.2f", debugInfo.pairScores[i]);

          cv::putText(rgb, text, cv::Point(box.x, std::max(15, box.y - 5)),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0),
                      1);
        }
      }

      // 绿色：经过灯条占用和重叠去重后保留的候选
      for (const cv::Rect &box : debugInfo.keptPairs) {
        cv::rectangle(rgb, box, cv::Scalar(0, 255, 0), 2);
      }
    }

    // FINAL：显示检测结果和跟踪结果
    else {
      // 绿色框：当前帧检测到的装甲板
      for (const auto &target : targets) {
        cv::rectangle(rgb, target.bbox, cv::Scalar(0, 255, 0), 2);

        // 红色点：当前帧检测中心
        cv::circle(rgb,
                   cv::Point(static_cast<int>(target.center.x),
                             static_cast<int>(target.center.y)),
                   4, cv::Scalar(255, 0, 0), -1);
      }

      // 跟踪状态和跟踪中心
      const char *stateText = "LOST";

      switch (tracked.state) {
      case TrackState::TRACKING:
        stateText = "TRACKING";
        break;
      case TrackState::TEMP_LOST:
        stateText = "TEMP_LOST";
        break;
      case TrackState::LOST:
        stateText = "LOST";
        break;
      }

      cv::putText(rgb, std::string("Track: ") + stateText, cv::Point(15, 100),
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

      if (tracked.state != TrackState::LOST) {
        // 黄色点：跟踪器输出的平滑中心
        cv::circle(rgb,
                   cv::Point(static_cast<int>(tracked.center.x),
                             static_cast<int>(tracked.center.y)),
                   6, cv::Scalar(255, 255, 0), 2);

        // 显示跟踪中心坐标
        std::string centerText =
            "Track Center: (" +
            std::to_string(static_cast<int>(tracked.center.x)) + ", " +
            std::to_string(static_cast<int>(tracked.center.y)) + ")";

        cv::putText(rgb, centerText, cv::Point(15, 130),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);

        if (tracked.state == TrackState::TEMP_LOST) {
          cv::putText(rgb, "Missed: " + std::to_string(tracked.missedFrames),
                      cv::Point(15, 160), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cv::Scalar(0, 165, 255), 2);
        }
      }
    }

    // 调试状态和数量
    cv::putText(rgb, std::string("Mode: ") + debug_mode_name(debugMode),
                cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 0), 2);

    std::string countText;

    if (debugMode == DebugMode::MASK) {
      countText =
          "White pixels: " + std::to_string(cv::countNonZero(debugInfo.mask));
    } else if (debugMode == DebugMode::LIGHTS) {
      countText = "Lights: " + std::to_string(debugInfo.lights.size());
    } else if (debugMode == DebugMode::PAIRS) {
      countText = "Pairs: " + std::to_string(debugInfo.pairs.size());
    } else {
      countText = "Targets: " + std::to_string(targets.size());
    }

    cv::putText(rgb, countText, cv::Point(15, 60), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(255, 255, 255), 2);

    // 触摸切换模式
    cv::rectangle(rgb, DEBUG_MODE_BUTTON, cv::Scalar(255, 255, 255), 2);

    cv::putText(rgb, "NEXT MODE",
                cv::Point(DEBUG_MODE_BUTTON.x + 10, DEBUG_MODE_BUTTON.y + 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

    disp.show(*img);
    delete img;
  }

  return 0;
}

int main(int argc, char *argv[]) {
  sys::register_default_signal_handle();
  CATCH_EXCEPTION_RUN_RETURN(_main, -1, argc, argv);
}