#include "maix_app.hpp"
#include "maix_camera.hpp"
#include "maix_display.hpp"
#include "maix_jpg_stream.hpp"
#include "maix_log.hpp"
#include "maix_sys.hpp"
#include "maix_time.hpp"
#include "maix_touchscreen.hpp"
#include "maix_uart.hpp"

#include "AimResult.hpp"
#include "AimTrackGate.hpp"
#include "AimUart.hpp"
#include "ArmorPoseSolver.hpp"
#include "ArmorTracker.hpp"
#include "Armorboard_Detector.hpp"
#include "CameraModel.hpp"
#include "ImuReader.hpp"
#include "UartFrameWriter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <memory>
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

static const char *aimRejectReasonName(AimTrackGate::RejectReason reason) {
  using R = AimTrackGate::RejectReason;
  switch (reason) {
  case R::NONE:
    return "OK";
  case R::LOST:
    return "LOST";
  case R::NON_FINITE:
    return "NON_FINITE";
  case R::OUT_OF_BOUNDS:
    return "OUT_OF_BOUNDS";
  case R::INVALID_BBOX:
    return "INVALID_BBOX";
  case R::NEED_CONFIRMATION:
    return "CONFIRM";
  case R::TARGET_JUMP:
    return "JUMP";
  case R::HOLD_EXPIRED:
    return "HOLD_EXPIRED";
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

/**
 * UART 启动前自检：
 * 1. 编码固定的 AimResult 测试数据
 * 2. 通过实际 UART 发送 20 帧
 * 3. 检查每次 write 返回的字节数
 *
 * 注意：这里的 IMU 数据是模拟值，仅用于测试通信协议。
 */
bool testAimUart(peripheral::uart::UART &serial) {
  constexpr int TEST_FRAME_COUNT = 20;
  uint8_t frame[AimUart::FRAME_SIZE];

  AimResult result{};
  result.targetFound = true;
  result.yawDeg = 1.234f;
  result.pitchDeg = -2.500f;

  // 模拟 IMU 数据，仅用于验证 UART 帧格式
  result.imu.valid = true;
  result.imu.attitudeValid = true;
  result.imu.rollDeg = 1.0f;
  result.imu.pitchDeg = 2.0f;
  result.imu.yawDeg = 3.0f;

  result.imu.gx = 10.0f;
  result.imu.gy = 20.0f;
  result.imu.gz = 30.0f;

  result.imu.ax = 0.1f;
  result.imu.ay = 0.2f;
  result.imu.az = 1.0f;
  result.imu.dtSeconds = 0.005f;

  int successCount = 0;
  uint8_t testSeq = 0;

  log::info("========== UART SELF TEST BEGIN ==========");
  log::info("Device test: %d frames, %d bytes/frame", TEST_FRAME_COUNT,
            static_cast<int>(AimUart::FRAME_SIZE));

  for (int i = 0; i < TEST_FRAME_COUNT; ++i) {
    AimUart::encode(result, testSeq++, frame);

    int written = writeCompleteFrame(serial, frame, AimUart::FRAME_SIZE)
                      ? static_cast<int>(AimUart::FRAME_SIZE)
                      : -1;

    if (written == static_cast<int>(AimUart::FRAME_SIZE)) {
      ++successCount;
      log::info("UART test frame %d/%d sent, bytes=%d", i + 1, TEST_FRAME_COUNT,
                written);
    } else {
      log::error("UART test frame %d/%d failed: written=%d, expected=%d", i + 1,
                 TEST_FRAME_COUNT, written,
                 static_cast<int>(AimUart::FRAME_SIZE));
    }

    // 每帧间隔 100ms，方便 STM32 接收和调试
    time::sleep_ms(100);
  }

  log::info("UART SELF TEST RESULT: %d/%d frames written", successCount,
            TEST_FRAME_COUNT);
  log::info("========== UART SELF TEST END ==========");

  return successCount == TEST_FRAME_COUNT;
}

int _main(int argc, char *argv[]) {
  camera::Camera cam(640, 480, image::Format::FMT_RGB888);

  // 用于 UART 通信的序列号
  uint8_t txSeq = 0;
  // 初始化 UART
  std::vector<std::string> ports = peripheral::uart::list_devices();
  // 列出所有可用的 UART 设备
  for (const auto &port : ports) {
    log::info("UART device: %s", port.c_str());
  }

  if (ports.empty()) {
    log::error("No UART device found!");
    return -1;
  }

  for (const auto &port : ports) {
    log::info("Found UART device: %s", port.c_str());
  }

  // MaixCAM 2 UART2: U2T / U2R
  const std::string uartDevice = "/dev/ttyS2";
  peripheral::uart::UART serial(uartDevice, 115200);

  log::info("UART opened: %s", uartDevice.c_str());

  // 正式初始化 IMU、标定参数和进入识别流程前，先测试 UART。
  const char *uartTest = std::getenv("MAIX_UART_TEST");
  if (uartTest && std::string(uartTest) == "1" && !testAimUart(serial)) {
    log::error("UART self-test failed. Stop before armor detection.");
    return -1;
  }

  // ===== 手机网页视频监控 =====
  http::JpegStreamer webStreamer("0.0.0.0", 8000, 2);

  const std::string monitorHtml = R"HTML(
  <!DOCTYPE html>
  <html lang="zh-CN">
  <head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>26rmic 自瞄监控</title>
  <style>
  body {
    margin: 0;
    padding: 16px;
    background: #111827;
    color: #f3f4f6;
    font-family: sans-serif;
  }
  h2 { margin-top: 0; }
  .panel {
    max-width: 800px;
    margin: auto;
  }
  img {
    display: block;
    width: 100%;
    border-radius: 10px;
    background: #000;
  }
  .note {
    color: #9ca3af;
    font-size: 14px;
    margin-top: 12px;
  }
  </style>
  </head>
  <body>
  <div class="panel">
    <h2>26rmic 自瞄监控</h2>
    <img src="/stream" alt="等待视频流">
    <p class="note">
      当前显示 MaixCAM 2 推送的图像。
      请保持手机与设备连接在同一热点网络。
    </p>
  </div>
  </body>
  </html>
  )HTML";

  if (webStreamer.set_html(monitorHtml) != err::ERR_NONE) {
    log::error("Failed to configure web monitor page");
    return -1;
  }

  if (webStreamer.start() != err::ERR_NONE) {
    log::error("Failed to start web video streamer");
    return -1;
  }

  log::info("Web video streamer started on port 8000");

  // 自检通过后再继续初始化。
  ImuReader imuReader;

  if (!imuReader.initialize()) {
    std::printf("[IMU] Disabled: initialization failed.\n");
    // 暂时允许视觉程序继续运行。
  }

  display::Display disp;

  // 触屏对象
  touchscreen::TouchScreen ts;

  cam.skip_frames(5);

  EnemyColor selectedColor = EnemyColor::BLUE;
  bool confirmed = false;
  bool lastPressed = false;
  const char *autoColor = std::getenv("RMIC_ENEMY_COLOR");
  if (autoColor &&
      (std::string(autoColor) == "red" || std::string(autoColor) == "blue")) {
    selectedColor =
        std::string(autoColor) == "red" ? EnemyColor::RED : EnemyColor::BLUE;
    confirmed = true;
  }

  ArmorDetector detector(selectedColor);
  ArmorTracker tracker(80.0f, 5);
  AimTrackGate aimGate;
  CameraModel cameraModel;

  if (!cameraModel.load("/root/camera_intrinsics.yml")) {
    log::error("camera calibration load failed");
    return -1;
  }

  if (!cameraModel.setImageSize(640, 480)) {
    log::error("camera image size configuration failed");
    return -1;
  }

  cv::Mat pnpCameraMatrix;
  cv::Mat pnpDistCoeffs;

  if (!cameraModel.getPnPParameters(pnpCameraMatrix, pnpDistCoeffs)) {
    log::error("Failed to get PnP camera parameters");
    return -1;
  }

  log::info("PnP camera parameters loaded");

  log::info("=== RMIC26 ===");

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

    // 将与本地屏幕相同的图像推送到网页
    if (webStreamer.write(img) != err::ERR_NONE) {
      // 暂时不因网页客户端断开而停止识别程序
    }

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
  const char *captureDir = std::getenv("RMIC_CAPTURE_DIR");
  unsigned captureCount = 0;
  FILE *trace = nullptr;
  if (captureDir)
    trace = std::fopen((std::string(captureDir) + "/trace.csv").c_str(), "w");
  if (trace)
    std::fprintf(trace, "ms,detected,state,missed,x,y\n");

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
    // Physical camera is rolled 180 degrees: correct pixels exactly once,
    // before detection and before drawing upright overlays.
    cv::rotate(rgb, rgb, cv::ROTATE_180);

    // 获取当前帧的检测结果和中间调试数据

    ArmorDebugInfo debugInfo;
    std::vector<ArmorTarget> targets = detector.detect(rgb, &debugInfo);

    // PnP 仅用于调试，不参与当前瞄准控制。
    ArmorPose bestPose;
    bool pnpValid = false;

    for (const ArmorTarget &target : targets) {
      ArmorPose pose;

      std::array<cv::Point2f, 4> imagePoints;
      std::copy(std::begin(target.corners), std::end(target.corners),
                imagePoints.begin());

      if (!solveSmallArmorPnP(imagePoints, pnpCameraMatrix, pnpDistCoeffs,
                              pose)) {
        continue;
      }

      // 多个候选目标时，暂时选择重投影误差最小的结果。
      if (!pnpValid || pose.reprojectionError < bestPose.reprojectionError) {
        bestPose = pose;
        pnpValid = true;
      }
    }

    const TrackedTarget &tracked = tracker.update(targets);

    if (captureDir && captureCount < 150) {
      char name[256];
      std::snprintf(name, sizeof(name), "%s/%04u.png", captureDir,
                    captureCount++);
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      cv::imwrite(name, bgr, {cv::IMWRITE_PNG_COMPRESSION, 1});
      if (trace) {
        std::fprintf(trace, "%llu,%zu,%d,%d,%.3f,%.3f\n",
                     (unsigned long long)time::ticks_ms(), targets.size(),
                     (int)tracked.state, tracked.missedFrames, tracked.center.x,
                     tracked.center.y);
        std::fflush(trace);
      }
    }
    // 统一保存本帧的瞄准结果
    AimResult aimResult;

    // 绿十字
    cv::drawMarker(rgb, cameraModel.mountedPrincipalPoint(),
                   cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 30, 3);

    if (pnpValid) {
      const std::string distanceText =
          "PnP Distance: " + std::to_string(bestPose.distance) + " m";

      const std::string errorText =
          "PnP Reproj: " + std::to_string(bestPose.reprojectionError) + " px";

      cv::putText(rgb, distanceText, cv::Point(15, 280),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);

      cv::putText(rgb, errorText, cv::Point(15, 305), cv::FONT_HERSHEY_SIMPLEX,
                  0.55, cv::Scalar(0, 255, 255), 2);
    } else {
      cv::putText(rgb, "PnP: NO VALID POSE", cv::Point(15, 280),
                  cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);
    }

    cv::Point2f aimCenter;
    const uint32_t frameNow = static_cast<uint32_t>(time::ticks_ms());
    const bool aimValid = aimGate.update(tracked, frameNow, aimCenter);

    if (!aimValid) {
      std::printf("[AimGate] rejected: %s\n",
                  aimRejectReasonName(aimGate.reason()));
    }

    static uint32_t statMs = frameNow;
    static unsigned statN = 0, statDetected = 0, statMeasured = 0, statHeld = 0,
                    statAim = 0;
    ++statN;
    statDetected += !targets.empty();
    statMeasured += tracked.state == TrackState::TRACKING;
    statHeld += aimValid && aimGate.held();
    statAim += aimValid;
    if (uint32_t(frameNow - statMs) >= 1000U) {
      log::info("[TRACK] frames=%u detected=%u measured=%u held=%u aim=%u",
                statN, statDetected, statMeasured, statHeld, statAim);
      statN = statDetected = statMeasured = statHeld = statAim = 0;
      statMs = frameNow;
    }

    if (aimValid) {
      const cv::Point2f angle = cameraModel.mountedPixelToAngle(aimCenter);

      const bool angleValid =
          std::isfinite(angle.x) && std::isfinite(angle.y) &&
          std::fabs(angle.x) <= 90.0f && std::fabs(angle.y) <= 90.0f;

      if (angleValid) {
        aimResult.targetFound = true;
        aimResult.yawDeg = angle.x;
        aimResult.pitchDeg = angle.y;
        aimResult.centerX = aimCenter.x;
        aimResult.centerY = aimCenter.y;

        cv::putText(rgb, aimGate.held() ? "Aim: HOLD <=60ms" : "Aim: MEASURED",
                    cv::Point(15, 250), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 0), 1);

        cv::putText(rgb, "Yaw:" + std::to_string(angle.x), cv::Point(15, 190),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);

        cv::putText(rgb, "Pitch:" + std::to_string(angle.y), cv::Point(15, 220),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
      } else {
        cv::putText(rgb, "Aim: INVALID ANGLE", cv::Point(15, 250),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 0), 1);
      }
    }

    // 读取并打印 IMU 数据
    //  读取 IMU，并把数据放入本帧 UART 发送结果。
    static uint64_t lastImuLogMs = 0;

    if (imuReader.isInitialized()) {
      ImuReader::Sample imuSample;

      if (imuReader.read(imuSample)) {
        aimResult.imu.valid = true;
        aimResult.imu.attitudeValid = imuSample.attitudeValid;
        aimResult.imu.dtOutOfRange = imuSample.dtOutOfRange;

        aimResult.imu.rollDeg = imuSample.rollDeg;
        aimResult.imu.pitchDeg = imuSample.pitchDeg;
        aimResult.imu.yawDeg = imuSample.yawDeg;

        aimResult.imu.gx = imuSample.gx;
        aimResult.imu.gy = imuSample.gy;
        aimResult.imu.gz = imuSample.gz;

        aimResult.imu.ax = imuSample.ax;
        aimResult.imu.ay = imuSample.ay;
        aimResult.imu.az = imuSample.az;

        aimResult.imu.dtSeconds = imuSample.dtSeconds;

        if (imuSample.timestampMs - lastImuLogMs >= 200) {
          lastImuLogMs = imuSample.timestampMs;

          std::printf("[IMU] dt=%.4f s attitude=(%.2f, %.2f, %.2f) "
                      "gyro=(%.2f, %.2f, %.2f) "
                      "acc=(%.3f, %.3f, %.3f) g\n",
                      imuSample.dtSeconds, imuSample.rollDeg,
                      imuSample.pitchDeg, imuSample.yawDeg, imuSample.gx,
                      imuSample.gy, imuSample.gz, imuSample.ax, imuSample.ay,
                      imuSample.az);
        }
      }
    }

    uint8_t txFrame[AimUart::FRAME_SIZE];
    AimUart::encode(aimResult, txSeq++, txFrame);
    int written = writeCompleteFrame(serial, txFrame, AimUart::FRAME_SIZE)
                      ? static_cast<int>(AimUart::FRAME_SIZE)
                      : -1;

    if (written != AimUart::FRAME_SIZE) {
      log::error("UART write failed or incomplete: %d", written);
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

    // 推送已经绘制好识别框、跟踪状态和调试信息的图像
    if (webStreamer.write(img) != err::ERR_NONE) {
      // 网络推流失败不应阻塞自瞄主流程
    }

    disp.show(*img);
    delete img;
  }

  return 0;
}

int main(int argc, char *argv[]) {
  // Read-only runtime checks: neither branch opens UART or commands actuators.
  if (argc > 1 && std::string(argv[1]) == "--check-mount") {
    CameraModel model;
    if (!model.load("/root/camera_intrinsics.yml") ||
        !model.setImageSize(640, 480))
      return 2;
    const auto center = model.mountedPrincipalPoint();
    const auto zero = model.mountedPixelToAngle(center);
    const auto rightUp =
        model.mountedPixelToAngle(center + cv::Point2f(80, -60));
    if (std::fabs(zero.x) > 0.001f || std::fabs(zero.y) > 0.001f ||
        rightUp.x <= 0 || rightUp.y <= 0)
      return 3;
    std::printf(
        "MOUNT180 CHECK PASS: right-up yaw=%.3f pitch=%.3f; UART not opened\n",
        rightUp.x, rightUp.y);
    return 0;
  }
  if (argc > 1 && std::string(argv[1]) == "--preview-check") {
    camera::Camera camera(640, 480, image::Format::FMT_RGB888);
    display::Display display;
    for (int n = 0; n < 30; ++n) {
      std::unique_ptr<image::Image> img(camera.read());
      if (!img || img->format() != image::Format::FMT_RGB888)
        return 4;
      cv::Mat rgb(img->height(), img->width(), CV_8UC3, img->data());
      cv::rotate(rgb, rgb, cv::ROTATE_180);
      display.show(*img);
    }
    std::puts("MOUNT180 CAMERA/DISPLAY CHECK PASS: 30 upright frames; UART not "
              "opened");
    return 0;
  }

  sys::register_default_signal_handle();
  CATCH_EXCEPTION_RUN_RETURN(_main, -1, argc, argv);
}