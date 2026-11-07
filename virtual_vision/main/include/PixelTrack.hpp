#pragma once
#include <opencv2/core.hpp>
#include "ColorContourDetector.hpp"
enum class TrackState
{
    LOST,
    TRACKING,
    TEMP_LOST
};

struct TrackedTarget
{
    cv::Point2f center{0.0f, 0.0f};
    cv::Point2f observedCenter{0.0f, 0.0f};
    cv::Point2f velocity{0.0f, 0.0f};
    cv::Rect bbox;
    TrackState state = TrackState::LOST;
    int missedFrames = 0;
    // Exact current detection used for this track; -1 for predicted/lost frames.
    int detectionIndex = -1;
};

