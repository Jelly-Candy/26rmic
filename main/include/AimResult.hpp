#pragma once

struct AimResult
{
    bool targetFound = false;

    float yawDeg = 0.0f;
    float pitchDeg = 0.0f;

    float centerX = 0.0f;
    float centerY = 0.0f;

    int targetId = -1;
};