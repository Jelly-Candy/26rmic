#pragma once

struct ImuTelemetry
{
    bool valid = false;
    bool attitudeValid = false;
    bool dtOutOfRange = false;

    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;

    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;

    float dtSeconds = 0.0f;
};

struct AimResult
{
    bool targetFound = false;

    float yawDeg = 0.0f;
    float pitchDeg = 0.0f;

    float centerX = 0.0f;
    float centerY = 0.0f;

    int targetId = -1;

    ImuTelemetry imu;
};
