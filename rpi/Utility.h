#pragma once
#include <ctime>
#include <cstdint>
#include <cmath>

inline uint64_t now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000 + static_cast<uint64_t>(ts.tv_nsec) / 1000000;
}

// Utility.h — uint32_t 반환 버리고 float 유지 + 음수 정규화
inline float rad_to_deg(float rad) {
    float deg = rad * 180.0f / (float)M_PI;
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}