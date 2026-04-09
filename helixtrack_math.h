#pragma once

#include <algorithm>
#include <cmath>

namespace helixtrack {

// Shared scalar helpers used by the phase tracker and pose objective:
// wrapped phase arithmetic, smooth soft band barriers, and RPM conversions.
constexpr float kPI = 3.14159265358979323846f;
constexpr float kTwoPI = 6.28318530717958647692f;
constexpr double kPiD = 3.14159265358979323846;
constexpr double kTwoPiD = 6.28318530717958647692;

inline float wrap_pi(float angle) {
    angle = std::fmod(angle + kPI, kTwoPI);
    if (angle < 0.f) angle += kTwoPI;
    return angle - kPI;
}

inline float wrap_2pi(float angle) {
    angle = std::fmod(angle, kTwoPI);
    if (angle < 0.f) angle += kTwoPI;
    return angle;
}

inline float sigmoid(float z) {
    return 1.f / (1.f + std::exp(-z));
}

inline float softplus_tau(float x, float tau) {
    const float z = x / std::max(1e-6f, tau);
    if (z > 12.f) return x;
    if (z < -12.f) return 0.f;
    return tau * std::log1p(std::exp(z));
}

inline double rpm_to_rad_per_sec(double rpm) {
    return rpm * kTwoPiD / 60.0;
}

inline double rad_per_sec_to_rpm(double omega) {
    return omega * 60.0 / kTwoPiD;
}

}  // namespace helixtrack
