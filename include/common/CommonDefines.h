#pragma once

#include <QtGlobal>
#include <cmath>
#include <vector>

inline float calculateMetersDistance(
        const float pos1_x,
        const float pos1_y,
        const float pos1_z,
        const float pos2_x,
        const float pos2_y,
        const float pos2_z)
{
    return sqrtf(powf(pos2_x - pos1_x, 2.f) + powf(pos2_y - pos1_y, 2.f) + powf(pos2_z - pos1_z, 2.f));
}

inline std::vector<double> ToEulerAngles(double qw, double qx, double qy, double qz)
{
    std::vector<double> quat{qx, qy, qz, qw};

    std::vector<double> eulerAngles;
    // roll (x-axis rotation)
    const auto          sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2]);
    const auto          cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1]);
    eulerAngles.push_back(std::atan2(sinr_cosp, cosr_cosp));

    // pitch (y-axis rotation)
    const auto sinp = 2 * (quat[3] * quat[1] - quat[2] * quat[0]);
    if (std::abs(sinp) >= 1) {
        eulerAngles.push_back(std::copysign(M_PI / 2, sinp)); // use 90 degrees if out of range
    } else {
        eulerAngles.push_back(std::asin(sinp));
    }

    // yaw (z-axis rotation)
    const auto siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1]);
    const auto cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    eulerAngles.push_back(std::atan2(siny_cosp, cosy_cosp));

    return eulerAngles;
}

inline std::vector<double> ToEulerAnglesDeg(double qw, double qx, double qy, double qz)
{
    auto                eaRad = ToEulerAngles(qw, qx, qy, qz);
    std::vector<double> eaDeg;
    for (auto& angle : eaRad) {
        eaDeg.push_back(angle * 180. / M_PI);
    }
    return eaDeg;
}

/// \note Asumming that: input={roll, pitch, yaw} and quat={w, x, y, z}
inline std::vector<float> ToQuaternion(float roll, float pitch, float yaw)
{
    float cy = std::cos(yaw * 0.5);
    float sy = std::sin(yaw * 0.5);
    float cp = std::cos(pitch * 0.5);
    float sp = std::sin(pitch * 0.5);
    float cr = std::cos(roll * 0.5);
    float sr = std::sin(roll * 0.5);

    std::vector<float> quat{1.0f, 0.0f, 0.0f, 0.0f};
    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;

    return quat;
}

/// \note Asumming that: input={roll, pitch, yaw} and quat={w, x, y, z}
inline std::vector<float> degToQuaternion(float roll, float pitch, float yaw)
{
    return ToQuaternion(roll * (M_PI / 180.0), pitch * (M_PI / 180.0), yaw * (M_PI / 180.0));
}