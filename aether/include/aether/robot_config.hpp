#ifndef _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
#define _AETHER_INCLUDE_ROBOT_CONFIG_HPP_

#include <cstdint>

#include "Eigen/Core"

constexpr float PI_F = 3.14159265358979f;
constexpr float PI_2_F = 1.57079632679490f;

static constexpr size_t NUM_TOFS = 6;
template <typename T> struct TofsData {
    const T right_side;
    const T right_diag;
    const T right_front;
    const T left_front;
    const T left_diag;
    const T left_side;

    const T &operator[](size_t i) const {
        switch (i) {
        case 0:
            return right_side;
        case 1:
            return right_diag;
        case 2:
            return right_front;
        case 3:
            return left_front;
        case 4:
            return left_diag;
        case 5:
            return left_side;
        default:
            return right_side;
        }
    }
};

struct UncertainPose {
    const float x;
    const float y;
    const float yaw;
    const float std;
};
using TofsReadings = TofsData<UncertainPose>;

struct Pose {
    float x;
    float y;
    float yaw;

    constexpr UncertainPose operator*(const UncertainPose &other) const {
        return {
            x + other.x * cosf(yaw) - other.y * sinf(yaw),
            y + other.x * sinf(yaw) + other.y * cosf(yaw),
            yaw + other.yaw,
            other.std,
        };
    }
};
using TofsPoses = TofsData<Pose>;

struct RobotConfig {
    const float robot_length;
    const float motors_offset;
    const float robot_width;
    const float wheel_radius;
    const float wheel_base;

    const float wall_width;
    const float cell_size;

    const float starting_x;
    const float starting_y;
    const float starting_yaw;

    const uint16_t freq_imu_enc;
    const uint16_t freq_tofs;

    const TofsPoses tofs_poses;

    constexpr float dt_imu_enc() const { return 1.0f / freq_imu_enc; }
    constexpr float dt_tofs() const { return 1.0f / freq_tofs; }
};

#endif // _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
