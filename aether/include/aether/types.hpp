#ifndef _AETHER_TYPES_HPP_
#define _AETHER_TYPES_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <tuple>

#include "aether/robot_config.hpp"

constexpr float PI_F = 3.14159265358979f;
constexpr float PI_2_F = 1.57079632679490f;

struct Twist {
    float vx;
    float omega;
};

struct FullState {
    float x;
    float y;
    float yaw;

    float vx;
    float omega;

    Pose operator*(const Pose &pose) const {
        // transform pose from local to global frame
        // TODO: potentially optimize, as I'm mostly using it with constexpr
        // poses, could precompute cosf and sinf
        return {
            x + pose.x * cosf(yaw) - pose.y * sinf(yaw),
            y + pose.x * sinf(yaw) + pose.y * cosf(yaw),
            yaw + pose.yaw,
        };
    }

    FullState operator-(const FullState &other) const {
        return {x - other.x, y - other.y, yaw - other.yaw, vx - other.vx,
                omega - other.omega};
    }

    Pose to_pose() const { return {x, y, yaw}; }
};

struct TofReading {
    const float dist;
    const float std;
};
using TofsReadings = TofsData<TofReading>;

using CellCoords = std::tuple<int32_t, int32_t>;
using RelCoords = std::tuple<float, float>;

struct CellWalls {
    float north;
    float west;
    float south;
    float east;
};

using Path = std::array<CellCoords, MAX_PATH_LENGTH>;

#endif // _AETHER_TYPES_HPP_
