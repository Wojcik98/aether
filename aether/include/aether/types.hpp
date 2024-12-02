#ifndef _AETHER_TYPES_HPP_
#define _AETHER_TYPES_HPP_

#include <cstdint>

constexpr float PI_F = 3.14159265358979f;
constexpr float PI_2_F = 1.57079632679490f;

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

struct Pose {
    float x;
    float y;
    float yaw;
};
using TofsPoses = TofsData<Pose>;

struct UncertainPose {
    const float x;
    const float y;
    const float yaw;
    const float std;
};
using TofsReadings = TofsData<UncertainPose>;

#endif // _AETHER_TYPES_HPP_
