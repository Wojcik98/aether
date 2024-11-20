#ifndef _AETHER_INCLUDE_ASYNC_LOCALIZATION_HPP_
#define _AETHER_INCLUDE_ASYNC_LOCALIZATION_HPP_

#include <cstdint>

// Time in milliseconds.
using Time = uint32_t;

struct Pose {
    float x;
    float y;
    float yaw;
};

// We currently only use z-axis from both accelerometer and gyroscope.
// Using other acceleration axes would require more complex calculations.
// Using other gyroscope axes is not needed in 2D.
struct ImuData {
    float acc_z;
    float om_z;
};

struct EncoderData {
    float om_l;
    float om_r;
};

struct TofsData {
    float right_side;
    float right_diag;
    float right_front;
    float left_front;
    float left_diag;
    float left_side;
};

// Interface for localization classes.
class Localization {
public:
    virtual ~Localization() = default;

    virtual void reset(Time time) = 0;

    virtual Pose get_latest_pose() = 0;

    virtual bool collision_detected() = 0;

    virtual void imu_update(Time time, const ImuData &imu_data) = 0;

    virtual void encoder_update(Time time, const EncoderData &encoder_data) = 0;

    virtual void tofs_update(Time time, const TofsData &tofs_data) = 0;
};

#endif // _AETHER_INCLUDE_ASYNC_LOCALIZATION_HPP_
