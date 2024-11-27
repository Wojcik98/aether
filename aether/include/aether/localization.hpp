#ifndef _AETHER_INCLUDE_ASYNC_LOCALIZATION_HPP_
#define _AETHER_INCLUDE_ASYNC_LOCALIZATION_HPP_

#include <cstdint>

#include "aether/robot_config.hpp"

// Time in milliseconds.
using Time = uint32_t;

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

// Interface for localization classes.
// Assumes that the IMU and encoder frequencies are equal.
class Localization {
public:
    virtual ~Localization() = default;

    virtual void reset(Time time) = 0;

    virtual Pose get_latest_pose() = 0;

    virtual bool collision_detected() = 0;

    virtual void imu_enc_update(Time time, const ImuData &imu_data,
                                const EncoderData &encoder_data) = 0;

    virtual void tofs_update(Time time, const TofsReadings &tofs_data) = 0;
};

#endif // _AETHER_INCLUDE_ASYNC_LOCALIZATION_HPP_