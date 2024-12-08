#ifndef _AETHER_INCLUDE_LOCALIZATION_INTERFACE_HPP_
#define _AETHER_INCLUDE_LOCALIZATION_INTERFACE_HPP_

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
// Uses the CRTP pattern to allow static polymorphism.
template <class LocalizationImpl> class LocalizationInterface {
public:
    void reset(Time time) {
        static_cast<LocalizationImpl *>(this)->reset(time);
    }

    Pose get_latest_pose() {
        return static_cast<LocalizationImpl *>(this)->get_latest_pose();
    }

    bool collision_detected() {
        return static_cast<LocalizationImpl *>(this)->collision_detected();
    }

    void imu_enc_update(Time time, const ImuData &imu_data,
                        const EncoderData &encoder_data) {
        static_cast<LocalizationImpl *>(this)->imu_enc_update(time, imu_data,
                                                              encoder_data);
    }

    void tofs_update(Time time, const TofsReadings &tofs_data) {
        static_cast<LocalizationImpl *>(this)->tofs_update(time, tofs_data);
    }
};

#endif // _AETHER_INCLUDE_LOCALIZATION_INTERFACE_HPP_
