#ifndef _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_
#define _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_

#include <array>
#include <cstdint>

#include "aether/localization.hpp"
#include "aether/map_confident.hpp"
#include "aether/robot_config.hpp"

// Localization class that uses finished map to localize the robot.
// Uses particle filter algorithm.
// Assumes that the robot is always in the map.
// Assumes that the IMU and encoder frequencies are higer than ToF.
class MappedLocalization : public Localization {
public:
    static constexpr uint16_t NUM_PARTICLES = 10;

    MappedLocalization(const MapConfident &map);

    void reset(Time time) override;

    Pose get_latest_pose() override; // { return latest_pose_; }

    bool collision_detected() override { return collision_detected_; }

    void imu_enc_update(Time time, const ImuData &imu_data,
                        const EncoderData &encoder_data) override;

    void tofs_update(Time time, const TofsReadings &tofs_data) override;

private:
    struct State {
        float x;
        float y;
        float yaw;

        float vx;
        float omega;
    };

    struct Particle {
        State state;
        float weight;
    };

    const MapConfident &map_;

    std::array<Particle, NUM_PARTICLES> particles_;

    Time last_predict_;

    bool collision_detected_;

    void motion_model(const EncoderData &encoder_data, const ImuData &imu_data);
    float particle_probability(const Particle &particle,
                               const TofsReadings &tofs_data);

    void reset_particles();
};

#endif // _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_
