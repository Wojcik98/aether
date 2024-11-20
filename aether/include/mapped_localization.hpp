#ifndef _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_
#define _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_

#include <array>
#include <cstdint>

#include "localization.hpp"
#include "map_confident.hpp"

// Localization class that uses finished map to localize the robot.
// Uses particle filter algorithm.
// Assumes that the robot is always in the map.
// Assumes that the IMU frequency is higher than encoder and TOF sensors.
class MappedLocalization : public Localization {
public:
    static constexpr uint16_t NUM_PARTICLES = 10;

    MappedLocalization(uint16_t freq_imu, uint16_t freq_enc, uint16_t freq_tofs,
                       const MapConfident &map);

    ~MappedLocalization() = default;

    void reset(Time time) override;

    Pose get_latest_pose() override { return latest_pose_; }

    bool collision_detected() override { return collision_detected_; }

    void imu_update(Time time, const ImuData &imu_data) override;

    void encoder_update(Time time, const EncoderData &encoder_data) override;

    void tofs_update(Time time, const TofsData &tofs_data) override;

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

    const uint16_t freq_imu_;
    const uint16_t freq_enc_;
    const uint16_t freq_tofs_;
    const float dt_enc_;

    const MapConfident &map_;

    std::array<Particle, NUM_PARTICLES> particles_;

    Time last_predict_;
    ImuData last_imu_data_;
    bool imu_received_ = false;

    bool collision_detected_;

    Pose latest_pose_;

    void motion_model(float dt, const EncoderData &encoder_data,
                      const ImuData &imu_data);

    void reset_particles();
    void update_latest_pose();
};

#endif // _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_
