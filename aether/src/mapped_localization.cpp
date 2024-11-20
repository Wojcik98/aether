#include "mapped_localization.hpp"

#include <cmath>

#include "robot_configs.hpp"

MappedLocalization::MappedLocalization(uint16_t freq_imu, uint16_t freq_enc,
                                       uint16_t freq_tofs,
                                       const MapConfident &map)
    : freq_imu_(freq_imu), freq_enc_(freq_enc), freq_tofs_(freq_tofs),
      dt_enc_(1.0f / freq_enc), map_(map), collision_detected_(false) {
    reset_particles();
}

void MappedLocalization::reset(Time time) {
    last_predict_ = time;
    collision_detected_ = false;
    reset_particles();
}

void MappedLocalization::imu_update(Time time, const ImuData &imu_data) {
    if (imu_data.acc_z > -0.5f) { // TODO get threshold and direction
        collision_detected_ = true;
    }

    last_imu_data_ = imu_data;
}

void MappedLocalization::encoder_update(Time time,
                                        const EncoderData &encoder_data) {
    if (!imu_received_) {
        return;
    }

    motion_model(dt_enc_, encoder_data, last_imu_data_);

    last_predict_ = time;
}

void MappedLocalization::motion_model(float dt, const EncoderData &encoder_data,
                                      const ImuData &imu_data) {
    // TODO: merge encoder and imu in omega?
    float omega = imu_data.om_z;
    // TODO: right formula?
    float vx = (encoder_data.om_l + encoder_data.om_r) * WHEEL_RADIUS / 2.0f;

    // differential drive model
    // TODO: add noise
    for (auto &particle : particles_) {
        particle.state.x += particle.state.vx * dt * cosf(particle.state.yaw);
        particle.state.y += particle.state.vx * dt * sinf(particle.state.yaw);
        particle.state.yaw += particle.state.omega * dt;

        particle.state.vx = vx;
        particle.state.omega = omega;
    }
}

void MappedLocalization::tofs_update(Time time, const TofsData &tofs_data) {}

void MappedLocalization::reset_particles() {
    for (auto &particle : particles_) {
        particle.state.x = STARTING_X;
        particle.state.y = STARTING_Y;
        particle.state.yaw = STARTING_YAW;
        particle.state.vx = 0.0f;
        particle.state.omega = 0.0f;
        particle.weight = 1.0f / NUM_PARTICLES;
    }
    update_latest_pose();
}

void MappedLocalization::update_latest_pose() {
    latest_pose_.x = 0.0f;
    latest_pose_.y = 0.0f;
    latest_pose_.yaw = 0.0f;

    // TODO: weighted average?
    // TODO: noise
    for (const auto &particle : particles_) {
        latest_pose_.x += particle.state.x;
        latest_pose_.y += particle.state.y;
        latest_pose_.yaw += particle.state.yaw;
    }

    latest_pose_.x /= NUM_PARTICLES;
    latest_pose_.y /= NUM_PARTICLES;
    latest_pose_.yaw /= NUM_PARTICLES;
}
