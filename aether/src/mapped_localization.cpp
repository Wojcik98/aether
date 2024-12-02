#include "aether/mapped_localization.hpp"

#include <cmath>

#include "aether/robot_config.hpp"

MappedLocalization::MappedLocalization(const MapConfident &map)
    : map_(map), collision_detected_(false) {
    reset_particles();
}

void MappedLocalization::reset(Time time) {
    last_predict_ = time;
    collision_detected_ = false;
    reset_particles();
}

void MappedLocalization::imu_enc_update(Time time, const ImuData &imu_data,
                                        const EncoderData &encoder_data) {
    // TODO: save data to buffer and process it from another thread.
    // When we run updates from ToF, they can take a while and we don't want to
    // miss any data.
    if (imu_data.acc_z > -0.5f) { // TODO get threshold and direction
        collision_detected_ = true;
    }

    motion_model(encoder_data, imu_data);
    last_predict_ = time;
}

void MappedLocalization::motion_model(const EncoderData &encoder_data,
                                      const ImuData &imu_data) {
    constexpr float dt = DT_IMU_ENC;
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

void MappedLocalization::tofs_update(Time time, const TofsReadings &tofs_data) {
    (void)time;
    for (auto &particle : particles_) {
        particle.weight = particle_probability(particle, tofs_data);
    }
}

float MappedLocalization::particle_probability(const Particle &particle,
                                               const TofsReadings &tofs_data) {
    std::array<float, NUM_TOFS> probs;
    for (size_t i = NUM_TOFS; i > 0; --i) {
        const auto &tof = tofs_data[i];
        float x = particle.state.x + tof.x * cosf(particle.state.yaw);
        float y = particle.state.y + tof.x * sinf(particle.state.yaw);
        probs[i] = map_.probability(x, y, tof.std);
    }

    return 0.0f;
}

void MappedLocalization::reset_particles() {
    for (auto &particle : particles_) {
        particle.state.x = STARTING_X;
        particle.state.y = STARTING_Y;
        particle.state.yaw = STARTING_YAW;
        particle.state.vx = 0.0f;
        particle.state.omega = 0.0f;
        particle.weight = 1.0f / NUM_PARTICLES;
    }
}

Pose MappedLocalization::get_latest_pose() {
    Pose pose{0.0f, 0.0f, 0.0f};

    // TODO: weighted average?
    // TODO: noise
    for (const auto &particle : particles_) {
        pose.x += particle.state.x;
        pose.y += particle.state.y;
        pose.yaw += particle.state.yaw;
    }

    pose.x /= NUM_PARTICLES;
    pose.y /= NUM_PARTICLES;
    pose.yaw /= NUM_PARTICLES;

    return pose;
}
