#ifndef _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_
#define _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "aether/localization_interface.hpp"
#include "aether/map_confident.hpp"
#include "aether/random.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

struct Particle {
    FullState state;
    float weight;
};

// Localization class that uses finished map to localize the robot.
// Uses particle filter algorithm.
// Assumes that the robot is always in the map.
// Assumes that the IMU and encoder frequencies are higer than ToF.
template <class MapImpl>
class MappedLocalization
    : public LocalizationInterface<MappedLocalization<MapImpl>> {
public:
    MappedLocalization(const MapConfident<MapImpl> &map)
        : map_(map), collision_detected_(false) {
        reset_particles();
    }

    void reset() {
        collision_detected_ = false;
        reset_particles();
    }

    Pose get_latest_pose() {
        Pose pose{0.0f, 0.0f, 0.0f};

        for (const auto &particle : particles_) {
            pose.x += particle.state.x * particle.weight;
            pose.y += particle.state.y * particle.weight;
            pose.yaw += particle.state.yaw * particle.weight;
        }

        return pose;
    }

    std::array<Particle, NUM_PARTICLES> get_particles() { return particles_; }

    bool collision_detected() { return collision_detected_; }

    void imu_enc_update(const ImuData &imu_data,
                        const EncoderData &encoder_data) {
        // TODO: save data to buffer and process it from another thread.
        // When we run updates from ToF, they can take a while and we don't want
        // to miss any data.
        if (imu_data.acc_z > -0.5f) { // TODO get threshold and direction
            collision_detected_ = true;
        }

        motion_model(encoder_data, imu_data);
    }

    void tofs_update(const TofsReadings &tofs_data) {
        float sum_weights = 0.0f;
        float sum_weights_sq = 0.0f;
        for (auto &particle : particles_) {
            particle.weight = particle_probability(particle, tofs_data);
            sum_weights += particle.weight;
            sum_weights_sq += particle.weight * particle.weight;
        }

        for (auto &particle : particles_) {
            particle.weight /= sum_weights;
        }

        float num_effective_particles = 1.0f / sum_weights_sq;
        if (num_effective_particles < NUM_EFF_PARTICLES_THRESHOLD) {
            resample_particles();
        }

        if (rand_gen.rand_left() < 0.1f) {
            rand_gen.regenerate();
        }
    }

    // void set_random_seed(uint32_t seed) { gen.seed(seed); }
    void set_random_seed(uint32_t seed) { (void)seed; }

private:
    RandomGenerator rand_gen{42};
    float noise(float std) { return rand_gen.normal(0.0f, std); }
    float rand_uniform(float a, float b) { return rand_gen.uniform(a, b); }

    const MapConfident<MapImpl> &map_;

    std::array<Particle, NUM_PARTICLES> particles_;

    bool collision_detected_;

    void motion_model(const EncoderData &encoder_data,
                      const ImuData &imu_data) {
        constexpr float MODEL_NOISE_X = 0.1f;
        constexpr float MODEL_NOISE_Y = 0.05f;
        constexpr float MODEL_NOISE_YAW = 0.5f;
        constexpr float dt = DT_IMU_ENC;
        // TODO: merge encoder and imu in omega?
        float omega = imu_data.om_z;
        float vx =
            (encoder_data.om_l + encoder_data.om_r) * WHEEL_RADIUS / 2.0f;

        // differential drive model
        for (auto &particle : particles_) {
            particle.state.vx = vx;
            particle.state.omega = omega;

            particle.state.x += (particle.state.vx * cosf(particle.state.yaw) +
                                 noise(MODEL_NOISE_X)) *
                                dt;
            particle.state.y += (particle.state.vx * sinf(particle.state.yaw) +
                                 noise(MODEL_NOISE_Y)) *
                                dt;
            particle.state.yaw +=
                (particle.state.omega + noise(MODEL_NOISE_YAW)) * dt;
        }
    }

    float particle_probability(const Particle &particle,
                               const TofsReadings &tofs_data) {
        constexpr float MIN_PROB = 0.01f;
        float sum = 0.0f;
#pragma unroll
        for (size_t i = 0; i < NUM_TOFS; ++i) {
            const auto &tof = tofs_data[i];

            float dist_pred = predict_tof(particle, i);
            if (std::isinf(tof.dist) && std::isinf(dist_pred)) {
                // both out of range
                sum += 0.0f;
            } else {
                // works even if one of them is inf, because it will add inf to
                // sum
                sum += (tof.dist - dist_pred) * (tof.dist - dist_pred) /
                       (tof.std * tof.std);
            }
        }

        return std::max(MIN_PROB, expf(-sum / 2.0f));
    }

    float predict_tof(const Particle &particle, size_t tof_idx) const {
        constexpr float ZERO_THRESH = 1e-4f;

        const auto &tof = TOF_POSES[tof_idx];
        auto tof_g = particle.state * tof;
        float yaw = tof_g.yaw;
        if (yaw < 0.0f) {
            yaw += 2.0f * PI_F;
        }

        // line equation: ax + by + c = 0
        float a = sinf(yaw);
        float b = -cosf(yaw);
        float c = -a * tof_g.x - b * tof_g.y;

        float dir_x = (PI_2_F <= yaw && yaw < 3 * PI_2_F) ? -1.0f : 1.0f;
        float dir_y = (yaw >= PI_F) ? -1.0f : 1.0f;
        float closest_dist = std::numeric_limits<float>::infinity();

        auto [x_cell, y_cell] =
            MapInterface<MapImpl>::get_cell_coords(tof_g.x, tof_g.y);
        const float start_x =
            (dir_x > 0) ? (x_cell + 1) * CELL_SIZE : x_cell * CELL_SIZE;
        const float start_y =
            (dir_y > 0) ? (y_cell + 1) * CELL_SIZE : y_cell * CELL_SIZE;

        // first, sweep in x direction
        if (abs(b) > ZERO_THRESH) {
            for (uint32_t i = 0; i <= TOF_RANGE_CELLS; ++i) {
                const float x =
                    start_x + dir_x * (i * CELL_SIZE - WALL_WIDTH / 2.0f);
                const float y = (-a * x - c) / b;
                if (map_.is_out_of_bounds(x, y)) {
                    break;
                }

                if (map_.is_obstacle(x, y)) {
                    closest_dist = sqrtf((tof_g.x - x) * (tof_g.x - x) +
                                         (tof_g.y - y) * (tof_g.y - y));
                    break;
                }
            }
        }

        // then, sweep in y direction
        if (abs(a) > ZERO_THRESH) {
            for (uint32_t i = 0; i <= TOF_RANGE_CELLS; ++i) {
                const float y =
                    start_y + dir_y * (i * CELL_SIZE - WALL_WIDTH / 2.0f);
                const float x = (-b * y - c) / a;
                if (map_.is_out_of_bounds(x, y)) {
                    break;
                }

                if (map_.is_obstacle(x, y)) {
                    float dist = sqrtf((tof_g.x - x) * (tof_g.x - x) +
                                       (tof_g.y - y) * (tof_g.y - y));
                    if (dist < closest_dist) {
                        closest_dist = dist;
                    }
                    break;
                }
            }
        }

        return closest_dist;
    }

    void resample_particles() {
        // use low variance resampling
        // weights are already normalized
        std::array<Particle, NUM_PARTICLES> new_particles;
        float r = rand_uniform(0.0f, 1.0f / NUM_PARTICLES);
        float c = particles_[0].weight;
        uint32_t i = 0;
        for (uint32_t m = 0; m < NUM_PARTICLES; ++m) {
            float u = r + m * 1.0f / NUM_PARTICLES;
            while (u > c) {
                i++;
                c += particles_[i].weight;
            }
            new_particles[m] = particles_[i];
            // new_particles[m].weight = 1.0f / NUM_PARTICLES;
            // new_particles[m] = particles_[m];
        }

        particles_ = new_particles;

        // reset weights
        for (auto &particle : particles_) {
            particle.weight = 1.0f / NUM_PARTICLES;
        }
    }

    void reset_particles() {
        for (auto &particle : particles_) {
            particle.state.x = STARTING_X;
            particle.state.y = STARTING_Y;
            particle.state.yaw = STARTING_YAW;
            particle.state.vx = 0.0f;
            particle.state.omega = 0.0f;
            particle.weight = 1.0f / NUM_PARTICLES;
        }
    }
};

#endif // _AETHER_INCLUDE_MAPPED_LOCALIZATION_HPP_
