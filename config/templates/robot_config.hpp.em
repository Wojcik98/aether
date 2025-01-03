@{
tofs_poses = config["tofs_poses"]
}
#ifndef _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
#define _AETHER_INCLUDE_ROBOT_CONFIG_HPP_

#include <cstdint>
#include <cmath>

template <typename T> struct TofsData {
    const T right_side;
    const T right_diag;
    const T right_front;
    const T left_front;
    const T left_diag;
    const T left_side;

    const T &operator[](uint32_t i) const {
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

    Pose inverse() const {
        // calculate inverse of pose
        return {
            -x * cosf(yaw) - y * sinf(yaw),
            x * sinf(yaw) - y * cosf(yaw),
            -yaw,
        };
    }

    Pose operator*(const Pose &pose) const {
        // transform pose from local to global frame
        return {
            x + pose.x * cosf(yaw) - pose.y * sinf(yaw),
            y + pose.x * sinf(yaw) + pose.y * cosf(yaw),
            yaw + pose.yaw,
        };
    }
};
using TofsPoses = TofsData<Pose>;

static constexpr uint32_t NUM_TOFS = 6;

// Poses of the TOFs relative to the base_link frame
constexpr TofsPoses TOF_POSES = {
@{
for tof in ["right_side", "right_diag", "right_front", "left_front", "left_diag", "left_side"]:
    pose = tofs_poses[tof]
    print(f"    {{{pose[0]}f, {pose[1]}f, {pose[2]}f}},")
}@
};

constexpr float CHASSIS_LENGTH = @(config["chassis_size"][0]);
constexpr float CHASSIS_WIDTH = @(config["chassis_size"][1]);

constexpr float WHEEL_RADIUS = @(config["wheel_radius"]);
constexpr float WHEEL_BASE = @(config["wheel_base"]);

constexpr float WALL_WIDTH = @(config["wall_width"]);
constexpr float CELL_SIZE = @(config["cell_size"]);
constexpr float CELL_INNER_SIZE = @(config["cell_inner_size"]);
constexpr uint32_t MAZE_SIZE_X_CELLS = @(config["maze_size"][0]);
constexpr uint32_t MAZE_SIZE_Y_CELLS = @(config["maze_size"][1]);
constexpr float MAZE_SIZE_X = MAZE_SIZE_X_CELLS * CELL_SIZE;
constexpr float MAZE_SIZE_Y = MAZE_SIZE_Y_CELLS * CELL_SIZE;
constexpr float TOF_RANGE = @(config["tof_range"]);
constexpr uint32_t TOF_RANGE_CELLS = @(config["tof_range_cells"]);

constexpr float STARTING_X = @(config["starting_pose"][0]);
constexpr float STARTING_Y = @(config["starting_pose"][1]);
constexpr float STARTING_YAW = @(config["starting_pose"][2]);

constexpr float FREQ_IMU_ENC = @(config["freq_imu_enc"]);
constexpr float FREQ_TOFS = @(config["freq_tofs"]);
constexpr float FREQ_CONTROL = @(config["freq_control"]);

constexpr float DT_IMU_ENC = 1.0f / FREQ_IMU_ENC;
constexpr float DT_TOFS = 1.0f / FREQ_TOFS;

constexpr uint32_t NUM_PARTICLES = @(config["num_particles"]);
constexpr float NUM_EFF_PARTICLES_THRESHOLD = @(config["num_eff_particles_threshold"]);

constexpr uint32_t MAX_PATH_LENGTH = MAZE_SIZE_X_CELLS * MAZE_SIZE_Y_CELLS;

#endif // _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
