@{
tofs_poses = config["tofs_poses"]
}
#ifndef _AETHER_INCLUDE_ROBOT_CONFIG_HPP_
#define _AETHER_INCLUDE_ROBOT_CONFIG_HPP_

#include <cstdint>

#include "Eigen/Core"

#include "aether/types.hpp"

static constexpr size_t NUM_TOFS = 6;

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
constexpr uint8_t MAZE_SIZE_X = @(config["maze_size"][0]);
constexpr uint8_t MAZE_SIZE_Y = @(config["maze_size"][1]);
constexpr float TOF_RANGE = @(config["tof_range"]);
constexpr uint8_t TOF_RANGE_CELLS = @(config["tof_range_cells"]);

constexpr float STARTING_X = @(config["starting_pose"][0]);
constexpr float STARTING_Y = @(config["starting_pose"][1]);
constexpr float STARTING_YAW = @(config["starting_pose"][2]);

constexpr float FREQ_IMU_ENC = @(config["freq_imu_enc"]);
constexpr float FREQ_TOFS = @(config["freq_tofs"]);

constexpr float DT_IMU_ENC = 1.0f / FREQ_IMU_ENC;
constexpr float DT_TOFS = 1.0f / FREQ_TOFS;

#endif // _AETHER_INCLUDE_ROBOT_CONFIG_HPP_