#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

#include <gtest/gtest.h>

#include "aether/map_confident.hpp"
#include "aether/map_interface.hpp"
#include "aether/mapped_localization.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class CorridorMapMock : public MapInterface<CorridorMapMock> {
public:
    CellWalls get_cell_walls(int32_t x, int32_t y) const {
        (void)x;
        (void)y;
        // Always walls on the sides
        return {0.0f, 1.0f, 0.0f, 1.0f};
    }

    bool is_out_of_bounds(float x, float y) const {
        // Only lower bound is checked
        return x < 0.0f || y > 0.0f;
    }
};

class TofReadingTest : public ::testing::Test {
protected:
    TofReadingTest() : map_confident(map), localization(map_confident) {
        localization.set_random_seed(0);
    }

    CorridorMapMock map;
    MapConfident<CorridorMapMock> map_confident;
    MappedLocalization<CorridorMapMock> localization;
};

TEST_F(TofReadingTest, IdealDriveForwardTest) {
    constexpr uint32_t IMU_TO_TOF_RATIO = (uint32_t)(FREQ_IMU_ENC / FREQ_TOFS);
    constexpr float CENTER_TO_WALL = CELL_INNER_SIZE / 2.0f;
    const float om_wheel = 0.1f;
    ImuData empty_imu;
    EncoderData encoder_data{om_wheel, om_wheel};

    // Assuming the robot is in the middle of the corridor and yaw is 0
    const float side_tof =
        abs((CENTER_TO_WALL + TOF_POSES[0].y) / sinf(TOF_POSES[0].yaw));
    const float diag_tof =
        abs((CENTER_TO_WALL + TOF_POSES[1].y) / sinf(TOF_POSES[1].yaw));
    const float front_tof = std::numeric_limits<float>::infinity();
    const float std = 0.001f;
    const TofsReadings tofs_data = {
        {side_tof, std},  {diag_tof, std}, {front_tof, std},
        {front_tof, std}, {diag_tof, std}, {side_tof, std},
    };

    const float dx_tof_step =
        om_wheel * DT_IMU_ENC * WHEEL_RADIUS * IMU_TO_TOF_RATIO;

    // Single step
    for (uint32_t i = 0; i < IMU_TO_TOF_RATIO; ++i) {
        localization.imu_enc_update(0, empty_imu, encoder_data);
    }
    localization.tofs_update(0, tofs_data);
    auto pose = localization.get_latest_pose();

    EXPECT_NEAR(pose.x, STARTING_X + dx_tof_step, 0.001f);
    EXPECT_NEAR(pose.y, STARTING_Y, 0.001f);
    EXPECT_NEAR(pose.yaw, STARTING_YAW, 0.001f);

    // Drive for a long time
    localization.reset(0);
    const uint32_t num_steps = 100'000;
    for (uint32_t i = 0; i < num_steps; ++i) {
        for (uint32_t j = 0; j < IMU_TO_TOF_RATIO; ++j) {
            localization.imu_enc_update(0, empty_imu, encoder_data);
        }
        localization.tofs_update(0, tofs_data);
    }
    pose = localization.get_latest_pose();

    // The error in x should be relatively big, as the corridor is featureless
    EXPECT_NEAR(pose.x, STARTING_X + dx_tof_step * num_steps, 0.1f);
    // But we should be sure about the y and yaw, as the side tofs are always
    // seeing the walls
    EXPECT_NEAR(pose.y, STARTING_Y, 0.001f);
    EXPECT_NEAR(pose.yaw, STARTING_YAW, 0.001f);
}
