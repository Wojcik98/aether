#include <gtest/gtest.h>

#include "aether/map_confident.hpp"
#include "aether/map_interface.hpp"
#include "aether/mapped_localization.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class EmptyMapMock : public MapInterface<EmptyMapMock> {
public:
    CellWalls get_cell_walls(int8_t x, int8_t y) const {
        (void)x;
        (void)y;
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }
};

class MotionModelTest : public ::testing::Test {
protected:
    MotionModelTest() : map_confident(map), localization(map_confident) {
        localization.set_random_seed(0);
    }

    EmptyMapMock map;
    MapConfident<EmptyMapMock> map_confident;
    MappedLocalization<EmptyMapMock> localization;
};

TEST_F(MotionModelTest, InitTest) {
    auto pose = localization.get_latest_pose();
    EXPECT_FLOAT_EQ(pose.x, STARTING_X);
    EXPECT_FLOAT_EQ(pose.y, STARTING_Y);
    EXPECT_FLOAT_EQ(pose.yaw, STARTING_YAW);
}

TEST_F(MotionModelTest, ResetTest) {
    localization.reset(0);
    auto pose = localization.get_latest_pose();
    EXPECT_FLOAT_EQ(pose.x, STARTING_X);
    EXPECT_FLOAT_EQ(pose.y, STARTING_Y);
    EXPECT_FLOAT_EQ(pose.yaw, STARTING_YAW);
}

TEST_F(MotionModelTest, CollisionTest) {
    ImuData imu_data{0.0f, -0.4f};
    EncoderData encoder_data;
    localization.imu_enc_update(0, imu_data, encoder_data);
    EXPECT_TRUE(localization.collision_detected());
}

TEST_F(MotionModelTest, NoCollisionTest) {
    ImuData imu_data{0.0f, -0.6f};
    EncoderData encoder_data;
    localization.imu_enc_update(0, imu_data, encoder_data);
    EXPECT_FALSE(localization.collision_detected());
}

TEST_F(MotionModelTest, DriveForwardTest) {
    const float om_wheel = 0.1f;
    ImuData imu_data;
    EncoderData encoder_data{om_wheel, om_wheel};

    // Single step
    localization.imu_enc_update(0, imu_data, encoder_data);
    auto pose = localization.get_latest_pose();

    float dx = om_wheel * DT_IMU_ENC * WHEEL_RADIUS;
    EXPECT_NEAR(pose.x, STARTING_X + dx, 0.001f);
    EXPECT_NEAR(pose.y, STARTING_Y, 0.001f);
    EXPECT_NEAR(pose.yaw, STARTING_YAW, 0.001f);

    // Drive for a while
    localization.reset(0);
    const int num_steps = 500;
    for (int i = 0; i < num_steps; ++i) {
        localization.imu_enc_update(0, imu_data, encoder_data);
    }
    pose = localization.get_latest_pose();

    dx = om_wheel * num_steps * DT_IMU_ENC * WHEEL_RADIUS;
    EXPECT_NEAR(pose.x, STARTING_X + dx, 0.001f);
    EXPECT_NEAR(pose.y, STARTING_Y, 0.001f);
    EXPECT_NEAR(pose.yaw, STARTING_YAW, 0.001f);
}

TEST_F(MotionModelTest, TurnTest) {
    const float om_robot = 0.1f;
    const float om_wheel = om_robot * WHEEL_RADIUS / WHEEL_BASE;

    ImuData imu_data{om_robot};
    EncoderData encoder_data{om_wheel, -om_wheel};

    // Single step
    localization.imu_enc_update(0, imu_data, encoder_data);
    auto pose = localization.get_latest_pose();

    float dtheta = om_robot * DT_IMU_ENC;
    EXPECT_NEAR(pose.x, STARTING_X, 0.001f);
    EXPECT_NEAR(pose.y, STARTING_Y, 0.001f);
    EXPECT_NEAR(pose.yaw, STARTING_YAW + dtheta, 0.001f);

    // Turn for a while
    localization.reset(0);
    const int num_steps = 500;
    for (int i = 0; i < num_steps; ++i) {
        localization.imu_enc_update(0, imu_data, encoder_data);
    }
    pose = localization.get_latest_pose();

    dtheta = om_robot * num_steps * DT_IMU_ENC;
    EXPECT_NEAR(pose.x, STARTING_X, 0.001f);
    EXPECT_NEAR(pose.y, STARTING_Y, 0.001f);
    EXPECT_NEAR(pose.yaw, STARTING_YAW + dtheta, 0.001f);
}

TEST_F(MotionModelTest, DriveAndTurnTest) {
    constexpr float om_robot = 0.1f;
    constexpr float v_robot = 0.1f;
    constexpr float om_l =
        (v_robot - om_robot * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    constexpr float om_r =
        (v_robot + om_robot * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    constexpr float circle_radius = v_robot / om_robot;
    constexpr float circle_period = 2.0f * M_PI * circle_radius / v_robot;
    // IntelliSense thinks sinf, cosf aren't constexpr, so let's just use const.
    const float center_x = STARTING_X - circle_radius * sinf(STARTING_YAW);
    const float center_y = STARTING_Y + circle_radius * cosf(STARTING_YAW);

    ImuData imu_data{om_robot};
    EncoderData encoder_data{om_l, om_r};

    // Single step
    localization.imu_enc_update(0, imu_data, encoder_data);
    auto pose = localization.get_latest_pose();

    float expected_x =
        center_x +
        circle_radius *
            sinf(STARTING_YAW + 2.0f * M_PI * DT_IMU_ENC / circle_period);
    float expected_y =
        center_y -
        circle_radius *
            cosf(STARTING_YAW + 2.0f * M_PI * DT_IMU_ENC / circle_period);
    float expected_yaw =
        STARTING_YAW + 2.0f * M_PI * DT_IMU_ENC / circle_period;

    EXPECT_NEAR(pose.x, expected_x, 0.001f);
    EXPECT_NEAR(pose.y, expected_y, 0.001f);
    EXPECT_NEAR(pose.yaw, expected_yaw, 0.001f);

    // Drive for a while
    localization.reset(0);
    const int num_steps = 500;
    const float dt = DT_IMU_ENC;
    const float num_periods = num_steps * dt / circle_period;

    for (int i = 0; i < num_steps; ++i) {
        localization.imu_enc_update(0, imu_data, encoder_data);
    }
    pose = localization.get_latest_pose();

    expected_x = center_x +
                 circle_radius * sinf(STARTING_YAW + num_periods * 2.0f * M_PI);
    expected_y = center_y -
                 circle_radius * cosf(STARTING_YAW + num_periods * 2.0f * M_PI);
    expected_yaw = STARTING_YAW + num_periods * 2.0f * M_PI;

    EXPECT_NEAR(pose.x, expected_x, 0.001f);
    EXPECT_NEAR(pose.y, expected_y, 0.001f);
    EXPECT_NEAR(pose.yaw, expected_yaw, 0.001f);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
